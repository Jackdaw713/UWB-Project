#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h> /* pow(), sqrt() ve fabs() fonksiyonlari icin eklendi */

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ds_twr_sts_sdc_init);

#define APP_NAME "DSTWR INIT 2D TRILAT v2.1 (SADECE EMA)"
#define RNG_DELAY_MS 50 

/* ========================================================= */
/* ----------- 2D ANCHOR (SABIT NODE) KOORDINATLARI -------- */
/* ========================================================= */
#define ANCHOR_1_X 0.0  /* Node 1 (Sol Alt) */
#define ANCHOR_1_Y 0.0

#define ANCHOR_2_X 3.6  /* Node 2 (Sag Alt) - 1.2m uzakta */
#define ANCHOR_2_Y 0.0

#define ANCHOR_3_X 0.0  /* Node 3 (Sol Ust) - 0.8m yukarida */
#define ANCHOR_3_Y 1.6

#define MAX_NODES 3
static double distances[MAX_NODES] = {0.0, 0.0, 0.0};
static bool distance_ready[MAX_NODES] = {false, false, false};

/* ========================================================= */
/* --------------- X/Y KOORDINAT FILTRE AYARLARI ----------- */
/* ========================================================= */
#define EMA_ALPHA_XY 0.40      /* %40 Yeni Konum, %60 Eski Konum (Deadband Iptal) */

/* Filtre Hafizalari */
static double filtered_x = 0.0;
static double filtered_y = 0.0;
static double display_x = 0.0; /* Ekranda gosterilecek X */
static double display_y = 0.0; /* Ekranda gosterilecek Y */
static bool is_first_calc = true;
/* ========================================================= */

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_256,    /* UZUN MESAFE Preamble */
    DWT_PAC16,       /* UZUN MESAFE PAC */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* 0 to use standard 8 symbol SFD */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (257 + 8 - 8),   /* UZUN MESAFE SFD timeout */
    DWT_STS_MODE_1 | DWT_STS_MODE_SDC, /* STS mode 1 with SDC */
    DWT_STS_LEN_64,  /* STS length */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Sabit Kalibrasyon */
#define TX_ANT_DLY 16381
#define RX_ANT_DLY 16381

/* MAC Adresleri İcin Indeksler */
#define DEST_ADDR_IDX 5 
#define SRC_ADDR_IDX  7 

/* Mesaj Sablonlari */
static uint8_t tx_poll_msg[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0xAA, 0xAA, 0x21}; 
static uint8_t rx_resp_msg[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0xAA, 0xAA, 0x00, 0x00, 0x10, 0x02, 0, 0};
static uint8_t tx_final_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0xAA, 0xAA, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xAA, 0xAA, 0x00, 0x00, 0x24, 0, 0, 0, 0}; 

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define REPORT_MSG_DIST_IDX 10 

static uint8_t frame_seq_nb = 0;
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

#define POLL_TX_TO_RESP_RX_DLY_UUS 17000  
#define RESP_RX_TO_FINAL_TX_DLY_UUS 20000 
#define FINAL_TX_TO_REPORT_RX_DLY_UUS 15000 
#define RX_TIMEOUT_UUS 10000         
#define PRE_TIMEOUT 0

static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

extern dwt_txconfig_t txconfig_options;
static uint8_t current_target_id = 1; 

/* ========================================================= */
/* ---------------- 2D HESAPLAMA FONKSIYONU ---------------- */
/* ========================================================= */
void calculate_2d_position(void) {
    double d1 = distances[0]; 
    double d2 = distances[1]; 
    double d3 = distances[2]; 

    /* 1. ASAMA: HAM KOORDINAT HESAPLAMA */
    double raw_x = (pow(d1, 2) - pow(d2, 2) + pow(ANCHOR_2_X, 2)) / (2.0 * ANCHOR_2_X);
    double raw_y = (pow(d1, 2) - pow(d3, 2) + pow(ANCHOR_3_X, 2) + pow(ANCHOR_3_Y, 2) - (2.0 * raw_x * ANCHOR_3_X)) / (2.0 * ANCHOR_3_Y);

    /* 2. ASAMA: FILTRELEME (SADECE EMA) */
    if (is_first_calc) {
        /* İlk degeri direkt kabul et */
        filtered_x = raw_x;
        filtered_y = raw_y;
        display_x = raw_x;
        display_y = raw_y;
        is_first_calc = false;
    } else {
        /* Ustel Hareketli Ortalama (EMA) ile yumusat */
        filtered_x = (EMA_ALPHA_XY * raw_x) + ((1.0 - EMA_ALPHA_XY) * filtered_x);
        filtered_y = (EMA_ALPHA_XY * raw_y) + ((1.0 - EMA_ALPHA_XY) * filtered_y);

        /* DEADBAND IPTAL: Anlik yumusatilmis veriyi direkt ekrana aktar */
        display_x = filtered_x;
        display_y = filtered_y;
    }

    /* Ekrana bas */
    LOG_INF("+++ KONUM: [ X: %.2f m | Y: %.2f m ] +++", display_x, display_y);
}

int app_main(void)
{
    LOG_INF(APP_NAME);
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);

    while (!dwt_checkidlerc()) { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("INIT FAILED");
        while (1) { };
    }

    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    LOG_INF("Basliyor... 2D Trilaterasyon Modu Aktif! (Sadece XY EMA Devrede)");

    while (1) {
        tx_poll_msg[DEST_ADDR_IDX] = current_target_id;
        tx_final_msg[DEST_ADDR_IDX] = current_target_id;
        rx_resp_msg[SRC_ADDR_IDX] = current_target_id;
        rx_report_msg[SRC_ADDR_IDX] = current_target_id;

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg)+FCS_LEN, 0, 1);
        
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RX_TIMEOUT_UUS);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };
        
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                
                uint32_t final_tx_time;
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 1);
                
                dwt_setrxaftertxdelay(FINAL_TX_TO_REPORT_RX_DLY_UUS);
                dwt_setrxtimeout(RX_TIMEOUT_UUS);

                int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                if (ret == DWT_SUCCESS) {
                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };
                    
                    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (frame_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);
                        
                        rx_buffer[ALL_MSG_SN_IDX] = 0;
                        if (memcmp(rx_buffer, rx_report_msg, ALL_MSG_COMMON_LEN) == 0) {
                            
                            /* VERIYI PARCALA */
                            uint32_t dist_mm = rx_buffer[REPORT_MSG_DIST_IDX] | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 1] << 8) | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 2] << 16) | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 3] << 24);
                                              
                            double final_distance = (double)dist_mm / 1000.0;
                            
                            /* Ufak bi offset (-0.08 burada duruyor, dokunmadik) */
                            final_distance -= 0.08; 
                            if (final_distance < 0.0) final_distance = 0.0;
                            
                            uint8_t arr_idx = current_target_id - 1;
                            distances[arr_idx] = final_distance;
                            distance_ready[arr_idx] = true;

                            /* 3 Node okundugunda konumu hesapla ve FILTRELE */
                            if (distance_ready[0] && distance_ready[1] && distance_ready[2]) {
                                calculate_2d_position();
                            }
                        }
                    } else {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                    }
                } else {
                    LOG_WRN("Final Delayed TX kacirildi!");
                }
            }
        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        current_target_id++;
        if (current_target_id > MAX_NODES) {
            current_target_id = 1;
        }

        frame_seq_nb++;
        Sleep(RNG_DELAY_MS);
    }
}