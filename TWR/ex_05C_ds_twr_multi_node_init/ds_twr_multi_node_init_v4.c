#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h> /* fabs() fonksiyonu icin */

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ds_twr_sts_sdc_init);

#define APP_NAME "DSTWR INIT MULTI-NODE v2.1 (FILTERED)"
#define RNG_DELAY_MS 50 

/* ========================================================= */
/* --- HIBRIT FILTRE PARAMETRELERI (TEST EDEREK DEGISTIR) -- */
/* ========================================================= */
#define MAX_VALID_JUMP 0.80  /* 80 cm'den buyuk ani sicramalari cope at */
#define EMA_ALPHA 0.55       /* %30 Yeni Veri, %70 Eski Veri (Lag/Smoothness ayari) */
#define MAX_NODES 3          /* Sistemdeki Responder Sayisi */

/* Filtre Hafizasi (ID 1, 2, 3 icin index 0, 1, 2 kullanilir) */
static double filtered_distances[MAX_NODES] = {0.0, 0.0, 0.0};
static bool is_first_reading[MAX_NODES] = {true, true, true};
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

/* Hassas Anten Kalibrasyonu (Initiator icin 16381 kalmali) */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

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

/* 4 Adimli Yeni Zamanlamalar */
#define POLL_TX_TO_RESP_RX_DLY_UUS 17000  
#define RESP_RX_TO_FINAL_TX_DLY_UUS 20000 
#define FINAL_TX_TO_REPORT_RX_DLY_UUS 15000 
#define RX_TIMEOUT_UUS 10000         
#define PRE_TIMEOUT 0

static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

extern dwt_txconfig_t txconfig_options;

/* BAŞLANGIC HEDEFİ: 1 */
static uint8_t current_target_id = 1; 

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

    LOG_INF("Basliyor... Veriler Initiator ekranina dusecek! (Filtre Devrede!)");

    while (1) {
        /* Adresleri Guncelle (Hedefi Sec) */
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

        /* 1. ADIM BEKLEMESI: Response bekle */
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
                    /* 2. ADIM BEKLEMESI: 4. mesaj olan Raporu Bekle */
                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };
                    
                    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (frame_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);
                        
                        rx_buffer[ALL_MSG_SN_IDX] = 0;
                        if (memcmp(rx_buffer, rx_report_msg, ALL_MSG_COMMON_LEN) == 0) {
                            
                            /* VERIYI 32-BIT PARCALAMA (PARSING) */
                            uint32_t dist_mm = rx_buffer[REPORT_MSG_DIST_IDX] | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 1] << 8) | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 2] << 16) | 
                                              (rx_buffer[REPORT_MSG_DIST_IDX + 3] << 24);
                                              
                            double raw_dist = (double)dist_mm / 1000.0;
                            
                            /* ===================================================== */
                            /* ------------- HIBRIT FILTRE UYGULAMASI -------------- */
                            /* ===================================================== */
                            uint8_t arr_idx = current_target_id - 1; // Arrayler 0'dan baslar
                            
                            if (is_first_reading[arr_idx]) {
                                /* Cihazdan gelen ilk mantikli veriyi direkt kabul et */
                                if(raw_dist > 0.0 && raw_dist < 100.0) {
                                    filtered_distances[arr_idx] = raw_dist;
                                    is_first_reading[arr_idx] = false;
                                }
                            } else {
                                /* 1. ASAMA: AYKIRI DEGER ENGELLEME (OUTLIER REJECTION) */
                                double diff = fabs(raw_dist - filtered_distances[arr_idx]);
                                
                                if (diff < MAX_VALID_JUMP) {
                                    /* 2. ASAMA: USTEL HAREKETLI ORTALAMA (EMA) */
                                    filtered_distances[arr_idx] = (EMA_ALPHA * raw_dist) + ((1.0 - EMA_ALPHA) * filtered_distances[arr_idx]);
                                } else {
                                    /* Sapan veri yakalandi! Veriyi reddet, ekrana log bas */
                                    LOG_WRN("[ID: %d] Sicrama onlendi! Ham: %.2fm, Beklenen: %.2fm", 
                                            current_target_id, raw_dist, filtered_distances[arr_idx]);
                                }
                            }

                            /* Filtrelenmis tertemiz mesafeyi ekrana bas */
                            if (!is_first_reading[arr_idx]) {
                                LOG_INF(">>> [ID: %d] MESAFE: %3.2f m <<<", current_target_id, filtered_distances[arr_idx]);
                            }
                            /* ===================================================== */

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

        /* --- DÖNGÜ MANTIĞI: 1 -> 2 -> 3 -> 1 --- */
        current_target_id++;
        if (current_target_id > MAX_NODES) {
            current_target_id = 1;
        }

        frame_seq_nb++;
        Sleep(RNG_DELAY_MS);
    }
}