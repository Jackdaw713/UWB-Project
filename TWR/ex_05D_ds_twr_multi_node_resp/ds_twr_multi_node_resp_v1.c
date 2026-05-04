#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ds_twr_sts_sdc_resp);

#define APP_NAME "DSTWR RESP MULTI-NODE v2.0"

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   DİKKAT: BU KARTI IKINCI MODÜLE YÜKLERKEN BURAYI 2 YAP !!!
   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
#define MY_RESPONDER_ID 1   

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

/* Hassas Anten Kalibrasyonu */
#define TX_ANT_DLY 16381
#define RX_ANT_DLY 16381

/* MAC Adresleri İcin Indeksler */
#define DEST_ADDR_IDX 5 
#define SRC_ADDR_IDX  7 

/* Mesaj Sablonlari */
static uint8_t rx_poll_msg[]   = {0x41, 0x88, 0, 0xCA, 0xDE, MY_RESPONDER_ID, 0x00, 0xAA, 0xAA, 0x21};
static uint8_t tx_resp_msg[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0xAA, 0xAA, MY_RESPONDER_ID, 0x00, 0x10, 0x02, 0, 0};
static uint8_t rx_final_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, MY_RESPONDER_ID, 0x00, 0xAA, 0xAA, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* 4. ADIM YENI MESAJ: Mesafe Raporu (Initiator'a yollanacak) */
static uint8_t tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xAA, 0xAA, MY_RESPONDER_ID, 0x00, 0x24, 0, 0}; // Son iki byte mesafe icin bos birakildi!

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define REPORT_MSG_DIST_IDX 10 // Mesafenin paket icindeki yeri

static uint8_t frame_seq_nb = 0;
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

/* 4 Adimli Yeni Zamanlamalar */
#define POLL_RX_TO_RESP_TX_DLY_UUS 20000  
#define RESP_TX_TO_FINAL_RX_DLY_UUS 17000 
#define FINAL_RX_TO_REPORT_TX_DLY_UUS 18000 /* YENI: Final geldikten ne kadar sure sonra Raporu firlatayim? */
#define RX_TIMEOUT_UUS 10000        
#define PRE_TIMEOUT 0

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static double tof;
static double distance;

extern dwt_txconfig_t txconfig_options;

int app_main(void)
{
    LOG_INF(APP_NAME);
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2); 

    while (!dwt_checkidlerc()) { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { while (1) { }; }
    
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    LOG_INF("Responder [ID: %d] Hazir ve Dinliyor!", MY_RESPONDER_ID);

    while (1) {
        dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* 1. ADIM BEKLEMESI: Poll bekle */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            int16_t stsqual;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            if (dwt_readstsquality(&stsqual)) {
                uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                if (frame_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;
                
                /* Eger gelen mesaj bana (ID 1) gelmisse ve POLL ise: */
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
                    
                    uint32_t resp_tx_time;
                    poll_rx_ts = get_rx_timestamp_u64();
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);
                    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                    dwt_setrxtimeout(RX_TIMEOUT_UUS);

                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_resp_msg)+FCS_LEN, 0, 1);
                    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                    
                    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
                    if (ret == DWT_ERROR) continue;

                    /* 2. ADIM BEKLEMESI: Final bekle */
                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };

                    frame_seq_nb++;

                    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                        if (dwt_readstsquality(&stsqual)) {
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                            if (frame_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, frame_len, 0);

                            rx_buffer[ALL_MSG_SN_IDX] = 0;
                            if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
                                
                                /* MATEMATIK KISMI */
                                uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                                double Ra, Rb, Da, Db;
                                int64_t tof_dtu;

                                resp_tx_ts = get_tx_timestamp_u64();
                                final_rx_ts = get_rx_timestamp_u64();

                                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                                poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                                resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                                final_rx_ts_32 = (uint32_t)final_rx_ts;
                                
                                Ra = (double)(resp_rx_ts - poll_tx_ts);
                                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                                Da = (double)(final_tx_ts - resp_rx_ts);
                                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                                
                                tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
                                tof = tof_dtu * DWT_TIME_UNITS;
                                distance = tof * SPEED_OF_LIGHT;

                                /* YENI ADIM: VERIYI PAKETLE VE INITIATORA GERI FIRLAT (REPORT) */
                                uint16_t dist_mm = (uint16_t)(distance * 1000.0); // Float'i Fixed-Point Integer yap! (Mesela 5984 mm)
                                tx_report_msg[REPORT_MSG_DIST_IDX] = dist_mm & 0xFF;         // Alt byte'i pakete koy
                                tx_report_msg[REPORT_MSG_DIST_IDX + 1] = (dist_mm >> 8) & 0xFF; // Ust byte'i pakete koy
                                
                                uint32_t report_tx_time = (final_rx_ts + (FINAL_RX_TO_REPORT_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                                dwt_setdelayedtrxtime(report_tx_time);

                                tx_report_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                                dwt_writetxdata(sizeof(tx_report_msg), tx_report_msg, 0);
                                dwt_writetxfctrl(sizeof(tx_report_msg)+FCS_LEN, 0, 1);
                                
                                int rep_ret = dwt_starttx(DWT_START_TX_DELAYED); // Sadece atip unutuyor, cevap beklemiyor!
                                
                                if (rep_ret == DWT_SUCCESS) {
                                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { };
                                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                                } else {
                                    // Log atmaya gerek yok, bir sonraki turda hallederiz.
                                }
                            }
                        }
                    } else {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                    }
                }
            } 
        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
    }
}