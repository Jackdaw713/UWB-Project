/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/random/random.h>
#include <stdio.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static void notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);
    printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    printk("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
}

struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) { printk("Baglanti hatasi (err %u)\n", err); }
    else { printk("Baglandi.\n"); }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Baglanti koptu (reason %u). Yeniden yayin yapiliyor...\n", reason);
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Yayin baslatilamadi: %d\n", err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

int main(void)
{
    int err;
    char buffer[64];

    printk("Sample - Bluetooth Peripheral NUS (JSON Format)\n");

    err = bt_nus_cb_register(&nus_listener, NULL);
    if (err) {
        printk("Failed to register NUS callback: %d\n", err);
        return err;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Failed to enable bluetooth: %d\n", err);
        return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Failed to start advertising: %d\n", err);
        return err;
    }

    printk("Initialization complete\n");

   while (true) {
        int fake_x = sys_rand32_get() % 500;
        int fake_y = sys_rand32_get() % 500;

        snprintf(buffer, sizeof(buffer), "{\"x\":%d,\"y\":%d}\n", fake_x, fake_y);

        err = bt_nus_send(NULL, buffer, strlen(buffer));
        
        if (err) {
            // Dongle henüz hazır değilse veya buffer doluysa paketi çöpe at.
            // Sistemi kilitlememek için uykuya geçip yeni paketi bekle.
            printk("Gonderim atlandi (Hata: %d)\n", err);
        } else {
            printk("Gonderildi: %s", buffer);
        }

        // Her halükarda 1 saniye bekle
        k_sleep(K_MSEC(1000));
    }

    return 0;
}