#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * AutoKart CAN/TWAI two-node bench test.
 *
 * Defaults:
 *   ESP32-S3: node 1, TX GPIO16, RX GPIO17
 *   XIAO C3:  node 2, TX GPIO3,  RX GPIO4
 *
 * These GPIOs connect to the SN65HVD230 transceiver logic pins. They do not
 * connect directly to CAN_H/CAN_L. Use one transceiver at each node.
 */

#if CONFIG_IDF_TARGET_ESP32S3
#define AUTOKART_NODE_ID 1
#define CAN_TX_GPIO GPIO_NUM_16
#define CAN_RX_GPIO GPIO_NUM_17
#elif CONFIG_IDF_TARGET_ESP32C3
#define AUTOKART_NODE_ID 2
#define CAN_TX_GPIO GPIO_NUM_3
#define CAN_RX_GPIO GPIO_NUM_4
#else
#error "This test supports ESP32-S3 and ESP32-C3 only"
#endif

#define CAN_BITRATE 500000
#define RX_QUEUE_LENGTH 24
#define AUTO_FRAME_ID_BASE 0x100

static const char *TAG = "AUTOKART_CAN";

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    bool is_extended;
} can_packet_t;

static QueueHandle_t rx_queue;
static twai_node_handle_t can_node;
static volatile uint32_t rx_overruns;
static volatile bool auto_send_enabled = true;

static bool on_rx_done(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *event,
                       void *user_ctx)
{
    (void)event;
    (void)user_ctx;

    uint8_t buffer[8] = {0};
    twai_frame_t frame = {
        .buffer = buffer,
        .buffer_len = sizeof(buffer),
    };
    BaseType_t task_woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        can_packet_t packet = {
            .id = frame.header.id,
            .len = frame.header.dlc,
            .is_extended = frame.header.ide,
        };

        if (packet.len <= sizeof(packet.data)) {
            memcpy(packet.data, buffer, packet.len);
            if (xQueueSendFromISR(rx_queue, &packet, &task_woken) != pdTRUE) {
                rx_overruns++;
            }
        }
    }

    return task_woken == pdTRUE;
}

static void print_packet(const char *prefix, const can_packet_t *packet)
{
    printf("%s ID=0x%lX %s DLC=%u DATA=",
           prefix,
           (unsigned long)packet->id,
           packet->is_extended ? "EXT" : "STD",
           packet->len);

    for (uint8_t i = 0; i < packet->len; i++) {
        printf("%02X", packet->data[i]);
        if (i + 1 < packet->len) {
            putchar(' ');
        }
    }
    putchar('\n');
}

static esp_err_t send_packet(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (id > 0x7FF || len > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_frame_t frame = {
        .header.id = id,
        .buffer = (uint8_t *)data,
        .buffer_len = len,
    };

    esp_err_t err = twai_node_transmit(can_node, &frame, 100);
    if (err == ESP_OK) {
        err = twai_node_transmit_wait_all_done(can_node, 200);
    }
    return err;
}

static void send_auto_frame(void)
{
    static uint8_t counter;
    uint8_t payload[2] = {AUTOKART_NODE_ID, counter++};
    esp_err_t err = send_packet(
        AUTO_FRAME_ID_BASE + AUTOKART_NODE_ID,
        payload,
        sizeof(payload)
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "automatic TX failed: %s", esp_err_to_name(err));
    } else {
        printf("TX AUTO node=%d count=%u\n", AUTOKART_NODE_ID, payload[1]);
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    can_packet_t packet;

    while (true) {
        if (xQueueReceive(rx_queue, &packet, portMAX_DELAY) == pdTRUE) {
            print_packet("RX", &packet);
        }
    }
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  help\n");
    printf("  send <id> <hex bytes>  e.g. send 0x120 01 02 AA\n");
    printf("  ping                   send a test frame\n");
    printf("  auto on|off            periodic frame every 1 s\n");
    printf("  status                 show node and RX state\n");
    printf("  reset                  reset the ESP\n\n");
}

static bool parse_id(const char *text, uint32_t *id)
{
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 0);

    if (end == text || *end != '\0' || value > 0x7FF) {
        return false;
    }
    *id = (uint32_t)value;
    return true;
}

static bool parse_byte(const char *text, uint8_t *value)
{
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 16);

    if (end == text || *end != '\0' || parsed > 0xFF) {
        return false;
    }
    *value = (uint8_t)parsed;
    return true;
}

static void handle_command(char *line)
{
    char *command = strtok(line, " \t\r\n");
    if (!command) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        print_help();
        return;
    }

    if (strcmp(command, "ping") == 0) {
        uint8_t payload[2] = {AUTOKART_NODE_ID, 0xAA};
        esp_err_t err = send_packet(0x010 + AUTOKART_NODE_ID,
                                     payload, sizeof(payload));
        printf("PING: %s\n", esp_err_to_name(err));
        return;
    }

    if (strcmp(command, "auto") == 0) {
        char *value = strtok(NULL, " \t\r\n");
        if (value && strcmp(value, "on") == 0) {
            auto_send_enabled = true;
            printf("AUTO TX ON\n");
        } else if (value && strcmp(value, "off") == 0) {
            auto_send_enabled = false;
            printf("AUTO TX OFF\n");
        } else {
            printf("Usage: auto on|off\n");
        }
        return;
    }

    if (strcmp(command, "status") == 0) {
        printf("NODE=%d BITRATE=%d TX=GPIO%d RX=GPIO%d AUTO=%s RX_OVERRUNS=%lu\n",
               AUTOKART_NODE_ID, CAN_BITRATE, CAN_TX_GPIO, CAN_RX_GPIO,
               auto_send_enabled ? "ON" : "OFF",
               (unsigned long)rx_overruns);
        return;
    }

    if (strcmp(command, "reset") == 0) {
        esp_restart();
        return;
    }

    if (strcmp(command, "send") == 0) {
        char *id_text = strtok(NULL, " \t\r\n");
        uint32_t id;
        uint8_t payload[8];
        uint8_t len = 0;

        if (!id_text || !parse_id(id_text, &id)) {
            printf("Usage: send <hex_or_decimal_id> <byte> ...\n");
            return;
        }

        char *byte_text;
        while ((byte_text = strtok(NULL, " \t\r\n")) != NULL) {
            if (len >= sizeof(payload) || !parse_byte(byte_text, &payload[len])) {
                printf("Invalid payload; use up to 8 hexadecimal bytes.\n");
                return;
            }
            len++;
        }

        esp_err_t err = send_packet(id, payload, len);
        printf("TX command: %s\n", esp_err_to_name(err));
        return;
    }

    printf("Unknown command. Type 'help'.\n");
}

static void console_task(void *arg)
{
    (void)arg;
    char line[128];
    print_help();

    while (true) {
        printf("CAN> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) != NULL) {
            handle_command(line);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void auto_tx_task(void *arg)
{
    (void)arg;
    while (true) {
        if (auto_send_enabled) {
            send_auto_frame();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(can_packet_t));
    if (!rx_queue) {
        ESP_LOGE(TAG, "RX queue allocation failed");
        return;
    }

    twai_onchip_node_config_t config = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 8,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&config, &can_node));

    twai_event_callbacks_t callbacks = {
        .on_rx_done = on_rx_done,
    };
    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(can_node, &callbacks, NULL)
    );
    ESP_ERROR_CHECK(twai_node_enable(can_node));

    printf("\nAutoKart CAN/TWAI test ready\n");
    printf("NODE=%d BITRATE=%d TX=GPIO%d RX=GPIO%d\n",
           AUTOKART_NODE_ID, CAN_BITRATE, CAN_TX_GPIO, CAN_RX_GPIO);
    printf("Use SN65HVD230 transceivers and 120-ohm termination at both bus ends.\n");

    xTaskCreate(rx_task, "can_rx", 4096, NULL, 8, NULL);
    xTaskCreate(console_task, "can_console", 4096, NULL, 5, NULL);
    xTaskCreate(auto_tx_task, "can_auto_tx", 3072, NULL, 5, NULL);
}
