#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "mbcontroller.h"

#define TAG "MODBUS_SYS"

// ---------------- UART CONFIG ----------------
#define UART_PORT UART_NUM_0
#define BUF_SIZE 512

// ---------------- MODBUS CONFIG ----------------
#define MB_UART_PORT UART_NUM_2
#define MB_TX GPIO_NUM_17
#define MB_RX GPIO_NUM_16
#define MB_RTS GPIO_NUM_4

#define BAUD_RATE 9600

// ---------------- GLOBAL ----------------
void* master_handler = NULL;

// ---------------- MODBUS INIT ----------------
void modbus_init(void)
{
    mb_communication_info_t comm = {
        .ser_opts.port = MB_UART_PORT,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = BAUD_RATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = 0,
        .ser_opts.response_tout_ms = 1000,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    ESP_ERROR_CHECK(mbc_master_create_serial(&comm, &master_handler));

    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_TX, MB_RX, MB_RTS, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(mbc_master_start(master_handler));

    ESP_LOGI(TAG, "Modbus Master Initialized");
}

// ---------------- HELPER ----------------
int extract_number(char *str, const char *key)
{
    char *ptr = strstr(str, key);
    if (!ptr) return -1;
    return atoi(ptr + strlen(key));
}

void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
        uart_write_bytes(UART_PORT, buf, len);
}

// ---------------- SLAVE COMMAND ----------------
void handle_slave_command(char *cmd)
{
    int sid = extract_number(cmd, "SLAVE_ID:");

    uart_printf("SLAVE_ID:%d", sid);

    char *block = cmd;
    int block_count = 0;

    while ((block = strstr(block, "{")) != NULL && block_count < 10)
    {
        char name[5] = {0};
        int ra = extract_number(block, "RA:");
        int rc = extract_number(block, "RC:");

        sscanf(block, "{%4[^|]", name);

        uint16_t data[2] = {0};
        esp_err_t err;

        // 🔥 CASE: RC = 2 (32-bit read)
        if (rc == 2)
        {
            mb_param_request_t req = {
                .slave_addr = sid,
                .command = 0x03,
                .reg_start = ra,
                .reg_size = 2
            };

            err = mbc_master_send_request(master_handler, &req, data);

            if (err == ESP_OK)
            {
                uint32_t combined = ((uint32_t)data[0] << 16) | data[1];

                float value;
                memcpy(&value, &combined, sizeof(float));

                uart_printf("|{%s:%.3f}", name, value);
            }
            else
            {
                uart_printf("|{%s|ERROR}", name);
            }
        }

        // 🔥 CASE: RC = 1 (single register)
        else if (rc == 1)
        {
            uint16_t val = 0;

            mb_param_request_t req = {
                .slave_addr = sid,
                .command = 0x03,
                .reg_start = ra,
                .reg_size = 1
            };

            err = mbc_master_send_request(master_handler, &req, &val);

            // fallback for paired-register devices
            if (err != ESP_OK)
            {
                uint16_t temp[2];

                mb_param_request_t fallback = {
                    .slave_addr = sid,
                    .command = 0x03,
                    .reg_start = ra,
                    .reg_size = 2
                };

                if (mbc_master_send_request(master_handler, &fallback, temp) == ESP_OK)
                {
                    uart_printf("|{%s:%d}", name, temp[0]);
                }
                else
                {
                    uart_printf("|{%s|ERROR}", name);
                }
            }
            else
            {
                uart_printf("|{%s:%d}", name, val);
            }
        }
        else
        {
            uart_printf("|{%s|INVALID_RC}", name);
        }

        block++;
        block_count++;
    }

    uart_printf("\n");
}

// ---------------- COMMAND PROCESSOR ----------------
void process_command(char *cmd)
{
    if (strncmp(cmd, "SLAVE_ID:", 9) == 0)
    {
        handle_slave_command(cmd);
    }
    else
    {
        uart_printf("UNKNOWN_CMD\n");
    }
}

// ---------------- UART TASK ----------------
void uart_task(void *arg)
{
    char line[512];
    int idx = 0;

    uart_printf("\r\n>> ");

    while (1)
    {
        uint8_t ch;

        if (uart_read_bytes(UART_PORT, &ch, 1, portMAX_DELAY))
        {
            if (ch == '\r')
            {
                uart_write_bytes(UART_PORT, "\r\n", 2);

                if (idx > 0)
                {
                    line[idx] = '\0';
                    process_command(line);
                    idx = 0;
                }

                uart_printf(">> ");
            }
            else if (ch == 0x08 || ch == 0x7F)
            {
                if (idx > 0)
                {
                    idx--;
                    uart_write_bytes(UART_PORT, "\b \b", 3);
                }
            }
            else if (ch >= 32 && ch <= 126)
            {
                if (idx < sizeof(line) - 1)
                {
                    line[idx++] = ch;
                    uart_write_bytes(UART_PORT, (char *)&ch, 1);
                }
            }
        }
    }
}

// ---------------- MAIN ----------------
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE); // clean terminal

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    modbus_init();

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}