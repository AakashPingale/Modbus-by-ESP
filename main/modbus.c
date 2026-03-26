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
int polling_interval = 5000;   // default 5 sec
int current_baud = 9600;       // default baud

// ---------------- GLOBAL ----------------
void* master_handler = NULL;


typedef enum {
    MODE_NONE,
    MODE_REQUEST,
    MODE_CONTINUOUS
} system_mode_t;

system_mode_t current_mode = MODE_NONE;

char last_command[512] = {0};
bool continuous_running = false;
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
void handle_mode_command(char *cmd)
{
    if (strstr(cmd, "REQUEST"))
    {
        current_mode = MODE_REQUEST;
        continuous_running = false;
        uart_printf("MODE|REQUEST|OK\n");
    }
    else if (strstr(cmd, "CONTINUOUS"))
    {
        current_mode = MODE_CONTINUOUS;
        continuous_running = true;
        uart_printf("MODE|CONTINUOUS|OK\n");
    }
    else if (strstr(cmd, "GET"))
    {
        if (current_mode == MODE_REQUEST)
            uart_printf("MODE|CURRENT|REQUEST\n");
        else if (current_mode == MODE_CONTINUOUS)
            uart_printf("MODE|CURRENT|CONTINUOUS\n");
        else
            uart_printf("MODE|CURRENT|NONE\n");
    }
    else if (strstr(cmd, "STOP"))
    {
        continuous_running = false;
        uart_printf("MODE|STOPPED\n");
    }
    else
    {
        uart_printf("MODE|ERROR\n");
    }
}

/// Time Handler 
void handle_time_command(char *cmd)
{
    int sid = extract_number(cmd, "SLAVE_ID:");
    int time_sec = extract_number(cmd, "TIME:");

    if (time_sec > 0)
    {
        polling_interval = time_sec * 1000;
        uart_printf("SLAVE_ID:%d|TIME:%d|OK\n", sid, time_sec);
    }
    else
    {
        uart_printf("SLAVE_ID:%d|TIME|ERROR\n", sid);
    }
}
// Baud rate Handler
void handle_speed_command(char *cmd)
{
    int sid = extract_number(cmd, "SLAVE_ID:");
    int baud = extract_number(cmd, "SPEED:");

    if (baud > 0)
    {
        current_baud = baud;

        uart_set_baudrate(MB_UART_PORT, baud);

        uart_printf("SLAVE_ID:%d|SPEED:%d|OK\n", sid, baud);
    }
    else
    {
        uart_printf("SLAVE_ID:%d|SPEED|ERROR\n", sid);
    }
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
    //  MODE command
    if (strncmp(cmd, "MODE|", 5) == 0)
    {
        handle_mode_command(cmd);
    }

    //  TIME command (separate config)
    else if (strstr(cmd, "|TIME:"))
    {
        handle_time_command(cmd);
    }

    //  SPEED command (baud rate)
    else if (strstr(cmd, "|SPEED:"))
    {
        handle_speed_command(cmd);
    }

    //  MODBUS read command
    else if (strncmp(cmd, "SLAVE_ID:", 9) == 0)
    {
        strcpy(last_command, cmd);

        if (current_mode == MODE_REQUEST)
        {
            handle_slave_command(cmd);
        }
        else if (current_mode == MODE_CONTINUOUS)
        {
            continuous_running = true;
        }
        else
        {
            uart_printf("ERROR|SET_MODE_FIRST\n");
        }
    }

    //  Unknown command
    else
    {
        uart_printf("UNKNOWN_CMD\n");
    }
}
// void process_command(char *cmd)
// {
//     if (strncmp(cmd, "MODE|", 5) == 0)
//     {
//         handle_mode_command(cmd);
//     }
//     else if (strncmp(cmd, "SLAVE_ID:", 9) == 0)
//     {
//         strcpy(last_command, cmd);

//         if (current_mode == MODE_REQUEST)
//         {
//             handle_slave_command(cmd);
//         }
//         else if (current_mode == MODE_CONTINUOUS)
//         {
//             continuous_running = true;
//         }
//         else
//         {
//             uart_printf("ERROR|SET_MODE_FIRST\n");
//         }
//     }
//     else
//     {
//         uart_printf("UNKNOWN_CMD\n");
//     }
// }

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
void continuous_task(void *arg)
{
    while (1)
    {
        if (continuous_running && current_mode == MODE_CONTINUOUS)
        {
            handle_slave_command(last_command);
            vTaskDelay(pdMS_TO_TICKS(polling_interval));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(200));
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
    xTaskCreate(continuous_task, "continuous_task", 4096, NULL, 5, NULL);
}