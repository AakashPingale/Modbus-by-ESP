#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "mbcontroller.h"

#define TAG "MODBUS_SYS"

// ---------------- UART (USB INPUT) ----------------
#define UART_PORT UART_NUM_0
#define BUF_SIZE 512

// ---------------- MODBUS UART ----------------
#define MB_UART_PORT UART_NUM_2
#define MB_TX GPIO_NUM_17
#define MB_RX GPIO_NUM_16
#define MB_RTS GPIO_NUM_4
#define BAUD 9600

#define SLAVE_TIMEOUT_MS 1000

// ---------------- GLOBALS ----------------
typedef enum {
    MODE_NONE,
    MODE_REQUEST,
    MODE_CONTINUOUS
} system_mode_t;

system_mode_t current_mode = MODE_NONE;
bool mode_set = false;

void* master_handler = NULL;

// ---------------- MODBUS INIT ----------------
void modbus_init()
{
    mb_communication_info_t comm = {
        .ser_opts.port = MB_UART_PORT,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = BAUD,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = 0,
        .ser_opts.response_tout_ms = 1000,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    ESP_ERROR_CHECK(mbc_master_create_serial(&comm, &master_handler));

    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_TX, MB_RX, MB_RTS, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_ERROR_CHECK(mbc_master_start(master_handler));

    ESP_LOGI(TAG, "Modbus Master Initialized");
}

// ---------------- MODBUS READ ----------------
uint16_t read_modbus(uint8_t sid, uint16_t ra)
{
    uint16_t data[1] = {0};

    mb_param_request_t req = {
        .slave_addr = sid,
        .command = 0x03,
        .reg_start = ra,
        .reg_size = 1
    };

    esp_err_t err = mbc_master_send_request(master_handler, &req, data);

    if (err == ESP_OK) {
        return data[0];
    } else {
        ESP_LOGE(TAG, "Read fail SID:%d RA:%d", sid, ra);
        return 0;
    }
}

// ---------------- HELPERS ----------------
int extract_number(char *str, const char *key)
{
    char *ptr = strstr(str, key);
    if (!ptr) return -1;
    return atoi(ptr + strlen(key));
}

// MODE HANDLER
void handle_mode(char *cmd)
{
    if (strstr(cmd, "REQUEST")) {
        current_mode = MODE_REQUEST;
        mode_set = true;
        printf("MODE|REQUEST|OK\n");
    }
    else if (strstr(cmd, "CONTINUOUS")) {
        current_mode = MODE_CONTINUOUS;
        mode_set = true;
        printf("MODE|CONTINUOUS|OK\n");
    }
    else {
        printf("MODE|ERROR\n");
    }
}

// ---------------- SLAVE COMMAND ----------------
void handle_slave_command(char *cmd)
{
    if (!mode_set) {
        printf("ERROR|MODE_NOT_SET\n");
        return;
    }

    int sid = extract_number(cmd, "SLAVE_ID:");

    printf("SLAVE_ID:%d", sid);

    char *block = cmd;
    int block_count = 0; // [MODIFIED] Added 'block_count' counter for up to 10 limits

    // [MODIFIED] Added '&& block_count < 10' condition to limit commands to B1..B10 limit
    while ((block = strstr(block, "{")) != NULL && block_count < 10)
    {
        char name[5] = {0};
        int ra = extract_number(block, "RA:");
        int rc = extract_number(block, "RC:"); // [MODIFIED] Added RC extraction command to parse the count.

        // Currently we read 1 value per query as defined in your read_modbus
        // but we extract RC to ensure it doesn't break parsing.
        
        // [MODIFIED] Added '4' inside sscanf %4[^|] to prevent overflowing memory beyond 4 length blocks (B10 is 3 length).
        sscanf(block, "{%4[^|]", name);  // Extract B1, B2...

        uint16_t value = read_modbus(sid, ra);

        // [MODIFIED] Output format updated to output exactly: |{B1|RA:123}
        printf("|{%s|RA:%d}", name, value);

        block++; // move forward
        block_count++; // [MODIFIED] Increment block tracking index
    }

    printf("\n");
}

// ---------------- COMMAND PROCESSOR ----------------
void process_command(char *cmd)
{
    if (strncmp(cmd, "MODE|SET|", 9) == 0) {
        handle_mode(cmd);
    }
    else if (strncmp(cmd, "SLAVE_ID:", 9) == 0) {
        handle_slave_command(cmd);
    }
    else {
        printf("UNKNOWN_CMD\n");
    }
}

// ---------------- UART TASK ----------------
void uart_task(void *arg)
{
    char line[512];
    int idx = 0;

    printf("\n>> Ready for Commands\n");

    while (1)
    {
        uint8_t ch;

        if (uart_read_bytes(UART_PORT, &ch, 1, portMAX_DELAY))
        {
            if (ch == '\n' || ch == '\r')
            {
                if (idx > 0)
                {
                    line[idx] = '\0';

                    ESP_LOGI(TAG, "CMD: %s", line);

                    process_command(line);

                    idx = 0;
                }
            }
            else
            {
                if (idx < sizeof(line) - 1)
                    line[idx++] = ch;
            }
        }
    }
}

// ---------------- MAIN ----------------
void app_main(void)
{
    // UART for terminal
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    modbus_init();

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}