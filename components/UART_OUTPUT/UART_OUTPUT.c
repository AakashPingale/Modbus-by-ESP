#include "uart_comm.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_0
#define BUF_SIZE 1024   // Increased for JSON

static void (*command_callback)(char *cmd) = NULL;

// PRINT FUNCTION
void uart_comm_printf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
        uart_write_bytes(UART_PORT, buf, len);
}

// UART TASK
static void uart_task(void *arg)
{
    char line[1024];
    int idx = 0;

    while (1)
    {
        uint8_t ch;

        if (uart_read_bytes(UART_PORT, &ch, 1, portMAX_DELAY))
        {
            // ENTER (\r or \n)
            if (ch == '\r' || ch == '\n')
            {
                // NEW CODE (ECHO)
                uart_write_bytes(UART_PORT, "\r\n", 2);
                
                if (idx > 0 && command_callback)
                {
                    line[idx] = '\0';

                    // Clean trailing chars
                    while (idx > 0 && 
                          (line[idx - 1] == '\n' || 
                           line[idx - 1] == '\r' || 
                           line[idx - 1] == ' '))
                    {
                        line[--idx] = '\0';
                    }

                    command_callback(line);
                    idx = 0;
                }
            }
            // BACKSPACE
            else if (ch == 0x08 || ch == 0x7F)
            {
                // NEW CODE (ECHO)
                if (idx > 0)
                {
                    idx--;
                    const char bs[] = {0x08, ' ', 0x08};
                    uart_write_bytes(UART_PORT, bs, 3);
                }
                
                /*
                // OLD CODE
                if (idx > 0)
                    idx--;
                */
            }
            // PRINTABLE CHAR
            else if (ch >= 32 && ch <= 126)
            {
                if (idx < sizeof(line) - 1)
                {
                    line[idx++] = ch;
                    
                    // NEW CODE (ECHO)
                    uart_write_bytes(UART_PORT, (const char*)&ch, 1);
                }
            }
        }
    }
}

// INIT
void uart_comm_init(void)
{
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    xTaskCreate(uart_task, "uart_task", 8192, NULL, 5, NULL); // NEW CODE 
    /* xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL); // OLD CODE */
}

// CALLBACK
void uart_comm_set_callback(void (*cb)(char *cmd))
{
    command_callback = cb;
}