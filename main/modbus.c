    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <stdarg.h>
    #include <stdbool.h>
    #include <time.h>

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"

    #include "esp_log.h"
    #include "esp_err.h"
    #include "esp_sntp.h"

   
    #include "driver/uart.h"
    #include "driver/gpio.h"

    #include "mbcontroller.h"
    // To communicate with the modbus device
    #include "uart_comm.h"
    // To parse the json
    #include "cJSON.h"
    // To evaluate the expression
    #include "tinyexpr.h"
    // To access the flash memory
    #include "nvs_flash.h"
    #include "nvs.h"

    #include "custom_mqtt.h"
    #include "wifi.h"

    #include "esp_sntp.h"

    #define GET_INT(obj, key, var) \
    do { \
        cJSON *tmp = cJSON_GetObjectItem(obj, key); \
        if (tmp && cJSON_IsNumber(tmp)) var = tmp->valueint; \
        else ESP_LOGW(TAG, "Invalid or missing key: %s", key); \
    } while(0)
    // function declarations
    void replace_str(char *str, const char *old_str, const char *new_str);
    void process_command(char *cmd);
    void execute_cycle(void);
    void init_sntp(void);
    void get_timestamp(char *buf, size_t len);


// ================= DEFAULT CONFIG =================
const char *default_config =
"{"
"\"settings\":{\"slave_id\":1,\"baud_rate\":9600,\"parity\":\"none\",\"stop_bits\":1,\"data_bits\":8,\"function_code\":3},"
"\"mode\":{\"auto\":true,\"request\":false},"
"\"read\":{\"register_address\":2,\"register_count\":3},"
"\"variables\":{\"co2\":\"float\",\"temp\":\"float\",\"hum\":\"float\"},"
"\"formula\":{\"co2\":\"register[0]\",\"temp\":\"(register[1]-500)*0.01\",\"hum\":\"(register[2]-500)*0.01\"},"
"\"send\":{"
"\"method\":\"mqtt\","
"\"url\":\"mqtt://test.mosquitto.org:1883\","
"\"topic\":\"co2sensor/1456/12345/data\","
"\"qos\":0,"
"\"retain\":0,"
"\"payload\":{"
"\"co2\":\"{{co2}}\","
"\"temperature\":\"{{temp}}\","
"\"humidity\":\"{{hum}}\","
"\"time\":\"{{timestamp}}\""
"}},"
"\"delay\":3"
"}";

// global variables for modbus
    int reg_addr = 0;
    int reg_count = 0;


    #define TAG "MODBUS_JSON"
    #define MAX_VARS 10

// Structure to store formula configuration
    typedef struct {
    char name[32];
    char expr[128];
    char type[16];
    double last_value;
} formula_config_t;

// GLOBAL STORAGE
formula_config_t formulas[MAX_VARS];
int formula_count = 0;
    //config initialization flag    
    bool config_initialized = false;



    //static int backup_reg_addr;
    //static int backup_reg_count;
    //static int backup_formula_count;

    
    // ERROR FLAG
    bool system_error=false;

    // Flag to prevent concurrent execution
    bool is_executing = false;
    
    // function to store the json for permenant
    char stored_json[1024];
    bool config_available = false;
    // defining operation mode
    char operation_mode[20]="auto"; //default
    //  auto mode declaration
    bool auto_mode = false;

    bool first_run = true;  
    // UART alias
    #define uart_printf uart_comm_printf

    // ---------------- MODBUS CONFIG ----------------
    #define MB_UART_PORT UART_NUM_2
    #define MB_TX GPIO_NUM_17
    #define MB_RX GPIO_NUM_16
    #define MB_RTS GPIO_NUM_4

    #define BAUD_RATE 9600
    // Global for the structure of the send config
    typedef struct {
        char url[128];
        int qos;
        int retain;

        char topic[128];
        char payload_template[512];
        int delay_ms;
    } send_config_t;

    send_config_t send_cfg;

    typedef struct {
        int reg_index;
        float offset;
        float scale;
    } formula_t;
    // GLOBAL
    void* master_handler = NULL;
    typedef struct {
        int slave_id;
        int baud_rate;
        int parity;
        int stop_bits;
        int data_bits;
        int function_code;
        bool is_valid;
    } device_config_t;

    device_config_t device_cfg = {
        .slave_id = 1,
        .baud_rate = 9600,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .data_bits = UART_DATA_8_BITS,
        .function_code = 3,
        .is_valid = false
    };
//backup variables
    static formula_config_t backup_formulas[MAX_VARS];
   // static device_config_t backup_device;
    static send_config_t backup_send;

    int parse_parity(char *parity)
    {
        if (strcmp(parity, "none") == 0) return UART_PARITY_DISABLE;
        if (strcmp(parity, "even") == 0) return UART_PARITY_EVEN;
        if (strcmp(parity, "odd") == 0) return UART_PARITY_ODD;
        return -1;
    }

    // ---------------- MODBUS INIT ----------------
    void modbus_init(void)
    {
        mb_communication_info_t comm = {
            .ser_opts.port = MB_UART_PORT,
            .ser_opts.mode = MB_RTU,
            .ser_opts.baudrate = BAUD_RATE,
            .ser_opts.parity = UART_PARITY_DISABLE,
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


    // Replace register[x] with actual values
    void replace_registers(char *expr, uint16_t *data, int reg_count)
    {
        for (int i = 0; i < reg_count; i++)
        {
            char key[32]; 
            char val[32];
            snprintf(key, sizeof(key), "register[%d]", i);
            snprintf(val, sizeof(val), "%d", data[i]);

            char *pos;
            while ((pos = strstr(expr, key)) != NULL)
            {
                char temp[256];
                strcpy(temp, pos + strlen(key));

                *pos = '\0';
                strcat(expr, val);
                strcat(expr, temp);
            }
        }
    }

    // Check bitwise presence
    bool is_bitwise(char *expr)
    {
        return strstr(expr, "<<") || strstr(expr, ">>") ||
            strstr(expr, "|")  || strstr(expr, "&");
    }

    // Simple bitwise evaluator
    int eval_bitwise(char *expr)
    {
        int a, b;

        if (sscanf(expr, "(%d << %d)", &a, &b) == 2)
            return a << b;

        if (sscanf(expr, "(%d >> %d)", &a, &b) == 2)
            return a >> b;

        if (sscanf(expr, "(%d | %d)", &a, &b) == 2)
            return a | b;

        if (sscanf(expr, "(%d & %d)", &a, &b) == 2)
            return a & b;

        return 0;
    }
    // FLOAT DECODER
    float decode_float(uint16_t high, uint16_t low)
    {
        uint32_t combined = ((uint32_t)high << 16) | low;

        float result;
        memcpy(&result, &combined, sizeof(result));

        return result;
    }

    void replace_str(char *str, const char *old_str, const char *new_str)
    {
        char buffer[1024];
        char *pos;
        if (old_str[0] == '\0') return;

        while ((pos = strstr(str, old_str)) != NULL)
        {
            buffer[0] = '\0';
            strncat(buffer, str, pos - str);
            strcat(buffer, new_str);
            strcat(buffer, pos + strlen(old_str));
            strcpy(str, buffer);
        }
    }

    void disable_auto_mode(void) {
        nvs_handle_t nvs;
        if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_key(nvs, "config");
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        config_available = false;
        auto_mode = false;
        stored_json[0] = '\0';
        ESP_LOGW(TAG, "Auto mode disabled, config erased.");
    }
// ---------------- JSON COMMAND PROCESSOR ----------------
void process_command(char *cmd)
{
    ESP_LOGI(TAG, "Processing new JSON command...");

    // Store raw JSON
    strncpy(stored_json, cmd, sizeof(stored_json) - 1);
    stored_json[sizeof(stored_json) - 1] = '\0';

    config_available = true;
    // ================= CUSTOM COMMAND HANDLER =================
if (strncmp(cmd, "\"Command\"", 9) == 0)
{
    ESP_LOGI(TAG, "Command detected: %s", cmd);

    // Find JSON part after "Command"
    char *json_part = strchr(cmd, '{');

    if (json_part)
    {
        cJSON *cmd_json = cJSON_Parse(json_part);

        if (cmd_json)
        {
            cJSON *device = cJSON_GetObjectItem(cmd_json, "Device");

            if (cJSON_IsString(device))
            {
                // ================= GET ALL =================
                if (strcmp(device->valuestring, "Get_All") == 0)
                {
                    ESP_LOGI(TAG, "GET_ALL command received");

                    if (config_available)
                    {
                        uart_printf("%s\n", stored_json);
                    }
                    else
                    {
                        uart_printf("%s\n", default_config);
                    }

                    cJSON_Delete(cmd_json);
                    return;
                }

                // ================= FACTORY RESET (optional) =================
                if (strcmp(device->valuestring, "Factory_Reset") == 0)
                {
                    ESP_LOGW(TAG, "Factory Reset triggered");

                    nvs_handle_t nvs;
                    if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
                    {
                        nvs_erase_key(nvs, "config");
                        nvs_commit(nvs);
                        nvs_close(nvs);
                    }

                    config_available = false;
                    auto_mode = true;

                    strncpy(stored_json, default_config, sizeof(stored_json) - 1);

                    uart_printf("{\"status\":\"factory_reset_done\"}\n");

                    cJSON_Delete(cmd_json);
                    return;
                }
            }

            cJSON_Delete(cmd_json);
        }
    }

    uart_printf("{\"status\":\"invalid_command\"}\n");
    return;
}

    // Parse JSON
    cJSON *root_json = cJSON_Parse(cmd);
    if (!root_json) {
        ESP_LOGE(TAG, "Invalid JSON");
        uart_printf("{\"status\":\"json_error\"}\n");
        return;
    }

    cJSON *root = cJSON_IsArray(root_json) ? cJSON_GetArrayItem(root_json, 0) : root_json;

    // ================= MODE =================
    cJSON *mode = cJSON_GetObjectItem(root, "mode");

    if (cJSON_IsObject(mode))
    {
        cJSON *auto_obj = cJSON_GetObjectItem(mode, "auto");
        cJSON *request_obj = cJSON_GetObjectItem(mode, "request");

        // -------- AUTO MODE --------
        if (cJSON_IsBool(auto_obj))
        {
            auto_mode = cJSON_IsTrue(auto_obj);
            ESP_LOGI("MODE", "Auto mode: %s", auto_mode ? "ENABLED" : "DISABLED");
            nvs_handle_t nvs;
            if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
            {
                nvs_set_str(nvs, "config", cmd);
                nvs_commit(nvs);
                nvs_close(nvs);

                ESP_LOGI(TAG, "User config saved to NVS");
            }
        }

        // -------- REQUEST TRIGGER --------
        if (cJSON_IsBool(request_obj) && cJSON_IsTrue(request_obj))
        {
            ESP_LOGI("MODE", "REQUEST trigger received");

            if (!is_executing)
            {
                // -------- BACKUP AUTO CONFIG --------
                device_config_t backup_device = device_cfg;

                int backup_reg_addr = reg_addr;
                int backup_reg_count = reg_count;
                memcpy(backup_formulas, formulas, sizeof(formulas));
               // formula_config_t backup_formulas[MAX_VARS];
                memcpy(backup_formulas, formulas, sizeof(formulas));

                int backup_formula_count = formula_count;

                //send_config_t backup_send;
                memcpy(&backup_send, &send_cfg, sizeof(send_cfg));

                // -------- EXECUTE REQUEST --------
                ESP_LOGI("MODE", "Executing REQUEST (interrupt)");

                execute_cycle();

                // -------- RESTORE AUTO CONFIG --------
                device_cfg = backup_device;
                reg_addr = backup_reg_addr;
                reg_count = backup_reg_count;
                memcpy(formulas, backup_formulas, sizeof(formulas));
                formula_count = backup_formula_count;
                memcpy(&send_cfg, &backup_send, sizeof(send_cfg));

                ESP_LOGI("MODE", "Auto config restored");
            }
            else
            {
                ESP_LOGW("MODE", "System busy, request skipped");
            }
        }
    }
    else if (cJSON_IsString(mode))
    {
        if (strcmp(mode->valuestring, "clear") == 0)
        {
            ESP_LOGW("MODE", "CLEAR mode");

            nvs_handle_t nvs;
            if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK)
            {
                nvs_erase_key(nvs, "config");
                nvs_commit(nvs);
                nvs_close(nvs);
            }

            config_available = false;
            config_initialized = false;
            auto_mode = false;

            cJSON_Delete(root_json);
            return;
        }
    }

    // ================= SETTINGS =================
    cJSON *settings = cJSON_GetObjectItem(root, "settings");

    if (cJSON_IsObject(settings))
    {
        GET_INT(settings, "slave_id", device_cfg.slave_id);
        GET_INT(settings, "baud_rate", device_cfg.baud_rate);
        GET_INT(settings, "function_code", device_cfg.function_code);

        device_cfg.is_valid = true;

        ESP_LOGI(TAG, "Device Config → ID:%d Baud:%d Func:%d",
                 device_cfg.slave_id,
                 device_cfg.baud_rate,
                 device_cfg.function_code);
    }
    // // ================= SETTINGS =================
    // cJSON *settings = cJSON_GetObjectItem(root, "settings");

    // if (settings)
    // {
    //     // device_cfg.slave_id =
    //     //     atoi(cJSON_GetObjectItem(settings, "slave_id")->valuestring);
    //     cJSON *slave = cJSON_GetObjectItem(settings, "slave_id");
    //     if (slave && cJSON_IsNumber(slave))
    //     {
    //         device_cfg.slave_id = slave->valueint;
    //     }

    //     // device_cfg.baud_rate =
    //     //     atoi(cJSON_GetObjectItem(settings, "baud_rate")->valuestring);
    //     cJSON *baud = cJSON_GetObjectItem(settings, "baud_rate");
    //     if (baud && cJSON_IsNumber(baud))
    //     {
    //         device_cfg.baud_rate = baud->valueint;
    //     }

    //     // device_cfg.function_code =
    //     //     atoi(cJSON_GetObjectItem(settings, "function_code")->valuestring);
    //     cJSON *func = cJSON_GetObjectItem(settings, "function_code");
    //     if (func && cJSON_IsNumber(func))
    //     {
    //         device_cfg.function_code = func->valueint;
    //     }

    //     device_cfg.is_valid = true;
    // }

    // ================= READ =================
    cJSON *read = cJSON_GetObjectItem(root, "read");

    if (cJSON_IsObject(read))
    {
        GET_INT(read, "register_address", reg_addr);
        GET_INT(read, "register_count", reg_count);

        ESP_LOGI(TAG, "Read Config → addr: %d, count: %d", reg_addr, reg_count);
    }

    // ================= READ =================
    // cJSON *read = cJSON_GetObjectItem(root, "read");

    // if (read)
    // {
    //     reg_addr =
    //         atoi(cJSON_GetObjectItem(read, "register_address")->valuestring);

    //     reg_count =
    //         atoi(cJSON_GetObjectItem(read, "register_count")->valuestring);
    // }

    // ================= MQTT CONFIG =================
    cJSON *send = cJSON_GetObjectItem(root, "send");

    if (send)
    {
        strcpy(send_cfg.url,
               cJSON_GetObjectItem(send, "url")->valuestring);

        strcpy(send_cfg.topic,
               cJSON_GetObjectItem(send, "topic")->valuestring);

        send_cfg.qos =
            cJSON_GetObjectItem(send, "qos")->valueint;

        send_cfg.retain =
            cJSON_GetObjectItem(send, "retain")->valueint;

        // Store payload template (for later use)
        char *payload = cJSON_PrintUnformatted(
            cJSON_GetObjectItem(send, "payload"));

        if (payload)
        {
            strncpy(send_cfg.payload_template, payload,
                    sizeof(send_cfg.payload_template) - 1);
            free(payload);
        }

        ESP_LOGI(TAG, "URL: %s", send_cfg.url);
        ESP_LOGI(TAG, "QoS: %d", send_cfg.qos);
        ESP_LOGI(TAG, "Retain: %d", send_cfg.retain);
        ESP_LOGI(TAG, "Topic: %s", send_cfg.topic);
        ESP_LOGI(TAG, "Payload: %s", send_cfg.payload_template);
    }
    // ================= FORMULA PARSE =================
cJSON *formula = cJSON_GetObjectItem(root, "formula");
cJSON *variables = cJSON_GetObjectItem(root, "variables");

formula_count = 0;  // reset every new command

if (cJSON_IsObject(formula) && cJSON_IsObject(variables))
{
    cJSON *item = NULL;

    cJSON_ArrayForEach(item, formula)
    {
        if (formula_count >= MAX_VARS)
            break;

        const char *name = item->string;
        const char *expr = item->valuestring;

        cJSON *type_obj = cJSON_GetObjectItem(variables, name);

        if (!cJSON_IsString(item) || !cJSON_IsString(type_obj))
            continue;

        strncpy(formulas[formula_count].name, name, sizeof(formulas[formula_count].name));
        strncpy(formulas[formula_count].expr, expr, sizeof(formulas[formula_count].expr));
        strncpy(formulas[formula_count].type, type_obj->valuestring, sizeof(formulas[formula_count].type));

        formulas[formula_count].last_value = 0;

        ESP_LOGI("FORMULA", "Loaded %s = %s (%s)",
                 name, expr, type_obj->valuestring);

        formula_count++;
    }

    ESP_LOGI("FORMULA", "Total formulas loaded: %d", formula_count);
}
else
{
    ESP_LOGW("FORMULA", "No formula/variables found in JSON");
}
    // ================= DELAY =================
    cJSON *delay = cJSON_GetObjectItem(root, "delay");

    if (cJSON_IsNumber(delay))
    {
        send_cfg.delay_ms = delay->valueint * 1000;
    }
    else
    {
        send_cfg.delay_ms = 2000; // default
    }
    // ================= FINAL FLAG =================
    config_initialized = true;

    cJSON_Delete(root_json);
}
    

bool config_parsed = false;
void init_sntp(void)
{
    ESP_LOGI("SNTP", "Initializing SNTP...");

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}
        void get_timestamp(char *buf, size_t len)
        {
            time_t now;
            struct tm timeinfo;

            time(&now);
            localtime_r(&now, &timeinfo);

            strftime(buf, len, "%Y-%m-%d %H:%M:%S", &timeinfo);
        }
void execute_cycle()
{
    ESP_LOGI(TAG, "Executing scheduled Modbus poll...");

    if (!device_cfg.is_valid) {
        ESP_LOGW(TAG, "Device config not valid");
        return;
    }

    // ================= MODBUS READ =================
    uint16_t data_buf[125] = {0};

    mb_param_request_t req = {
        .slave_addr = device_cfg.slave_id,
        .command    = device_cfg.function_code,
        .reg_start  = reg_addr,
        .reg_size   = reg_count
    };

    esp_err_t err = mbc_master_send_request(master_handler, &req, data_buf);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Modbus Error: 0x%x", err);
        return;
    }

    // ================= FORMULA ENGINE =================
    char output[512];
    int offset = 0;
    bool first = true;

    offset += snprintf(output + offset, sizeof(output) - offset,
                       "{\"status\":\"ok\",\"decoded\":{");

    for (int i = 0; i < formula_count; i++)
    {
        char expr_buf[128];
        strcpy(expr_buf, formulas[i].expr);

        // Replace register[x] → actual values
        replace_registers(expr_buf, data_buf, reg_count);

        

        double result;

        // Bitwise or math
        if (is_bitwise(expr_buf))
            result = eval_bitwise(expr_buf);
        else
            result = te_interp(expr_buf, 0);

        formulas[i].last_value = result;

        if (!first)
            offset += snprintf(output + offset, sizeof(output) - offset, ",");

        first = false;

        if (strcmp(formulas[i].type, "int") == 0)
        {
            offset += snprintf(output + offset, sizeof(output) - offset,
                               "\"%s\":%d",
                               formulas[i].name,
                               (int)result);
        }
        else
        {
            offset += snprintf(output + offset, sizeof(output) - offset,
                               "\"%s\":%.2f",
                               formulas[i].name,
                               result);
        }
    }

    offset += snprintf(output + offset, sizeof(output) - offset, "}}");

    // Debug output (optional)
    uart_printf("%s\n", output);

    // ================= PAYLOAD BUILD =================
    char payload_buf[256];
    strncpy(payload_buf, send_cfg.payload_template, sizeof(payload_buf) - 1);

    // Replace variables
    for (int i = 0; i < formula_count; i++)
    {
        char key[64], value[64];

        snprintf(key, sizeof(key), "{{%s}}", formulas[i].name);

        if (strcmp(formulas[i].type, "int") == 0)
            snprintf(value, sizeof(value), "%d", (int)formulas[i].last_value);
        else
            snprintf(value, sizeof(value), "%.2f", formulas[i].last_value);

        replace_str(payload_buf, key, value);
    }

    // Timestamp
    char time_str[32];
    get_timestamp(time_str, sizeof(time_str));
    replace_str(payload_buf, "{{timestamp}}", time_str);

    // ================= MQTT PUBLISH =================
    if (mqtt_is_connected())
    {
        ESP_LOGI(TAG, "Publishing to MQTT...");
        mqtt_publish(send_cfg.topic, payload_buf, send_cfg.qos, send_cfg.retain);
    }
    else
    {
        ESP_LOGW(TAG, "MQTT not connected, skipping publish");
    }
}
    void modbus_loop_task(void *arg)
{
    static bool first_run = true;
    // static bool mqtt_started = false;
    static char last_url[128] = "";

    while(1)
    {
        //  Apply stored config safely (after system is ready)
        if (config_available && auto_mode && first_run)
        {
            ESP_LOGI(TAG, "Applying stored config...");

            process_command(stored_json);   //SAFE HERE

            first_run = false;
        }
            //MQTT SAFE START
            if (config_available)
            {
                if (strlen(send_cfg.url) > 0 &&
                    strcmp(last_url, send_cfg.url) != 0)
                {
                    ESP_LOGI("MQTT", "Reconnecting with new URL...");
                    mqtt_reconnect(send_cfg.url);

                    strcpy(last_url, send_cfg.url);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
   
        if(config_available && auto_mode && config_initialized && !system_error)
        {
            if(!is_executing)
            {
                execute_cycle();
            }
        }

        int delay_ms = (send_cfg.delay_ms > 0) ? send_cfg.delay_ms : 2000;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

    
                
    // ---------------- MAIN ----------------
    void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");
    // ================= LOAD CONFIG =================
    nvs_handle_t nvs;
    size_t size = sizeof(stored_json);

    bool nvs_found = false;

    if (nvs_open("storage", NVS_READONLY, &nvs) == ESP_OK)
    {
        if (nvs_get_str(nvs, "config", stored_json, &size) == ESP_OK)
        {
            ESP_LOGI(TAG, "Loaded USER config from NVS");

            config_available = true;
            auto_mode = true;
            nvs_found = true;
        }
        nvs_close(nvs);
    }

    // ================= FALLBACK TO DEFAULT =================
    if (!nvs_found)
    {
        ESP_LOGW(TAG, "No user config found → using DEFAULT config");

        strncpy(stored_json, default_config, sizeof(stored_json) - 1);
        stored_json[sizeof(stored_json) - 1] = '\0';

        config_available = true;
        auto_mode = true;
    }
    // // ================= NVS INIT =================
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ESP_ERROR_CHECK(nvs_flash_init());
    // }

    // // ================= LOAD STORED CONFIG =================
    // nvs_handle_t nvs;
    // size_t size = sizeof(stored_json);

    // if (nvs_open("storage", NVS_READONLY, &nvs) == ESP_OK)
    // {
    //     if (nvs_get_str(nvs, "config", stored_json, &size) == ESP_OK)
    //     {
    //         ESP_LOGI(TAG, "Loaded AUTO config from NVS");

    //         config_available = true;
    //         auto_mode = true;

            
    //     }
    //     nvs_close(nvs);
    // }

    // ================= WIFI =================
    wifi_init();   // Blocks until connected

    // ================= SNTP (TIME) =================
  
    init_sntp();

    // Give time to sync (simple method)
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ================= MQTT =================
    mqtt_init();

    // ================= MODBUS =================
    modbus_init();

    // ================= UART =================
    uart_comm_init();
    uart_comm_set_callback(process_command);

    // ================= MAIN LOOP TASK =================
    xTaskCreate(modbus_loop_task,
                "modbus_task",
                8192,
                NULL,
                5,
                NULL);

    ESP_LOGI(TAG, "System initialization complete");
}