    #include "mqtt_client.h"
    #include "esp_log.h"
    #include "esp_event.h"
    #include "esp_netif.h"
    #include <string.h>

    static const char *TAG = "MQTT";

    extern void process_command(char *cmd);

    static esp_mqtt_client_handle_t client = NULL;
    static bool is_connected = false;
    static char current_uri[128] = "";

    bool mqtt_is_connected(void) {
        return is_connected;
    }

    // ---------------- EVENT HANDLER ----------------
    static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
    {
        switch ((esp_mqtt_event_id_t)event_id)
        {
            case MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected");
                is_connected = true;
                esp_mqtt_client_subscribe(client, "modbus/gateway/control", 0);
                break;
            
            case MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected");
                is_connected = false;
                break;

            case MQTT_EVENT_DATA:
            {
                esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

                char msg[512];

                if (event->data_len < sizeof(msg))
                {
                    memcpy(msg, event->data, event->data_len);
                    msg[event->data_len] = '\0';

                    ESP_LOGI(TAG, "MQTT RX: %s", msg);

                    process_command(msg);   // Forward to JSON parser
                }
                break;
            }

            default:
                break;
        }
    }

    // ---------------- INIT (DEFAULT) ----------------
    void mqtt_init(void)
    {
        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = "mqtt://test.mosquitto.org:1883"
        };

        client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(client);
    }

    // ---------------- RECONNECT (DYNAMIC URL) ----------------
    void mqtt_reconnect(const char *url)
    {
        if (url == NULL || strlen(url) == 0)
        {
            ESP_LOGW(TAG, "Empty URL, skipping reconnect");
            return;
        }

        // Only reconnect if the URL actually changed
        if (client != NULL && strcmp(current_uri, url) == 0) {
            ESP_LOGD(TAG, "URL same as current, skipping reconnect");
            return; 
        }

        ESP_LOGI(TAG, "Reconnecting MQTT to: %s", url);
        strncpy(current_uri, url, sizeof(current_uri) - 1);
        current_uri[sizeof(current_uri) - 1] = '\0';

        if (client != NULL)
        {
            is_connected = false;
            esp_mqtt_client_stop(client);
            esp_mqtt_client_destroy(client);
            client = NULL;
        }

        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = url
        };

        client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(client);
    }

    // ---------------- PUBLISH ----------------
    void mqtt_publish(const char *topic, const char *data, int qos, int retain)
    {
        if (client == NULL)
        {
            ESP_LOGE(TAG, "MQTT client not initialized");
            return;
        }

        if (topic == NULL || data == NULL)
        {
            ESP_LOGE(TAG, "Invalid topic or payload");
            return;
        }

        ESP_LOGI(TAG, "Publishing -> Topic: %s", topic);
        ESP_LOGI(TAG, "Payload -> %s", data);

        esp_mqtt_client_publish(client, topic, data, 0, qos, retain);
    }