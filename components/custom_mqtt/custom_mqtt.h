#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize MQTT
void mqtt_init(void);

// Publish data
void mqtt_publish(const char *topic, const char *data, int qos, int retain);

// Reconnect MQTT
void mqtt_reconnect(const char *url);

// Optional
void mqtt_subscribe(const char *topic);
bool mqtt_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H