// Copie este arquivo para secrets.h e preencha. secrets.h não vai pro git.

#define WIFI_SSID        "minha-rede"
#define WIFI_PASS        "minha-senha"

#define MQTT_HOST        "192.168.1.10"   // mesmo broker do gateway
#define MQTT_PORT        1883
#define MQTT_USER        ""               // vazio = sem autenticação
#define MQTT_PASS        ""
#define MQTT_CLIENT_ID   "lora-display"   // diferente do gateway!
#define MQTT_TOPIC_BASE  "lora"           // o mesmo do gateway
