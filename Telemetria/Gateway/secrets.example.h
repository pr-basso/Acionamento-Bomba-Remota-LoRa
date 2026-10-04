// Copie este arquivo para secrets.h e preencha. secrets.h não vai pro git.

// WiFi da placa (só com USE_WIFI definido no Gateway.ino)
#define WIFI_SSID        "minha-rede"
#define WIFI_PASS        "minha-senha"

#define MQTT_HOST        "192.168.1.10"   // broker Mosquitto
#define MQTT_PORT        1883
#define MQTT_USER        ""               // vazio = sem autenticação
#define MQTT_PASS        ""
#define MQTT_CLIENT_ID   "lora-gateway"
#define MQTT_TOPIC_BASE  "lora"           // tópicos: lora/<SRC>/<chave>

// IP fixo (opcional). Comente USE_STATIC_IP para usar DHCP.
// #define USE_STATIC_IP
#define STATIC_IP        192, 168, 1, 50
#define STATIC_GATEWAY   192, 168, 1, 1
#define STATIC_SUBNET    255, 255, 255, 0
#define STATIC_DNS       192, 168, 1, 1
