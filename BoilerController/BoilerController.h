//WiFi settings
#define wifiSSID               "Engrie"
#define wifiPassword           "1357924680"
#define location               "Home"

// NTP settings
#define ntpServer              "be.pool.ntp.org"
#define ntpUTCOffset_sec       3600                 
#define ntpDSTOffset_sec       3600
#define ntpUpdate_sec          60

// OTA settings
// Port default is 3232 
#define otaPort                3232
#define otaPassword            ""
// Password can be set with it's md5 value as well 
// Eg:  MD5 of "admin" = 21232f297a57a5a743894a0e4a801fc3
#define otaPasswordHash        ""

// MQTT settings 
#define mqttServer             "mqtt.home"
#define mqttPort               1883
#define mqttLogin              ""
#define mqttPassword           ""
#define mqttTopicBase          "engrie111/bc"
// send MQTT message every x seconds
#define mqttInterval           60
