//--------------------------------------------------------------------------
// Versioning
//--------------------------------------------------------------------------
#define VERSION    "1.20"

//--------------------------------------------------------------------------
// Debugging flag
//--------------------------------------------------------------------------
//#define DEBUG

//----------------------------------------------------------------------------
// includes
//----------------------------------------------------------------------------
#include <Ticker.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// application settings
#include "BoilerController.h"

//----------------------------------------------------------------------------
// defines
//----------------------------------------------------------------------------

// GPIO for controlling relay
#define RELAY_GPIO         33
#define RELAYOFF           false
#define RELAYON            true

//DAC output port
#define DAC_GPIO           26

// OneWire data line connected to GPIO
#define ONE_WIRE_BUS        4

// ADC input channel for ZMPT101B
#define ZMPT101B_GPIO      35

// ADC input channel for ACS712
#define ACS712_GPIO        34

#define ADCvolt            (float)0.210     // volt per ADC count
#define ADCamps            (float)0.0195    // amp per ADC count

#define CLOCKSPEED         80               // clockspeed of ESP32 in MHz
#define nomFREQ            50.00            // nominal frequency

// max MQTT topic size
#define MQTT_TOPIC_SIZE    150

// easy wrapper for counting seconds instead of millisceonds
#define seconds()          (millis()/1000)
// easy wrapper for executing something every t seconds
#define every(t)           for (static unsigned long _lasttime; (unsigned long)((unsigned long)millis() - _lasttime) >= (t); _lasttime = millis())
// easy wrapper for getting number of element in array
#define ARRAY_SIZE(array)  ((sizeof(array))/(sizeof(array[0])))

//----------------------------------------------------------------------------
// constants
//----------------------------------------------------------------------------

// number of samples to take before RMS is calculate
const int iADC = 2 * nomFREQ;    // one sample every 1 ms -> 100ms = 5 periods @ 50Hz
// number of Vrms to hold before average Vrms is outputted
const int iRMS =  50;            // 100 x 5 periods -> 100 * 100 ms -> 5 seconds

//----------------------------------------------------------------------------
// global variables
//----------------------------------------------------------------------------
// to hold MAC address
String wifiMACAddress = WiFi.macAddress();
char   caMAC[20];
char   hostname[64];

bool useLED = true;

// time related
unsigned long time_now;
char today[11];
// to hold date & time
char  nuDate[12];
char  nuTime[10];
char  currDST[3];
char  lastDST[3];
char *dt;
int   days[] = {31,29,31,30,31,30,31,31,30,31,30,31};

char  mqttClientID[30];
char  mqttTopicCmd[MQTT_TOPIC_SIZE];
char  mqttTopicSts[MQTT_TOPIC_SIZE];
char  mqttTopicDat1[MQTT_TOPIC_SIZE];
char  mqttTopicDat2[MQTT_TOPIC_SIZE];
char  payload[MQTT_MAX_PACKET_SIZE - MQTT_TOPIC_SIZE];

// a temporary buffer to store characters mostly to print/mail stuff
char caTemp[256];
char caJson[256];

// DS18B20 addresses
DeviceAddress ds18b20_1 = { 0x28, 0x2F, 0x9C, 0x56, 0xB5, 0x01, 0x3C, 0x9B };
DeviceAddress ds18b20_2 = { 0x28, 0xFF, 0x64, 0x02, 0xC8, 0x7D, 0x29, 0xB7 };
DeviceAddress ds18b20_3 = { 0x28, 0xFF, 0x64, 0x1E, 0x0C, 0x04, 0xC5, 0x1E };

float temp_1;
float temp_2;
float temp_3;

// DAC
unsigned int dacn = 0;
unsigned int upct = 0;

// Relay
unsigned int relay_state;

// counters for array index
int  cntADC = 0;
int  cntRMS = 0;

// vars related to ADC and Currents
float ADCval;

// voltage
float ADCzeroV   = 1700;

unsigned long valZERv[iADC];
unsigned long avgZERv[iRMS];
float         valADCv[iADC];
float         valRMSv[iRMS];

float         rmsValV = 0;

// amperes
float ADCzeroA   = 1900;

unsigned long valZERa[iADC];
unsigned long avgZERa[iRMS];
float         valADCa[iADC];
float         valRMSa[iRMS];

float         rmsValA = 0;

// needed for timer interrupt
volatile byte cntrINT;
hw_timer_t * timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//----------------------------------------------------------------------------
// objects
//----------------------------------------------------------------------------
// Initiate led blinker library
Ticker ticker;

// object for NTP
WiFiUDP   ntpUDP;
NTPClient ntpClient(ntpUDP, ntpServer, ntpUTCOffset_sec, ntpUpdate_sec * 1000);

// MQTT / PubSub needings
WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);

// 1-Wire object
OneWire           oneWire(ONE_WIRE_BUS);

// DS18B20 object
DallasTemperature sensors(&oneWire);


//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------

//............................................................................
//** Timer interrupt
//****************************************************************************
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  cntrINT++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//****************************************************************************

//............................................................................
// Call-back functions
//............................................................................
//** Call-back TICKER
//****************************************************************************
void tick()
{
  // get the current state of GPIO1 pin
  int state = digitalRead(LED_BUILTIN);
  // set pin to the opposite state
  digitalWrite(LED_BUILTIN, !state);
}
//****************************************************************************

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ WIFI Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** Set-up WIFI
//**************************************************************************
void wifiConnect()
{

  // reset WiFi configuration
  WiFi.disconnect();
  // Next line was needed before Core 1.0.5 to set the hostname correctly
  // since Core 1.0.5 rc6 no longer. see https://github.com/espressif/arduino-esp32/issues/4732
  // leaving the line will result in getting 255.255.255.255 as IP-address.
  // WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // removing line above curres getting ip 255.255.255.255 but
  // brings back the issue of hostname not being set in DHCP -  DAMNED!!!!
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(wifiSSID, wifiPassword);

  #if defined(DEBUG)
    Serial.print("WiFi attempting connection to "); Serial.println(wifiSSID);
  #endif

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    #if defined(DEBUGWIFI)
      Serial.print('.');
    #endif

    // delay non-blocking
    unsigned long currTime = millis();
    while(millis() < currTime + 500){}
    if((++i % 16) == 0)
    {
      #if defined(DEBUGWIFI)
        Serial.println("");
        Serial.println("still trying to connect");
      #endif
    }

    if(i > 32)
    {
      #if defined(DEBUGWIFI)
        Serial.println("Restarting... (WIFI)");
      #endif
      ESP.restart();
    }
  }
}
//**************************************************************************

//** WiFi call-back
//**************************************************************************
void wifiEvent(WiFiEvent_t event)
{
  Serial.printf("  [WiFi-event : %d] = ", event);

  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:
      Serial.println("Interface ready");
      break;

    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for APs");
      break;

    case SYSTEM_EVENT_STA_START:
      Serial.println("STA Client started");
      break;

    case SYSTEM_EVENT_STA_STOP:
      Serial.println("STA Client stopped");
      break;

    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to AP");
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from AP");
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("Auth changed");
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("Obtained IP: ");
      Serial.println(WiFi.localIP());
      break;

    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("Lost IP");
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WPS succeeded");
      break;

    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WPS failed");
      break;

    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WPS timeout");
      break;

    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WPS pin code");
      break;

    case SYSTEM_EVENT_AP_START:
      Serial.println("AP started");
      break;

    case SYSTEM_EVENT_AP_STOP:
      Serial.println("AP stopped");
      break;

    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("AP Client connected");
      break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("AP Client disconnected");
      break;

    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println("Assigned IP to client");
      break;

    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;

    case SYSTEM_EVENT_GOT_IP6:
      Serial.println("IPv6 preferred");
      break;

    case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;

    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;

    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;

    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP");
      break;

    default:
        break;
  }
}
//**************************************************************************

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ mDNS Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** Set-up mDNS
//**************************************************************************
void mdnsSetup()
{
  #if defined(DEBUG)
    Serial.println("mDNS activating");
  #endif

  if(!MDNS.begin(hostname))
  {
    #if defined(DEBUG)
      Serial.println("mDNS activation FAILED");
    #endif
    return;
  }

  // indicate there is a webserver on this device
  MDNS.addService("http", "tcp", 80);

  #if defined(DEBUG)
    Serial.println("mDNS activated");
  #endif
}
//**************************************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ NTP/Time Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** Set-up NTP
//**************************************************************************
void ntpSetup()
{
  #if defined(DEBUG)
    Serial.println("NTP setup");
  #endif

  // clear some vars
  memset(lastDST, NULL, 3);
  memset(currDST, NULL, 3);

  // Init NTP
  ntpClient.begin();

  String nu = ntpClient.getFormattedDate();
  #if defined(DEBUG)
    Serial.print("Init    ->  Date - Time: "); Serial.print(nu); Serial.println();
  #endif

  ntpClient.forceUpdate();

  nu = ntpClient.getFormattedDate();
  #if defined(DEBUG)
    Serial.print("Updated ->  Date - Time: "); Serial.print(nu); Serial.println();
  #endif

  //adjust DST if needed
  checkDST();

  nu = ntpClient.getFormattedDate();
  #if defined(DEBUG)
    Serial.print("Final   ->  Date - Time: "); Serial.print(nu); Serial.println();
  #endif

  #if defined(DEBUG)
    Serial.println("NTP activated");
  #endif
}
//**************************************************************************

//** Check for DST
//**************************************************************************
void checkDST()
{
  #if defined(DEBUG)
    Serial.println("DST Checking ....");
  #endif

  // due to problem/bug in time /timezone, the following code
  // is required to make sure we have the right offset for DST

  // make sure we have recently updated
  ntpClient.forceUpdate();

  // get date
  String nu = ntpClient.getFormattedDate().substring(0,19);
  #if defined(DEBUG)
    Serial.println(nu);
  #endif

  int y, m, w;
  // correct for leapyear
  y = nu.substring(0,4).toInt();
  days[1] -= (y % 4) || (!(y % 100) && (y % 400));
  w = y * 365 + 97 * (y - 1) / 400 + 4;

  char buff[20];
  // find last sunday of March
  m = 2; // March
  w = (w + days[m]) % 7;
  sprintf(buff, "%04d-%02d-%02dT02:00:00", y, m + 1,days[m] - w);
  String DSTstart(buff);
  #if defined(DEBUG)
    Serial.println(DSTstart);
  #endif

  // find last sunday of Octobre
  m = 9; // Octobre
  w = (w + days[m]) % 7;
  sprintf(buff, "%04d-%02d-%02dT03:00:00", y, m + 1,days[m] - w);
  String DSTend(buff);
  #if defined(DEBUG)
    Serial.println(DSTend);
  #endif

  // if nu is between March 02:00:00 and Oct 03:00:00
  // DST is active (at least till 2024)
  if(nu >= DSTstart && nu < DSTend)
  {
    // DST = UTC + ntpUTCOffset_sec + ntpDSTOffset_sec
    ntpClient.setTimeOffset(ntpUTCOffset_sec + ntpDSTOffset_sec);
    #if defined(DEBUG)
      Serial.println("DST set");
    #endif
  }
  else
  {
    // non-DST = UTC + ntpUTCOffset_sec
    ntpClient.setTimeOffset(ntpUTCOffset_sec);
    #if defined(DEBUG)
      Serial.println("DST reset");
    #endif
  }

  // make sure we update local date/time
  ntpClient.forceUpdate();

  // save last check of DST
  sprintf(lastDST, "%02d", ntpClient.getHours());
  sprintf(currDST, "%02d", ntpClient.getHours());
  ntpClient.getFormattedTime().toCharArray(nuTime, 8);
  nuTime[9] = NULL;

  #if defined(DEBUG)
    Serial.println("DST Checking done");
  #endif
}

//** Get Date & Time
//**************************************************************************
char* ntpLocalDateTime()
{
  static char caDT[21];

  String nu = ntpClient.getFormattedDate().substring(0,19);
  // convert to char array
  nu.toCharArray(caDT, 20);
  // remove T
  caDT[10] = ' ';
  //terminate to string
  caDT[20] = NULL;

  return(caDT);
}
//**************************************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ OTA Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** Set-up OTA
//**************************************************************************
void otaSetup()
{
  #if defined(DEBUG)
    Serial.println("OTA activating");
  #endif

  // Port defaults to 3232
  ArduinoOTA.setPort(otaPort);

  // Hostname defaults to esp32-[MAC]
  ArduinoOTA.setHostname(hostname);

  // Set OTA Password. No authentication by default
  if(otaPasswordHash != "")
  {
    ArduinoOTA.setPasswordHash(otaPasswordHash);
  }
  else if(otaPassword != "")
  {
    ArduinoOTA.setPassword(otaPassword);
  }

  ArduinoOTA
    .onStart([]()
    {
      String type;
      if(ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      #if defined(DEBUG)
        Serial.println(" - Start updating " + type);
      #endif
    })

    .onProgress([](unsigned int progress, unsigned int total)
    {
      #if defined(DEBUG)
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      #endif
    })

    .onEnd([]()
    {
      #if defined(DEBUG)
        Serial.print("\n"); Serial.println(" - End");
      #endif
    })

    .onError([](ota_error_t error)
    {
      #if defined(DEBUG)
        Serial.printf(" - Error[%u]: ", error);
        if(error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
        else if(error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if(error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if(error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if(error == OTA_END_ERROR)     Serial.println("End Failed");
      #endif
    }

  );

  ArduinoOTA.begin();
  #if defined(DEBUG)
    Serial.println("OTA activated");
  #endif
}
//**************************************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ MQTT Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** Set-up MQTT
//****************************************************************************
void mqttConnect()
{

  #if defined(DEBUG)
    Serial.println(F("MQTT activating"));
    Serial.print(F("MQTT attempting connection to ")); Serial.print(mqttServer);  Serial.print(F(":")); Serial.println(mqttPort);
  #endif

  // setup and connect to MQTT server
  // mqttClientID
  wifiMACAddress.replace(":","");
  wifiMACAddress.toCharArray(caMAC, 20);
  strcpy(mqttClientID,"bc_");
  strcat(mqttClientID, caMAC);
  #if defined(DEBUG)
    Serial.print(F("  MQTT ClientID: <")); Serial.print(mqttClientID); Serial.println(F(">"));
  #endif

  // mqttTopicCmd
  strcpy(mqttTopicCmd, mqttTopicBase);
  strcat(mqttTopicCmd, "/bc_");
  strcat(mqttTopicCmd, caMAC);
  strcat(mqttTopicCmd, "/command");
  #if defined(DEBUG)
    Serial.print(F("  MQTT Topic CMD: <")); Serial.print(mqttTopicCmd); Serial.println(F(">"));
  #endif

  // mqttTopicSts
  strcpy(mqttTopicSts, mqttTopicBase);
  strcat(mqttTopicSts, "/bc_");
  strcat(mqttTopicSts, caMAC);
  strcat(mqttTopicSts, "/status");
  #if defined(DEBUG)
    Serial.print(F("  MQTT Topic STS: <")); Serial.print(mqttTopicSts); Serial.println(F(">"));
  #endif

  // mqttTopicDat
  strcpy(mqttTopicDat1, mqttTopicBase);
  strcat(mqttTopicDat1, "/bc_");
  strcat(mqttTopicDat1, caMAC);
  strcat(mqttTopicDat1, "/data/gen");
  #if defined(DEBUG)
    Serial.print(F("  MQTT Topic DAT1: <")); Serial.print(mqttTopicDat1); Serial.println(F(">"));
  #endif

  strcpy(mqttTopicDat2, mqttTopicBase);
  strcat(mqttTopicDat2, "/bc_");
  strcat(mqttTopicDat2, caMAC);
  strcat(mqttTopicDat2, "/data/dat");
  #if defined(DEBUG)
    Serial.print(F("  MQTT Topic DAT2: <")); Serial.print(mqttTopicDat2); Serial.println(F(">"));
  #endif

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttOnMessage);

  // ESP will connect to mqtt broker with clientID, last will, ...
  // bool connect(const char* id, const char* user, const char* pass, const char* willTopic, uint8_t willQos, bool willRetain, const char* willMessage, bool cleanSession);
  if (mqttClient.connect(mqttClientID, mqttLogin, mqttPassword, mqttTopicSts, 1, true, "offline", false))
  {

    #if defined(DEBUG)
      Serial.println(F("MQTT connected"));
    #endif

    // subscribe to command topic
    mqttClient.subscribe(mqttTopicCmd, 1);
    // let know we are online
    mqttClient.publish(mqttTopicSts, "online", true);
    #if defined(DEBUG)
      Serial.println(F("MQTT activated"));
    #endif
  }
  else
  {
    #if defined(DEBUG)
      Serial.println(F("MQTT activation Failed"));
    #endif
  }

}
//****************************************************************************

// ** reconnect MQTT
//****************************************************************************
void mqttReconnect()
{
  uint8_t i = 0;

  while (!mqttClient.connected())
  {
    #if defined(DEBUG)
      Serial.print(F("MQTT attempting reconnection to ")); Serial.print(mqttServer);  Serial.print(F(":")); Serial.println(mqttPort);
    #endif
    if (mqttClient.connect(mqttClientID, mqttLogin, mqttPassword, mqttTopicSts, 1, true, "offline", false))
    {
      #if defined(DEBUG)
        Serial.println(F("MQTT reconnected"));
      #endif
      // subscribe to command topic
      mqttClient.subscribe(mqttTopicCmd, 1);
      // let know we are online
      mqttClient.publish(mqttTopicSts, "online", true);
    }
    else
    {
      #if defined(DEBUG)
        Serial.print(F("MQTT reconnection failed, rc=")); Serial.print(mqttClient.state()); Serial.println(F(" ... try again in 5 seconds"));
      #endif
      // Wait 15 seconds before retrying
      // delay non-blocking
      time_now = millis();
      while(millis() < time_now + 1500){}
      ++i;
      if (i > 5)
      {
        #if defined(DEBUG)
          Serial.println(F("Restarting..."));
        #endif
        ESP.restart();
     }
    }
  }
}
//****************************************************************************

// ** send MQTT messages
//****************************************************************************
void mqttSend(char *topic, char* data)
{
  mqttClient.publish(topic, data, true);
}
//****************************************************************************

//** receive MQTT messages
//****************************************************************************
void mqttOnMessage(char* topic, byte* payload, unsigned int payloadlength)
{
  //callback includes topic and payload
  #if defined(DEBUG)
    Serial.print(F("Message arrived on topic [")); Serial.print(topic); Serial.print(F("] : "));
  #endif

  char caTemp[payloadlength + 1];
  strncpy(caTemp, (char*)payload, payloadlength);
  caTemp[payloadlength] = NULL;

  #if defined(DEBUG)
    Serial.print(F("  <")); Serial.print(caTemp); Serial.println(F(">"));
  #endif

  // do what is needed based on the payload
  if(strncasecmp(caTemp, "Out=", 4) == 0)
  {
    #if defined(DEBUG)
      Serial.print("MQTT Message Received: "); Serial.println(caTemp);
    #endif

    char *pct;
    // part before =
    pct = strtok(caTemp, "=");
    // part after =
    pct = strtok(NULL, "=");
    //#if defined(DEBUG)
    //   Serial.print("%: "); Serial.println(pct);
    //#endif
    if (strlen(pct) > 0)
    {
      setDAC(pct);
    }
  }
  else if(strcasecmp(caTemp, "Relay=ON") == 0)
  {
    #if defined(DEBUG)
      Serial.println("MQTT Message Received: Relay=ON");
    #endif
    setRelay(RELAYON);
  }
  else if(strcasecmp(caTemp, "Relay=OFF") == 0)
  {
    #if defined(DEBUG)
      Serial.println("MQTT Message Received: Relay=OFF");
    #endif
    setRelay(RELAYOFF);
  }
  else if(strcasecmp(caTemp, "ON") == 0)
  {
    #if defined(DEBUG)
      Serial.println("MQTT Message Received: ON");
    #endif
    setRelay(RELAYON);
  }
  else if(strcasecmp(caTemp, "OFF") == 0)
  {
    #if defined(DEBUG)
      Serial.println("MQTT Message Received: OFF");
    #endif
    setRelay(RELAYOFF);
  }
}
//****************************************************************************

//** send MQTT data
//****************************************************************************
void mqttSendData()
{
  //--------------------------------------------------------------------------
  // Read DS18B20 sensors
  //--------------------------------------------------------------------------
  #if defined(DEBUG)
    Serial.println("");
    Serial.println("Requesting temperatures...");
  #endif
  // Send the command to get temperatures
  sensors.requestTemperatures();

  temp_1 = sensors.getTempC(ds18b20_1);
  temp_2 = sensors.getTempC(ds18b20_2);
  temp_3 = sensors.getTempC(ds18b20_3);

  #if defined(DEBUG)
    // display temps
    Serial.print("Sensor 1 :"); Serial.printf(" %3.1f C\n", temp_1);
    Serial.print("Sensor 2 :"); Serial.printf(" %3.1f C\n", temp_2);
    Serial.print("Sensor 3 :"); Serial.printf(" %3.1f C\n", temp_3);
  #endif

  if(useLED)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  dt = ntpLocalDateTime();

  #if defined(DEBUG)
    Serial.println(F("Sending data via MQTT"));
  #endif
  strcpy(caJson, "{ ");
  sprintf(caTemp, "\"IP\": \"%s\", ", WiFi.localIP().toString().c_str());
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Hostname\": \"%s\", ", hostname);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Location\": \"%s\", ", location);
  strcat(caJson, caTemp);
  dt = ntpLocalDateTime();
  sprintf(caTemp, "\"TimeStamp\": \"%s\" ", dt);
  strcat(caJson, caTemp);
  strcat(caJson, "}");
  mqttSend(mqttTopicDat1, caJson);
  #if defined(DEBUG)
    Serial.printf("  --> %s\n", mqttTopicDat1);
    Serial.printf("      %s\n", caJson);
  #endif

  strcpy(caJson, "{ ");
  sprintf(caTemp, "\"Volt\": %.0f, ", rmsValV);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Amps\": %.1f, ", rmsValA);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Power\": %.0f, ", rmsValV * rmsValA);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Temp1\": %.1f, ", temp_1);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Temp2\": %.1f, ", temp_2);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Temp3\": %.1f, ", temp_3);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Out\": %u, ", upct);
  strcat(caJson, caTemp);
  sprintf(caTemp, "\"Relay\": %u", relay_state);
  strcat(caJson, caTemp);
  strcat(caJson, "}");
  mqttSend(mqttTopicDat2, caJson);
  #if defined(DEBUG)
    Serial.printf("  --> %s\n", mqttTopicDat2);
    Serial.printf("      %s\n", caJson);
  #endif
  mqttSend(mqttTopicSts, "online");
  if(useLED)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** set Relay
//****************************************************************************
void setRelay(bool OnOff)
{

  if(OnOff == RELAYON)
  {
    // set output high
    digitalWrite(RELAY_GPIO, RELAYON);
    #if defined(DEBUG)
      Serial.println("Relay ON");
    #endif
  }
  else
  {
    // set output low
    digitalWrite(RELAY_GPIO, RELAYOFF);
    #if defined(DEBUG)
      Serial.println("Relay OFF");
    #endif
  }

  if (digitalRead(RELAY_GPIO) == RELAYON)
  {
    relay_state = 1;
  }
  else
  {
    relay_state = 0;
  }
  mqttSendData();
}
//****************************************************************************

//** set DAC
//****************************************************************************
void setDAC(char *percent)
{

  // convert string to float and then to a DAC number
  upct = (unsigned int)(abs(atoi(percent)));

  // keep between boundaries
  if(upct < 0)
  {
    upct = 0;
  }
  else if (upct > 100)
  {
    upct = 100;
  }

  dacn = (unsigned int)((255 * upct) / 100);
  #if defined(DEBUG)
    Serial.printf("Setting DAC: PCT: %u - DACN: %u\n", upct, dacn);
  #endif
  dacWrite(DAC_GPIO, dacn);
  mqttSendData();

}
//****************************************************************************

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ ADC VA Functions +++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//** reading voltage
//****************************************************************************
void readVolt(bool ADC, bool RMS)
{

  float sum;

  // read the ADC and save value
  valZERv[cntADC] = (unsigned int)(analogRead(ZMPT101B_GPIO));
  ADCval          = (float)(valZERv[cntADC]);
  ADCval          = (ADCval - ADCzeroV) * ADCvolt;
  valADCv[cntADC] = (float)(ADCval);

  if( ADC )
  {
    // find DC offset
    // skip first as it might not be correct due to long loop
    sum = 0;
    for (int n=1; n<iADC; n++)
    {
      sum = sum + (float)valZERv[n];
    }
    avgZERv[cntRMS] = (unsigned int)(sum / (iADC - 1));
    // adjust Zero offset for current
    ADCzeroV = (float)avgZERv[cntRMS];

    // find RMS
    // RMS = the square root of the mean square (the arithmetic mean of the squares) of the set
    float valV;
    sum = 0;
    // skip first as it might not be correct due to long loop
    for (int n=1; n<iADC; n++)
    {
      valV = (float)valADCv[n];
      sum  = sum + (float)(valV * valV);
    }
    valRMSv[cntRMS] = (float)(sqrt(sum / (iADC - 1)));
  }

  if( RMS )
  {
    // find average RMS
    sum = 0;
    for (int n=0; n<iRMS; n++)
    {
      sum = sum +(float)valRMSv[n];
    }
    // The average RMS value of ADC values
    rmsValV = (float)(sum / iRMS);

    #if defined(DEBUG)
      Serial.printf(" Line Voltage   : %4.0f Vrms\n", rmsValV);
    #endif
  }
}
//****************************************************************************


//** reading amperes
//****************************************************************************
void readAmps(bool ADC, bool RMS)
{

  float sum;

  // read the ADC and save value
  valZERa[cntADC] = (unsigned int)(analogRead(ACS712_GPIO));
  ADCval          = (float)(valZERa[cntADC]);
  ADCval          = (ADCval - ADCzeroA) * ADCamps;
  valADCa[cntADC]  = (float)ADCval;

  if( ADC )
  {
    // find DC offset
    // skip first as it might not be correct due to long loop
    sum = 0;
    for (int n=1; n<iADC; n++)
    {
      sum = sum + (float)valZERa[n];
    }
    avgZERa[cntRMS] = (unsigned int)(sum / (iADC - 1));
    // adjust Zero offset for current
    ADCzeroA = (float)avgZERa[cntRMS];

    // find RMS
    // RMS = the square root of the mean square (the arithmetic mean of the squares) of the set
    float valV;
    sum = 0;
    // skip first as it might not be correct due to long loop
    for (int n=1; n<iADC; n++)
    {
      valV = (float)valADCa[n];
      sum  = sum + (float)(valV * valV);
    }
    valRMSa[cntRMS] = (float)(sqrt(sum / (iADC - 1)));
  }

  if( RMS )
  {
    // find average RMS
    sum = 0;
    for (int n=0; n<iRMS; n++)
    {
      sum = sum +(float)valRMSa[n];
    }
    // The average RMS value of ADC values
    rmsValA = (float)(sum / iRMS);

    #if defined(DEBUG)
      Serial.printf(" Line Amperes   : %4.2f Arms\n", rmsValA);
    #endif
  }
}
//****************************************************************************
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//============================================================================
void setup()
{
  #if defined(DEBUG)
    Serial.begin(115200);
    delay(1000);
    Serial.println(F(""));Serial.println(F(""));
    Serial.println(F("======================="));
    Serial.print(F("Booting (Version: ")); Serial.print(VERSION); Serial.println(F(")"));
    Serial.println(F("======================="));
  #endif

  //--------------------------------------------------------------------------
  // Set led pin as output
  //--------------------------------------------------------------------------
  pinMode(LED_BUILTIN, OUTPUT);
 // Start ticker to indicate set-up mode
  ticker.attach(0.5, tick);

  //--------------------------------------------------------------------------
  // Startup WiFi
  //--------------------------------------------------------------------------
  String sTmp = wifiMACAddress;
  #if defined(DEBUG)
    WiFi.onEvent(wifiEvent);
  #endif
  // setup hostname
  sTmp.replace(":", "");
  sTmp.toUpperCase();
  sTmp.toCharArray(caMAC, 20);
  strcpy(hostname, "ESP32-BC-");
  strcat(hostname, caMAC);
  wifiConnect();

  #if defined(DEBUG)
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("  MAC Address: "); Serial.println(wifiMACAddress);
    Serial.print("  IP Address : "); Serial.println(WiFi.localIP());
    Serial.print("  Hostname   : "); Serial.println(WiFi.getHostname());
  #endif
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup time
  //--------------------------------------------------------------------------
  ntpSetup();
  // get time
  dt = ntpLocalDateTime();
  strncpy(today, dt, 10);
  today[10] = NULL;
  #if defined(DEBUG)
    Serial.print(dt); Serial.print(F(" - Today : [")); Serial.print(today); Serial.println(F("]"));
  #endif

  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup MDNS
  //--------------------------------------------------------------------------
  mdnsSetup();
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup OTA
  //--------------------------------------------------------------------------
  otaSetup();
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Stop Bluetooth
  //--------------------------------------------------------------------------
  btStop();
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup mqtt
  //--------------------------------------------------------------------------
  mqttConnect();
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup DS18B20 sensors
  //--------------------------------------------------------------------------
  sensors.begin();
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Set-up Relay port
  //--------------------------------------------------------------------------
  pinMode(RELAY_GPIO, OUTPUT);
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Startup timer interrupt
  //--------------------------------------------------------------------------
  //set prescaler every microsecond eg: clockspeed 80.000.000Hz / 80 = 1.000.000Hz -> 1 µs period
  timer = timerBegin(0, CLOCKSPEED, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // interrupt every 1 millisecond = 1000 µs
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
  //--------------------------------------------------------------------------

  // Ending set-up mode, turn led off to save power
  ticker.detach();
  digitalWrite(LED_BUILTIN, LOW);

  //--------------------------------------------------------------------------
  // init Realy and DAC
  //--------------------------------------------------------------------------
  setRelay(RELAYOFF);
  setDAC(itoa(upct,caTemp,10));
  //--------------------------------------------------------------------------

  #if defined(DEBUG)
    Serial.println(F("Ready"));
  #endif
}
//============================================================================

//============================================================================
void loop()
{

  //--------------------------------------------------------------------------
  // OTA handling
  //--------------------------------------------------------------------------
  ArduinoOTA.handle();

  //--------------------------------------------------------------------------
  // Read ADC
  //--------------------------------------------------------------------------

  // interrupt occured if cntrINT > 1
  if (cntrINT > 0)
  {
    cntADC++;
    // if array is full, calc rms
    if(cntADC >= (iADC - 1))
    {
      cntRMS++;

      // if array is full, calc average rms
      if(cntRMS >= (iRMS - 1))
      {
        readVolt(true, true);
        readAmps(true, true);
        // reset counter
        cntADC = -1;
        cntRMS = -1;
      }
      else
      {
        readVolt(true, false);
        readAmps(true, false);
        // reset counter
        cntADC = -1;
      }
    }
    else
    {
      readVolt(false, false);
      readAmps(false, false);
    }

    // to make sure we do not handle pilled up interrupts
    // and keep a pace of 1 every ms
    portENTER_CRITICAL(&timerMux);
    cntrINT = 0;
    portEXIT_CRITICAL(&timerMux);
  }

  //--------------------------------------------------------------------------
  // MQTT handling
  //--------------------------------------------------------------------------
  if (!mqttClient.connected())
  {
    mqttReconnect();
  }
  // start mqtt listener
  mqttClient.loop();

  every(mqttInterval * 1000)
  {
    mqttSendData();
  }

  //--------------------------------------------------------------------------
  // reboot every day around midnight
  //--------------------------------------------------------------------------
  // check only so often = 5 minutes, not every loop
  every(300 * 1000)
  {
    char  nu[11];
    dt = ntpLocalDateTime();
    // only compare date , not time
    strncpy(nu, dt, 10);
    nu[10] = NULL;

    #if defined(DEBUG)
      Serial.print(F(" - Today : [")); Serial.print(today); Serial.println(F("]"));
      Serial.print(F(" - Now   : [")); Serial.print(nu);    Serial.println(F("]"));
    #endif

    // if (strcmp(today, nu) < 0)
    // {
      // #if defined(DEBUG)
        // Serial.println(F("Restarting..."));
      // #endif
      // ESP.restart();
    // }
    strcpy(today, nu);
  }

}
