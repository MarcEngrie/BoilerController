#include <OneWire.h>
#include <DallasTemperature.h>

// uncomment to list all DS18B20 addresses
//#define LISTDS

//OneWire data line connected to GPIO 
#define ONE_WIRE_BUS  4

// create objects required
OneWire           oneWire(ONE_WIRE_BUS);
#if not defined(LISTDS)
  DallasTemperature sensors(&oneWire);

  // these can be obtained using the LISTDS code
  DeviceAddress ds18b20_1 = { 0x28, 0x2F, 0x9C, 0x56, 0xB5, 0x01, 0x3C, 0x9B };
  DeviceAddress ds18b20_2 = { 0x28, 0xFF, 0x64, 0x02, 0xC8, 0x7D, 0x29, 0xB7 };
  DeviceAddress ds18b20_3 = { 0x28, 0xFF, 0x64, 0x1E, 0x0C, 0x04, 0xC5, 0x1E };
#endif

// setup
void setup(void) 
{
  Serial.begin(115200);
  #if not defined(LISTDS)
    sensors.begin();
  #endif
}

// loop
void loop(void) 
{

  #if defined(LISTDS)
  
    byte addr[8];
    static int cnt = 0;
    
    if (!oneWire.search(addr)) 
    {
      Serial.println();
      oneWire.reset_search();
      cnt = 0;
      delay(3000);
      return;
    }
    else
    {
      cnt++;
    }
    
    Serial.printf("DeviceAddress ds18b20_%d = {", cnt);
    for (byte i = 0; i < 8; i++) 
    {
      if(addr[i] < 16)
      {
        Serial.write(" 0x0"); Serial.print(addr[i], HEX);
      }
      else
      {
        Serial.write(" 0x"); Serial.print(addr[i], HEX);
      }
      if(i < 7)
      {
        Serial.write(",");
      }
    }
    Serial.println(" };");
    
  #else
  
    Serial.println("");
    Serial.println("Requesting temperatures...");
    // Send the command to get temperatures
    sensors.requestTemperatures(); 
  
    // display temps
    Serial.print("Sensor 1 :"); Serial.printf(" %3.1f C\n", sensors.getTempC(ds18b20_1)); 
    Serial.print("Sensor 2 :"); Serial.printf(" %3.1f C\n", sensors.getTempC(ds18b20_2)); 
    Serial.print("Sensor 3 :"); Serial.printf(" %3.1f C\n", sensors.getTempC(ds18b20_3)); 

    delay(2000);
    
  #endif
  
  
 
}
