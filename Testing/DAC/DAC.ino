//DAC output port 
#define DAC_GPIO  26

void setup()
{
  Serial.begin(115200);
}
 
void loop()
{
  for(int i = 0; i < 256; i+=16)
  {
    dacWrite(DAC_GPIO, i);
    Serial.printf(" %d \n", i); 
    delay(1000);            
  }
  dacWrite(DAC_GPIO, 255);  
  Serial.println("Max (255) reached"); 
  delay(5000);
  
  for(int i = 254; i > 0; i-=16)
  {
    dacWrite(DAC_GPIO, i);
    Serial.printf(" %d \n", i);
    delay(1000);            
  } 
  dacWrite(DAC_GPIO, 0);
  Serial.println("Min (0) reached"); 
  delay(5000);
  
}
