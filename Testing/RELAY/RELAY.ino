// set pin numbers
#define RELAY_GPIO  33


void setup() 
{
  Serial.begin(115200);  
  // initialize GPIO as an output
  pinMode(RELAY_GPIO, OUTPUT);
}

void loop() 
{
  // set output high
  digitalWrite(RELAY_GPIO, HIGH);
  Serial.println("Output HIGH"); 
  delay(5000);
  // set output low
  digitalWrite(RELAY_GPIO, LOW);
  Serial.println("Output LOW"); 
  delay(5000);
}
