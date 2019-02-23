#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <Wire.h>
//#include <wolf.h>

//wolf master;
OpenLog logOne;
int x;
int LED = 13;

//#include <SPI.h>  

void setup() {
//  master = wolf();
  // put your setup code here, to run once:
  Wire.begin();
  logOne.begin();
  Serial.begin(9600);
  Serial.println("BOOT");
  x = 0;
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  x++;
  logOne.println("X = " + String(x));
  Serial.println("X = " + String(x));
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}



