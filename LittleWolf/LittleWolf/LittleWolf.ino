int received;

void setup() 
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Serial.println("Listen up little wolf");
  Serial.flush();
  // put your setup code here, to run once:
}

void loop() 
{
  Serial.flush();
  if(Serial.available() > 0)
  {
    String toPrint = Serial.readStringUntil('\r');
    Serial.print(toPrint);
  } 
  // put your main code here, to run repeatedly:

}
