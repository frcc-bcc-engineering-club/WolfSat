bool passed;
void setup() 
{
  passed = false;
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  Serial.println(":: LittleWolf Listener ::");
  Serial.flush();
  // put your setup code here, to run once:
}

void loop() 
{
  if (Serial.available() > 0)
  {
    String local = Serial.readStringUntil('\r');
    if((passed == false)&&(local == "Listen up LittleWolf"))
    {
      Serial.println("I'm listening...");
      passed = true;
    }
    
    Serial.print(local);
  }

}
