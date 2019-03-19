int received;

void setup() 
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  //Serial.println("Listen up little wolf");
  // put your setup code here, to run once:
}

void loop() 
{
  if(Serial.available() > 0)
  {
    //String toPrint = Serial.readString();
    Serial.println(Serial.readString());
  }
  
  digitalWrite(13, HIGH);
  delay(45);
  digitalWrite(13, LOW);
  delay(45);
  digitalWrite(13, HIGH);
  delay(45);
  digitalWrite(13, LOW);
  delay(360);
  
  // put your main code here, to run repeatedly:

}
