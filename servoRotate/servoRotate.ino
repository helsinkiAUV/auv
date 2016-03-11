
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}

void loop() {
  digitalWrite(2, HIGH);  
  digitalWrite(3, LOW);
  delay(1000);              
  digitalWrite(2, LOW);    
  digitalWrite(3, HIGH);
  delay(1000);              
}
