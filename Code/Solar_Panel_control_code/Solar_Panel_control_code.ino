#define out 2
#define len 500
void setup() {
  pinMode(out, OUTPUT);

}

void loop() {
  digitalWrite(out, HIGH); // sets the digital pin 13 on
  delay(len);            // waits for a second
  digitalWrite(out, LOW);  // sets the digital pin 13 off
  delay(len);  

}
