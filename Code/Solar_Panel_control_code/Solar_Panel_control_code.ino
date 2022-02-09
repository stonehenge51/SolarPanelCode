#define direction_signal_M1 2
#define pulse_signal_M1 3
#define on_signal_M1 4

#define direction_signal_M2 5
#define pulse_signal_M2 6
#define on_signal_M2 7

#define len 1

int val = 0; 

void setup() {
  pinMode(direction_signal_M1, OUTPUT);
  pinMode(pulse_signal_M1, OUTPUT);
  pinMode(on_signal_M1, INPUT);

  pinMode(direction_signal_M2, OUTPUT);
  pinMode(pulse_signal_M2, OUTPUT);
  pinMode(on_signal_M2, INPUT);

}

void loop() {
  val = digitalRead(on_signal_M1);
  if(val == HIGH){
    digitalWrite(direction_signal_M1, HIGH);
    delay(1);
    digitalWrite(pulse_signal_M1, HIGH);
    delay(len);
    digitalWrite(pulse_signal_M1, LOW);
    delay(len);  
  }
  if(val == HIGH){
    digitalWrite(direction_signal_M1, LOW);
  }
  

}
