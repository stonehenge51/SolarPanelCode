#include <Wire.h>
#include <SparkFun_RV8803.h>

RV8803 rtc;

#define Manual 4
#define direction_signal_M1 3
#define pulse_signal_M1 2

#define direction_signal_M2 7
#define pulse_signal_M2 8

#define M1_right 9
#define M1_left 12

#define M2_up 11
#define M2_down 10

#define len1 15
#define len2 1

const int DigComp_ADDR     = 0x60; // I2C Address of the sensor
const uint8_t comp1            = 0x02;
const uint8_t comp2            = 0x03;
const uint8_t XAXIS1           = 12;
const uint8_t XAXIS2           = 13;
const uint8_t YAXIS1           = 14;
const uint8_t YAXIS2           = 15;
const uint8_t ZAXIS1           = 16;
const uint8_t ZAXIS2           = 17;
const uint8_t Pitch            = 4;
const uint8_t Roll             = 5;

double elevation = 0.0;
double azimuth = 0.0;
int manual = 0;
int right = 0;
int left = 0;
int up = 0;
int down = 0;
int counter = 0;

unsigned int compassdirection = 0;
int x_data;
int y_data;
int z_data;
int8_t pitchAngle = 0;
int8_t rollAngle = 0;


void setup() {

  Wire.begin();
  Serial.begin(9600);

  if(rtc.begin() == false)
  {
    Serial.println("Something went wrong, check wiring");
    while(1);
  }
  
  Serial.println("RTC online!");
  
  if(rtc.is12Hour() == true)
  {
    rtc.set24Hour();
  }

//  rtc.setYear(2022);
//  rtc.setMonth(3);
//  rtc.setDate(9);
//  rtc.setHours(12);
//  rtc.setMinutes(5);

  pinMode(direction_signal_M1, OUTPUT);
  pinMode(pulse_signal_M1, OUTPUT);
  pinMode(direction_signal_M2, OUTPUT);
  pinMode(pulse_signal_M2, OUTPUT);

  pinMode(Manual , INPUT);
  pinMode(M1_left, INPUT_PULLUP);
  pinMode(M1_right, INPUT_PULLUP);
  pinMode(M2_up, INPUT_PULLUP);
  pinMode(M2_down , INPUT_PULLUP);

}

void loop() {
  double second = 0.0;
  double minute = 0.0;
  double hour = 0.0;
  double day = 0.0;
  double month = 0.0;
  double year = 0.0;
  double Tgmt = 0.0;

  if(rtc.updateTime() == true)
  {
    second = rtc.getSeconds();
    minute = rtc.getMinutes();
    hour = rtc.getHours();
    day = rtc.getDate();
    month = rtc.getMonth();
    year = rtc.getYear();
    Tgmt = 8 + hour + minute/60 + second/3600;
    solarzenithelevation(year, month, day, hour, minute, Tgmt);

    compassdirection = highLowByteRead(comp1, comp2);
    AccelerometerInit();
//    Serial.print("Compass Direction = ");
//    Serial.println(compassdirection);
    delay(1000);
  }
  
   

  manual = digitalRead(Manual);
  

  if(manual == LOW){
    right = digitalRead(M1_right);
    left = digitalRead(M1_left);
    up = digitalRead(M2_up);
    down = digitalRead(M2_down);
    
    if(right == LOW)turnRight();
    else{
      counter = 0;
      setdirection(0,0);
    }
    if(left == LOW)turnLeft();
    else counter = 0;
  
    if(up == LOW)increaseElevation();       
    else{
      counter = 0;
      setdirection(1,0);
    }
    if(down == LOW)decreaseElevation();
    else counter = 0;
  }
  
  if(manual == HIGH)autoController();
  
}

void autoController(){

  int elev_diff = 0;
  pitchAngle = (pitchAngle - 90) * -1;
  elev_diff = pitchAngle - elevation;
  
  Serial.print("elev_diff = ");
  Serial.println(elev_diff);
//  Serial.println(pitchAngle);
  if (pitchAngle > 90){
    decreaseElevation();
//    Serial.println("Lowering to starting position");
  }
  else counter = 0;
  
  if (pitchAngle <= 90){
    if (elev_diff > 4){
      decreaseElevation();
//      Serial.println("Lowering");
    }
    else counter = 0;
  }

  if (pitchAngle > 0){
    if (elev_diff < -2){
      increaseElevation();
//      Serial.println("Raising");
    }
    else{
        counter = 0;
        setdirection(1,0);
      }
  }

  float azim_diff = 0;
  float compassdir = 0.0;
  compassdir = compassdirection;
  compassdir = compassdir/10;
  azim_diff = azimuth - compassdir;
  Serial.print("azim_diff = ");
  Serial.println(azim_diff);
  Serial.println(compassdir);

  if (azimuth < 90){
    decreaseElevation();
//    Serial.println("Lowering to starting position");
  }
  else counter = 0;
  
  if (1 == 1){
    if (azim_diff <= -2){
      turnLeft();
      Serial.println("Turning left");
    }
    else counter = 0;
  }

  if (1 == 1){
    if (azim_diff > 2){
      turnRight();
      Serial.println("Turning right");
    }
    else{
        counter = 0;
        setdirection(1,0);
      }
  }
  
}

void turnRight(){
  if(counter == 0){
    counter = 1;
    setdirection(0,1);
  }
  pulse(pulse_signal_M1, len1); 
}
void turnLeft(){
  if(counter == 0){
    counter = 1;
    setdirection(0,0);
  }
  pulse(pulse_signal_M1, len1);
}
void increaseElevation(){
  if(counter == 0){
    counter = 1;
    setdirection(1,0);
  }
  pulse(pulse_signal_M2, len2);
}
void decreaseElevation(){
  if(counter == 0){
    counter = 1;
    setdirection(1,0);
   }
   pulse(pulse_signal_M2, len2);
}
void pulse(int pin, int len){
  digitalWrite(pin, LOW);
  delay(len);
  digitalWrite(pin, HIGH);
  delay(len);
  digitalWrite(pin, LOW);
}

void setdirection(int output, int value){
  delayMicroseconds(20);
  if(output == 0){
    if(value == 1) digitalWrite(direction_signal_M1, HIGH);
    if(value == 0) digitalWrite(direction_signal_M1, LOW);
  }
  if(output == 1){
    if(value == 1) digitalWrite(direction_signal_M2, HIGH);
    if(value == 0) digitalWrite(direction_signal_M2, LOW);
  }
}

void AccelerometerInit() 
{
//  int Xaxis = 0;
//  int Yaxis = 0;
//  int Zaxis = 0;
  
//  Xaxis = highLowByteRead(XAXIS1, XAXIS2);
//  x_data = Xaxis;
//  Serial.print("XAXIS = ");
//  Serial.println(Xaxis);

//  Yaxis = highLowByteRead(YAXIS1, YAXIS2);
//  y_data = Yaxis;
//  Serial.print("YAXIS = ");
//  Serial.println(Yaxis);

//  Zaxis = highLowByteRead(ZAXIS1, ZAXIS2);
//  z_data = Zaxis;
//  Serial.print("ZAXIS = ");
//  Serial.println(Zaxis);

  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(Pitch); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    pitchAngle = Wire.read(); // receive a byte as characte
  }
//  Serial.print("pitch = ");
//  Serial.println(pitchAngle);
  
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(Roll); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    rollAngle = Wire.read(); // receive a byte as characte
  }
//  Serial.print("roll = ");
//  Serial.println(rollAngle);
      
} 
int highLowByteRead(uint8_t addresshigh, uint8_t addresslow){
  byte high = 0;
  byte low = 0;
  int value = 0;
  
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(addresshigh); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    high = Wire.read(); // receive a byte as characte
  }  
  value = high<<8;
 
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(addresslow); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    low = Wire.read(); // receive a byte as characte
  }  
  value += low;
  return value; 
}

void solarzenithelevation(double year, double month, double day, double hour, double minute, double Tgmt){
  double lambdaO = -123.12722630891494 * PI/180;
  double psiO = 49.17491793381123 * PI/180;
  double delta = sundeclination(year, month, day, hour+7.0, minute);
  double Emin = equationoftime(year, month, day, hour, minute);
  double psiS = delta;
  double lambdaS = -15*(Tgmt - 12 + Emin/60.0) * PI/180.0;
  double Sx = cos(psiS)* sin(lambdaS - lambdaO);
  double Sy = cos(psiO) * sin(psiS) - sin(psiO) * cos(psiS) * cos(lambdaS - lambdaO);
  double Sz = sin(psiO) * sin(psiS) + cos(psiO) * cos(psiS) * cos(lambdaS - lambdaO);
  double Z = asin(Sz);
  double ys = atan2(Sx,Sy);

  elevation = Z * 180/PI;
  azimuth = ys * 180/PI;
  if(azimuth < 0.0){
    azimuth += 360;
  }
}

double rangecheck(double value){
  while(value < 0){
    value = value + 360;
  }
  while(value > 360){
    value = value - 360;
  }

  return value;
}

double sundeclination(double year, double month, double day, double hour, double minute){
  double N = Ncalc(year, month, day, hour, minute);
  double declination = asin(sin(-23.44*DEG_TO_RAD) * cos(2*PI/365.24*(N+10) + 2*0.0167 * sin(2*PI/365.24*(N-2))));
  return declination;
}

double greenwichmeantime(int switchstate, int timedata []){


  if(switchstate == 1){
    return timedata[3] + (double)timedata[4]/60 + (double)timedata[5]/(60*60);
  }
  return 0.0;
}

double equationoftime(double year, double month, double day, double hour, double minute){
  double timezone = -7;

  double aaa = 367 * year - 730531.5;
  double bbb = -int((7 * int(year + (month + 9)/12))/4);
  double ccc = int(275 * month/9) + day;
  double Dtoday = (hour + minute/60 - timezone)/24;
  double Ddays = aaa + bbb + ccc + Dtoday;
  double Cycle = int(Ddays / 365.25);
  double thetarad = 0.0172024 * (Ddays - 365.25 * Cycle);
  double amp1 = 7.36303 - Cycle * 0.00009;
  double amp2 = 9.92465 - Cycle * 0.00014;
  double phi1 = 3.07892 - Cycle * 0.00019;
  double phi2 = -1.38995 + Cycle * 0.00013;
  double EoT1 = amp1 * sin(1 * (thetarad + phi1));
  double EoT2 = amp2 * sin(2 * (thetarad + phi2));
  double EoT3 = 0.31730 * sin(3 * (thetarad - 0.94686));
  double EoT4 = 0.21922 * sin(4 * (thetarad - 0.60716));
  double EoTmins = 0.00526 + EoT1 + EoT2 + EoT3 + EoT4;

  return EoTmins;
}

double Ncalc(int year, double month, double day, double hour, double minute){
  double daysMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  double days = 0.0;

  for(int i=0; i < month-1; i++){
    double num = year % 4;
    if(num != 0){
      days = days + daysMonth[i];
    }
    
    else{
      if(i == 1){
        days = days + daysMonth[i] + 1;
      }
    }
  }
  return days + day + hour/24 + minute/1440;
}
