#include <Wire.h>
#include <SparkFun_RV8803.h>

RV8803 rtc;

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

double elevation = 0.0;
double azimuth = 0.0;
int right = 0;
int left = 0;
int up = 0;
int down = 0;
int counter = 0;

byte Version[3];
int8_t x_data;
int8_t y_data;
int8_t z_data;


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
  
//  Wire.beginTransmission();

  if(rtc.updateTime() == true)
  {
    second = rtc.getSeconds();
    minute = rtc.getMinutes();
    hour = rtc.getHours();
    day = rtc.getDate();
    month = rtc.getMonth();
    year = rtc.getYear();
    Tgmt = 8 + hour + minute/60 + second/3600;

//    Serial.print("Year =  ");
//    Serial.println(year);
//    Serial.print("Month =  ");
//    Serial.println(month);
//    Serial.print("Day =  ");
//    Serial.println(day);
//    Serial.print("Hour =  ");
//    Serial.println(hour);
//    Serial.print("Minute =  ");
//    Serial.println(minute);
//    Serial.print("Second =  ");
//    Serial.println(second);
    
    solarzenithelevation(year, month, day, hour, minute, Tgmt);
//    Serial.print("Elevation = ");
//    Serial.println(elevation);
//    Serial.print("Azimuth = ");
//    Serial.println(azimuth);
    
    delay(10000);
//    Serial.println(minute);
  }

  AccelerometerInit(); 
  
  right = digitalRead(M1_right);
  left = digitalRead(M1_left);
  up = digitalRead(M2_up);
  down = digitalRead(M2_down);

  if(right == LOW){
    if(counter == 0){
      counter = 1;
      setdirection(0,1);
    }
    pulse(pulse_signal_M1, len1);        
  }
  else{
    counter = 0;
    setdirection(0,0);
  }
  if(left == LOW){
    if(counter == 0){
      counter = 1;
      setdirection(0,0);
    }
    pulse(pulse_signal_M1, len1);
  }
  else{
    counter = 0;
  }

  if(up == LOW){
    if(counter == 0){
      counter = 1;
      setdirection(1,1);
    }
    pulse(pulse_signal_M2, len2);        
  }
  else{
    counter = 0;
    setdirection(1,0);
  }
  if(down == LOW){
    if(counter == 0){
      counter = 1;
      setdirection(1,0);
    }
    pulse(pulse_signal_M2, len2);
  }
  else{
    counter = 0;
  }
  
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
   Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x04); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    Version[0] = Wire.read(); // receive a byte as characte
  }  
  x_data=(int8_t)Version[0]>>2;
 
  Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x06); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    Version[1] = Wire.read(); // receive a byte as characte
  }  
  y_data=(int8_t)Version[1]>>2;
  
  Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x08); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
   while(Wire.available())    // slave may send less than requested
  { 
    Version[2] = Wire.read(); // receive a byte as characte
  }  
   z_data=(int8_t)Version[2]>>2; 
   
   Serial.print("X=");   
   Serial.print(x_data);         // print the character
   Serial.print("  "); 
   Serial.print("Y=");   
   Serial.print(y_data);         // print the character
   Serial.print("  "); 
   Serial.print("Z=");  
   Serial.println(z_data);   
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
