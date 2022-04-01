#include <Wire.h>
#include <SparkFun_RV8803.h>
RV8803 rtc;

//these define which pins control the stepper motors direction and pulse inputs
#define direction_signal_M1 3
#define pulse_signal_M1 2
#define direction_signal_M2 7
#define pulse_signal_M2 8

//these define the pins used for the manual operation buttons for controlling the stepper motors
#define M1_right 9
#define M1_left 12
#define M2_up 11
#define M2_down 10

//this defines the pin which controls whether the code is operating in manual or automated operation
#define Manual 4

//these define the pins used for the calibration hall sensors
#define Hall90 5
#define Hall180 6

//these define the length of delay between each pulse to the stepper motor(minimum value of 5 micro seconds 
#define len1 40 //mili seconds
#define len1a 15 //mili seconds
#define len2 900 //micro seconds

//global variables for storing the time data from the RTC(RV8803)
double second = 0.0;
double minute = 0.0;
double hour = 0.0;
double day = 0.0;
double month = 0.0;
double year = 0.0;
double Tgmt = 0.0; //this is calculated from the current RTC time

const int DigComp_ADDR     = 0x60; //I2C Address of the digital compass sensor
const uint8_t comp1            = 2; //high byte address for compass direction (unsigned 16 bit integer) 
const uint8_t comp2            = 3; //low  byte address for compass direction (unsigned 16 bit integer) 
const uint8_t Pitch            = 4; //pitch angle address for digital compass (signed 8 bit integer)
const uint8_t Roll             = 5; //roll angle address for digital compass (signed 8 bit integer)
const uint8_t caliadd          = 30; //sensor calibration address for digital compass (4 - unsigned 2 bit integers)

//the global variables for storing the azimuthal and elevation angles calculated for RTC time data
double elevation = 0.0; 
double azimuth = 0.0;

//the global variables for storing DIO pins HIGH or LOW status
int manual = 0;
int right = 0;
int left = 0;
int up = 0;
int down = 0;

int counter = 0; //a time saving counter that blocks additional resetting of the stepper motor direction pin
int steps = 0; //counts the steps taken by the azimuthal motor
int cycle_counter = 0; //for slowing the cycle rate of non essential functions/code executions
float Nratio = 1.7/(13.0)*1.45;

float correctiondata[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int index = 0;

bool startup = true;
int hall90 = 0;
int hall180 = 0;

unsigned int compassdirection = 0;
int x_data;
int y_data;
int z_data;
int8_t pitchAngle = 0;
int8_t rollAngle = 0;

int half = 0;
int elev_diff = 0;

float azim_diff = 0.0;
float azimutherror = 0.0;
float compassdir = 90.0;

uint8_t calbyte = 0;
uint8_t calsys = 0;
uint8_t calgyro = 0;
uint8_t calaccel = 0;
uint8_t calmagn = 0;



void setup() {

  Wire.begin();
  Serial.begin(57600);
  Serial.flush();

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

  pinMode(direction_signal_M1, OUTPUT);
  pinMode(pulse_signal_M1, OUTPUT);
  pinMode(direction_signal_M2, OUTPUT);
  pinMode(pulse_signal_M2, OUTPUT);

  pinMode(Manual , INPUT);
  pinMode(Hall90 , INPUT);
  pinMode(Hall180 , INPUT);
  pinMode(M1_left, INPUT_PULLUP);
  pinMode(M1_right, INPUT_PULLUP);
  pinMode(M2_up, INPUT_PULLUP);
  pinMode(M2_down , INPUT_PULLUP);

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
      }

}

void loop() {
//  while(startup == true)calibration();
  
  if(cycle_counter == 51){
    cycle_counter = 0;
  }

  if(cycle_counter == 50){
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
//        compassdirection = highLowByteRead(comp1, comp2);
      AccelerometerInit();
//        Serial.print("compassdirection = ");
//        Serial.println(compassdirection);
      Serial.print("compassdir = ");
      Serial.println(compassdir);
    }
  }

  manual = digitalRead(Manual);
  delay(1);
  hall90 = digitalRead(Hall90);
  delay(1);
  hall180 = digitalRead(Hall180);
  delay(1);
  
  if(hall90 == HIGH){
    if(cycle_counter == 50)Serial.println("90");
    azimutherror = 95 - compassdir;
    correctiondata[index] = azimutherror;    
  }
  
  if(hall180 == HIGH){
    if(cycle_counter == 50)Serial.println("180");
    azimutherror = 182 - compassdir;
    correctiondata[index] = azimutherror;
  }
  
  if(manual == LOW){
    right = digitalRead(M1_right);
    delay(1);
    left = digitalRead(M1_left);
    delay(1);
    up = digitalRead(M2_up);
    delay(1);
    down = digitalRead(M2_down);
    delay(1);
    
    if(right == LOW)turnRight(len1a);
    else{
      counter = 0;
      setdirection(0,0);
    }
    if(left == LOW)turnLeft(len1a);
    else counter = 0;
  
    if(up == LOW)increaseElevation(len2);       
    else{
      counter = 0;
      setdirection(1,0);
    }
    if(down == LOW)decreaseElevation(len2);
    else counter = 0;
  }

  if(manual == HIGH)autoController();
  cycle_counter += 1;
}

void calibration(){
  //add hall sensor calibration code
  if(0 > -1){
    Serial.println("Yes");    
  }
  hall90 = digitalRead(Hall90);
  delay(1);
  hall180 = digitalRead(Hall180);
  delay(1);

  if(hall90 == HIGH){
    if(cycle_counter == 50)Serial.println("90 Startup complete");
    compassdir = 95;
    startup = false;
  }
  
  if(hall180 == HIGH){
    if(cycle_counter == 50)Serial.println("180 Startup complete");
    compassdir = 182;
    startup = false;
  }
  turnLeft(len1);
}

void autoController(){

  if(cycle_counter == 50){
    pitchAngle = (pitchAngle - 90) *-1;
//    elevation = 90.0;  //maunal elevation
    elev_diff = pitchAngle - elevation;
  }
  if (pitchAngle > 90){
    decreaseElevation(len2);
  }
  else counter = 0;
  
  if (pitchAngle <= 90){
    if (elev_diff > 2){
      decreaseElevation(len2);
    }
    else counter = 0;
  }

  if (pitchAngle > 0){
    if (elev_diff < -2){
      increaseElevation(len2);
    }
    else{
        counter = 0;
        setdirection(1,0);
      }
  }
  
//  azimuth = 180.0;
  azim_diff = (azimuth + 9) - compassdir;
  if(cycle_counter == 50){
    Serial.print("azimuth = ");
    Serial.println(azimuth);
    Serial.print("azim_diff = ");
    Serial.println(azim_diff);
    Serial.print("Elevation = ");
    Serial.println(elevation);
    Serial.print("elev_diff = ");
    Serial.println(elev_diff);
    Serial.print("pitchangle = ");
    Serial.println(pitchAngle);
    Serial.println("-----------------------------");
  }
  if(azim_diff > 5){
    half = 5;
    turnRight(len1);
  }
  if(azim_diff < -5){
    half = -5;
    turnLeft(len1);
  }
  else{
    counter = 0;
    half = 0.0;
  }
  
  
}

void turnRight(int len){
  if(counter == 0){
    counter = 1;
    setdirection(0,0);
  }
  compassdir += 360.0/400.0 * Nratio;
  pulse(pulse_signal_M1, len); 
}
void turnLeft(int len){
  if(counter == 0){
    counter = 1;
    setdirection(0,1);
  }
  compassdir += -360.0/400.0 * Nratio;
  pulse(pulse_signal_M1, len);
}
void increaseElevation(int len){
  if(counter == 0){
    counter = 1;
    setdirection(1,0);
  }
//  Serial.println("Raising");
  micropulse(pulse_signal_M2, len);
}
void decreaseElevation(int len){
  if(counter == 0){
    counter = 1;
    setdirection(1,1);
   }
//   Serial.println("Lowering");
   micropulse(pulse_signal_M2, len);
}
void pulse(int pin, int len){
  digitalWrite(pin, LOW);
  delay(len);
  digitalWrite(pin, HIGH);
  delay(len);
  digitalWrite(pin, LOW);
}

void micropulse(int pin, int len){
  digitalWrite(pin, LOW);
  delayMicroseconds(len);
  digitalWrite(pin, HIGH);
  delayMicroseconds(len);
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

void runcorrection(){
  float diff = 0.0;
  Serial.println("Running correction :");
  diff = 180 - compassdir;
  if(index == 4){
    int i = 0; 
    float value = 0.0;
    Serial.println("===========================================");
    Serial.print("Correction data output = ");
    for(int i = 0; i <= 4; i++){
      value = correctiondata[i];
      Serial.print(value);
      if(i != 4)Serial.print(", ");
      if(i == 4)Serial.println();
    }
    Serial.println("===========================================");
    index = 0;
  }
  if(diff > 5){
    compassdir = 180.0;
    correctiondata[index] = diff;
    index++;
  }
  if(diff < -5){
    compassdir = 180.0;
    correctiondata[index] = diff;
    index++;
  }
}

void AccelerometerInit() 
{
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  Wire.write(Pitch); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  {
    currenttime = millis();
    if(0 > (endtime - currenttime)){
      break;
    }
    pitchAngle = Wire.read(); // receive a byte as characte
  }
  
  starttime = millis();
  endtime = starttime + 100;
  currenttime = 0;
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(Roll); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){
      break;
    }
    rollAngle = Wire.read(); // receive a byte as characte
  }      
} 

void readDigitalCompassCalibrationData(){
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  Wire.write(caliadd); // calibration data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from device
  while(Wire.available())    // slave may send less than requested
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){
      break;
    }
    calbyte = Wire.read(); // receive a byte as characte
  }
  
  calsys = calbyte>>6;
  calgyro = calbyte>>4;
  calgyro = calgyro&(0x03);
  calaccel = calbyte>>2;
  calaccel = calaccel&(0x03);
  calmagn = calbyte&(0x03);  
}

unsigned int highLowByteRead(uint8_t addresshigh, uint8_t addresslow){
  byte high = 0;
  byte low = 0;
  unsigned int value = 0;
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  bool problem = false;
  
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(addresshigh); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){
      problem = true;
      break;
    }
    high = Wire.read(); // receive a byte as characte
  }  
  if(problem = true)return;
  value = high<<8;

  starttime = millis();
  endtime = starttime + 100;
  currenttime = 0;
  Wire.beginTransmission(DigComp_ADDR); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(addresslow); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){
      problem = true;
      break;
    }
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
