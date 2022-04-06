#include <Wire.h>
#include <SparkFun_RV8803.h> //SparkFun RTC commication functions
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

//these define the length of delay between each pulse to the stepper motor(minimum value of 5 micro seconds)
#define len1 40 //mili seconds
#define lenM 15 //mili seconds
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
float Nratio = 1.7/(13.0)*1.2; //the small to large pulley ratio for calculating the current azimuthal angle

float correctiondata[] = {0.0, 0.0, 0.0, 0.0, 0.0}; //stores hall sensors correction data during operation
int index = 0; //for traking the current index of the hall sensors correction data

bool startup = true; //used to run the startup calibration function once on startup
int hall90 = 0; //stores 90 degree hall sensor DIO pin HIGH or LOW status
int hall180 = 0; //stores 180 degree hall sensor DIO pin HIGH or LOW status

//these global variables are for storing the values sent from the digital compass
unsigned int compassdirection = 0;
int8_t pitchAngle = 0;
int8_t rollAngle = 0;

//these variables store the difference values between panel position and solar positon
int elev_diff = 0;
float azim_diff = 0.0;

float azimutherror = 0.0;
float compassdir = 90.0;

//Bytes are for getting/storing the digital compass calibration values
uint8_t calbyte = 0;
uint8_t calsys = 0;
uint8_t calgyro = 0;
uint8_t calaccel = 0;
uint8_t calmagn = 0;


//setup starts the I2C and serial communication lines and checks that there 
//is a working I2C connection between Arduino and RTC
void setup() {

  Wire.begin();
  Serial.begin(57600);

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

  //output pins to the 2 stepper motor controllers
  pinMode(direction_signal_M1, OUTPUT);
  pinMode(pulse_signal_M1, OUTPUT);
  pinMode(direction_signal_M2, OUTPUT);
  pinMode(pulse_signal_M2, OUTPUT);

  //input pins to the manual motor controllers and hall sensors
  pinMode(Manual , INPUT);
  pinMode(Hall90 , INPUT);
  pinMode(Hall180 , INPUT);
  pinMode(M1_left, INPUT_PULLUP);
  pinMode(M1_right, INPUT_PULLUP);
  pinMode(M2_up, INPUT_PULLUP);
  pinMode(M2_down , INPUT_PULLUP);
}

void loop() {
  while(startup == true)calibration(); //on Arduino startup/reset the panel is calibrated
  
  if(cycle_counter == 100)cycle_counter = 0; //resets cycle counter
  
  //check both calibration hall sensors for azimuthal error correction purposes
  hall90 = digitalRead(Hall90);
  delay(1);
  hall180 = digitalRead(Hall180);
  delay(1);
  
  if(hall90 == HIGH){ 
    if(index == 4)index = 0;
    //if panel the panel is pointing at a azimuthal angle of 90 degrees then check the calculated azimuthal error 
    if(cycle_counter == 50)Serial.println("90");
    correctiondata[index] = 95 - compassdir;
    //potenialy write these error values to the eprom for storage    
  }
  if(hall180 == HIGH){
    if(index == 4)index = 0;
    //if panel the panel is pointing at a azimuthal angle of 180 degrees then check the calculated azimuthal error
    if(cycle_counter == 50)Serial.println("180");
    correctiondata[index] = 182 - compassdir;
    //potenialy write these error values to the eprom for storage  
  }

  //if manual control is actiavetd start checking the manual control buttons
  manual = digitalRead(Manual);
  delay(1);
  if(manual == LOW){
    right = digitalRead(M1_right);
    delay(1);
    left = digitalRead(M1_left);
    delay(1);
    up = digitalRead(M2_up);
    delay(1);
    down = digitalRead(M2_down);
    delay(1);
    
    if(right == LOW)turnRight(lenM);
    else{
      counter = 0; //resets the set direction cycle blocker
    }
    if(left == LOW)turnLeft(lenM);
    else counter = 0; //resets the set direction cycle blocker
  
    if(up == LOW)increaseElevation(len2);       
    else{
      counter = 0; //resets the set direction cycle blocker
    }
    if(down == LOW)decreaseElevation(len2);
    else counter = 0; //resets the set direction cycle blocker
  }

  if(manual == HIGH)autoController(); //if manual control is off run the automatic controller
  
  cycle_counter += 1;
}

//startup calibration that turns the solar panel left until one of the two angle hall sensors are triggered
void calibration(){
  hall90 = digitalRead(Hall90);
  delay(1);
  hall180 = digitalRead(Hall180);
  delay(1);

  if(hall90 == HIGH){
    Serial.println("90 Startup complete");
    compassdir = 95;//set current panel compass direction
    startup = false;
  }
  if(hall180 == HIGH){
    Serial.println("180 Startup complete");
    compassdir = 180;
    startup = false;
  }
  turnLeft(len1);
}

//gets new time data from RTC and calculates new Solar elevation and azimuthal angle
//and then request data from the digital compass
void updateTimePositionData(){
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
//    compassdirection = highLowByteRead(comp1, comp2);
    digitalCompassDataRequest();
  }
}

//this is the solar panel auto controller that will normally control the solar panel position
void autoController(){
  updateTimePositionData(); //update time data if the RTC has updated its time registers
  pitchAngle = (pitchAngle - 90) *-1; //switches the sensor pitch angle into correct elevation angle
//    elevation = 90.0; //maunal setting of the auto elevation angle
  elev_diff = pitchAngle - elevation;
//    azimuth = 180.0; //maunal setting of the auto azimuthal angle
  azim_diff = (azimuth) - compassdir;

  if (pitchAngle > 90)decreaseElevation(len2); //if solar panel elevation is over 90 degrees
  else counter = 0; //resets the set direction cycle blocker
  if (pitchAngle <= 90){
    if (elev_diff > 1)decreaseElevation(len2);
    else counter = 0; //resets the set direction cycle blocker
  }
  if (pitchAngle > 10){
    if (elev_diff < -1)increaseElevation(len2);
    else counter = 0; //resets the set direction cycle blocker
  }
  if (pitchAngle <= 10)increaseElevation(len2); //if solar panel elevation is too low
  
  if(cycle_counter == 50)printInfo();
  
//  if(90 < compassdir && compassdir < 300){
    delay(1);
    if(azim_diff > 5)turnRight(len1);
    if(azim_diff < -5)turnLeft(len1);
    else counter = 0; //resets the set direction cycle blocker
//  }
//  if(90 > compassdir)turnRight(len1); //if solar panel azimuthal angle is too low
//  if(compassdir > 300)turnLeft(len1); //if solar panel azimuthal angle is too high
}

//prints out relevant info
void printInfo(){
  Serial.print("Solar Elevation = ");
  Serial.println(elevation);
  Serial.print("Compassdir = ");
  Serial.println(compassdir);
  Serial.print("Solar Azimuth = ");
  Serial.println(azimuth);
  Serial.print("Elevation_difference = ");
  Serial.println(elev_diff);
  Serial.print("Azimuthal_difference = ");
  Serial.println(azim_diff);
  Serial.println("-----------------------------");
}

void turnRight(int len){
  float addition = + 360.0/400.0 * Nratio;
  if(counter == 0){
    counter = 1; //locks the set direction cycle blocker
    setdirection(0,0);
  }
  compassdir = compassdir + 360.0/400.0 * Nratio;
  pulse(pulse_signal_M1, len); //pulse the stepper motor
}
void turnLeft(int len){
  if(counter == 0){
    counter = 1; //locks the set direction cycle blocker
    setdirection(0,1);
  }
  compassdir = compassdir + -360.0/400.0 * Nratio;
  pulse(pulse_signal_M1, len); //pulse the stepper motor
}
void increaseElevation(int len){
  if(counter == 0){
    counter = 1; //locks the set direction cycle blocker
    setdirection(1,0);
  }
  micropulse(pulse_signal_M2, len); //pulse the stepper motor
}
void decreaseElevation(int len){
  if(counter == 0){
    counter = 1; //locks the set direction cycle blocker
    setdirection(1,1);
   }
   micropulse(pulse_signal_M2, len); //pulse the stepper motor
}

void pulse(int pin, int len){
  digitalWrite(pin, LOW);
  delay(len); //milisecond delay
  digitalWrite(pin, HIGH);
  delay(len); //milisecond delay
  digitalWrite(pin, LOW);
}
void micropulse(int pin, int len){
  digitalWrite(pin, LOW);
  delayMicroseconds(len); //microsecond delay
  digitalWrite(pin, HIGH);
  delayMicroseconds(len); //microsecond delay
  digitalWrite(pin, LOW);
}

//sets the stepper motor direction of rotation
void setdirection(int output, int value){
  delayMicroseconds(20); //allows stepper motor controller to register direction
  if(output == 0){ //for azimuthal motor
    if(value == 1) digitalWrite(direction_signal_M1, HIGH);
    if(value == 0) digitalWrite(direction_signal_M1, LOW);
  }
  if(output == 1){ //for elevation motor
    if(value == 1) digitalWrite(direction_signal_M2, HIGH);
    if(value == 0) digitalWrite(direction_signal_M2, LOW);
  }
  delayMicroseconds(20); //allows stepper motor controller to register direction
}

//if correction data needs to be printed out
void printCorrectionData(){
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
  delay(10000); //10 second delay for reading of  correction data
}

//requestes the pitch/roll angle data from the digital compass
void digitalCompassDataRequest(){
  //these longs are used to time out the I2C waiting for data from the digital compass
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  
  Wire.beginTransmission(DigComp_ADDR); // address of the digital compass 
  Wire.write(Pitch); // pitchAngle data address
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from digital compass 
  while(Wire.available())    // wait for digital compass data
  {
    currenttime = millis();
    if(0 > (endtime - currenttime)){ //if digital compass I2C communication has errored 
      break;
    }
    pitchAngle = Wire.read(); // receive a byte as an integer
    if(pitchAngle < 0)pitchAngle *= -1;
  }
  //this is not used in current digital compass configuration
//  starttime = millis();
//  endtime = starttime + 100;
//  currenttime = 0;
//  Wire.beginTransmission(DigComp_ADDR); // address of the digital compass  
//  Wire.write(Roll); // rollAngle data address
//  Wire.endTransmission(); 
//  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from digital compass 
//  while(Wire.available())    // wait for digital compass data
//  { 
//    currenttime = millis();
//    if(0 > (endtime - currenttime)){ //if digital compass I2C communication has errored 
//      break;
//    }
//    rollAngle = Wire.read(); // receive a byte as an integer
//    if(rollAngle < 0)rollAngle *= -1;
//  }      
} 

//requests the calibration data from the digital compass
void readDigitalCompassCalibrationData(){
  //these longs are used to time out the I2C waiting for data from the digital compass
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  
  Wire.beginTransmission(DigComp_ADDR); // address of the digital compass 
  Wire.write(caliadd); // calibration data address
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from digital compass 
  while(Wire.available())    // wait for digital compass data
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){ //if digital compass I2C communication has errored
      break;
    }
    calbyte = Wire.read(); // receive a byte as unsigned integer
  }

  //these split the calibration Byte into the 4 pieces its composed of
  calsys = calbyte>>6;
  calgyro = calbyte>>4;
  calgyro = calgyro&(0x03);
  calaccel = calbyte>>2;
  calaccel = calaccel&(0x03);
  calmagn = calbyte&(0x03);  
}

//this function is used to read 2 Bytes (High, Low) from the digital compass and merger them into a signal unsigned integer
unsigned int highLowByteRead(uint8_t addresshigh, uint8_t addresslow){
  byte high = 0;
  byte low = 0;
  unsigned int value = 0;
  //these longs are used to time out the I2C waiting for data from the digital compass
  unsigned long starttime = millis();
  unsigned long endtime = starttime + 100;
  unsigned long currenttime = 0;
  bool problem = false; //used to end function early
  
  Wire.beginTransmission(DigComp_ADDR); // address of the digital compass  
  // reset the accelerometer 
  Wire.write(addresshigh); // High byte address
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from digital compass 
  while(Wire.available())    // wait for digital compass data
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){ //if digital compass I2C communication has errored
      problem = true;
      break;
    }
    high = Wire.read(); // receive a byte as a byte
  }  
  if(problem = true)return;
  value = high<<8;

  starttime = millis();
  endtime = starttime + 100;
  currenttime = 0;
  Wire.beginTransmission(DigComp_ADDR); // address of the digital compass 
  // reset the accelerometer 
  Wire.write(addresslow); // Low byte address
  Wire.endTransmission(); 
  Wire.requestFrom(DigComp_ADDR,1);    // request 8 bytes from digital compass
  while(Wire.available())    // wait for digital compass data
  { 
    currenttime = millis();
    if(0 > (endtime - currenttime)){ //if digital compass I2C communication has errored
      problem = true;
      break;
    }
    low = Wire.read(); // receive a byte as a byte
  }
  if(problem = true)return;
  value += low;
  return value; //return merged bytes as an unsigned integer
}

//calculates the Solar elevation and azimuthal angle using current time data
void solarzenithelevation(double year, double month, double day, double hour, double minute, double Tgmt){
  double lambdaO = -123.12722630891494 * PI/180; //location longitude in radians
  double psiO = 49.17491793381123 * PI/180; //location latitude in radians
  double psiS = sundeclination(year, month, day, hour+7.0, minute); //current solar declination
  double Emin = equationoftime(year, month, day, hour, minute); //equation of time
  double lambdaS = -15*(Tgmt - 12 + Emin/60.0) * PI/180.0;
  double Sx = cos(psiS)* sin(lambdaS - lambdaO);
  double Sy = cos(psiO) * sin(psiS) - sin(psiO) * cos(psiS) * cos(lambdaS - lambdaO);
  double Sz = sin(psiO) * sin(psiS) + cos(psiO) * cos(psiS) * cos(lambdaS - lambdaO);
  double Z = asin(Sz); //solar elevation angle
  double ys = atan2(Sx,Sy); //solar azimuthal angle

  elevation = Z * 180/PI;
  azimuth = ys * 180/PI;
  if(azimuth < 0.0){ //this adjusts the (-180) - 180 azimuthal output of atan2 to 0 - 360 degrees
    azimuth += 360;
  }
}

//this function calculates the Solar declination for the current date and time
double sundeclination(double year, double month, double day, double hour, double minute){
  double N = Ncalc(year, month, day, hour, minute);
  double declination = asin(sin(-23.44*DEG_TO_RAD) * cos(2*PI/365.24*(N+10) + 2*0.0167 * sin(2*PI/365.24*(N-2))));
  return declination;
}

//this function calculates the equation of times value for the current date and time
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

//this function calculates the number of days since january 1 to the current date
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
