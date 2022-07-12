
/*System Mode: 
 * 0 = Yes/No For Launch
 * 1 = Controlled/Stabilized Ascent
 * 2 = Coasting After burnout
 * 3 = Parachute Deployment
 * 4 = Parachute Assisted Descent
 * 5 = Abort Sequence
 */

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "BMI088.h"
#include <BMP280_DEV.h>                           

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

float temperature, pressure, altitude;            
BMP280_DEV bmp280;     

double PIDX, PIDY, errorX, errorY,prev_errX , prev_errY, pwmX, pwmY, previouslog, OreX, OreY, OreZ;
double PrevGyroX, PrevGyroY, PrevGyroZ, IntGyroX, IntGyroY, RADGyroX, RADGyroY, RADGyroZ, RawGyZ, DiffGyroX, DiffGyroY, DiffGyroZ, m1, m2, m3;
double m4, m5, m6, m7, m8, m9, PrevGyX, PrevGyY, PrevGyZ, RawGyX, RawGyY, GyAngleX, GyAngleY, GyAngleZ, GyRawX, GyRawY, GyRawZ;

//Upright Angle of the Rocket
int Stable_AngleX = 0;//servoY
int Stable_AngleY = 0;//servoX

//Servo Offsets for centering the TVC Mount 
int servoY_offs = 65;
int servoX_offs = 125;

//Position of servos through the startup function
int servoXstart = servoX_offs;
int servoYstart = servoY_offs;

//The amount the servo moves/actuates in the starting function
int servo_movementamount = 20;

//Ratio Between Servo & TVC Mount
float servoX_ratio = 6;
float servoY_ratio = 6;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double Ax;
double Ay;

int LED_RED = 2;    // LED connected to digital pin 9
int LED_BLUE = 5;    // LED connected to digital pin 9
int LED_GREEN = 6;    // LED connected to digital pin 9

int BUZZ = 21;
int teensyLED = 13;

Servo servoX;
Servo servoY;
Servo para;

double dt, currentTime, previousTime;

//SD CARD CS
const int chipSelect = BUILTIN_SDCARD;

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;


//PID Gains
double kp = 1.9;//0.09
double ki = 0.01;//0.03
double kd = 0.0275;//0.0275

int mode;

//Launch Site Altitude in Meters(ASL)
float launchalt;
int a=1;

//Timer settings for log in Hz
unsigned long prevLog = 0;        
const long logInterval = 250;  

void setup(){
 
  Serial.begin(9600);
  Wire.begin();
  servoX.attach(29);
  servoY.attach(28);// was 30 because of 4.1 its 28 
  para.attach(7);
  bmp280.begin(BMP280_I2C_ALT_ADDR);              
  bmp280.startNormalConversion();
  
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
 
  pinMode(teensyLED, OUTPUT);
  starting_sequence();
  sdcardstart();
  launchchecks();
  a=1;
}
void loop() {
  //Defining Time Variables
  
  if (a==1)
  {
   if (bmp280.getMeasurements(temperature, pressure, altitude))
  {
    launchalt=altitude;
    Serial.println(launchalt);
    a++;
  }
  }      
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  launchdetection();
  datadumpinSDCARD();
  burnoutdetction();
  previousTime = currentTime;  
  
}

void orientation_computation() {
  //Change Variable so its easier to refrence later on
  GyRawX = (gyro.getGyroY_rads());
  GyRawY = (gyro.getGyroZ_rads());
  GyRawZ = (gyro.getGyroX_rads());

  //Integrate over time to get Local Orientation
  GyAngleX += GyRawX * dt;
  GyAngleY += GyRawY * dt;
  GyAngleZ += GyRawZ * dt;

  PrevGyroX = RADGyroX;
  PrevGyroY = RADGyroY;
  PrevGyroZ = RADGyroZ;
  
  RADGyroX = GyAngleX;
  RADGyroY = GyAngleY;
  RADGyroZ = GyAngleZ;
  
  DiffGyroX = (RADGyroX - PrevGyroX);
  DiffGyroY = (RADGyroY - PrevGyroY);
  DiffGyroZ = (RADGyroZ - PrevGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;
  
 //X Matrices
  m1 = (cos(DiffGyroZ) * cos(DiffGyroY));
  m2 = (((sin(DiffGyroZ) * -1) * cos(DiffGyroX) + (cos(DiffGyroZ)) * sin(DiffGyroY) * sin(DiffGyroX)));
  m3 = ((sin(DiffGyroZ) * sin(DiffGyroX) + (cos(DiffGyroZ)) * sin(DiffGyroY) * cos(DiffGyroX)));
  
 //Y Matrices
  m4 = sin(DiffGyroZ) * cos(DiffGyroY);
  m5 = ((cos(DiffGyroZ) * cos(DiffGyroX) + (sin(DiffGyroZ)) * sin(DiffGyroY) * sin(DiffGyroX)));
  m6 = (((cos(DiffGyroZ) * -1) * sin(DiffGyroX) + (sin(DiffGyroZ)) * sin(DiffGyroY) * cos(DiffGyroX)));

 //Z Matrices
  m7 = (sin(DiffGyroY)) * -1;
  m8 = cos(DiffGyroY) * sin(DiffGyroX);
  m9 = cos(DiffGyroY) * cos(DiffGyroX);

 OrientationX = ((OreX * m1)) + ((OreY * m2)) + ((OreZ * m3));
 OrientationY = ((OreX * m4)) + ((OreY * m5)) + ((OreZ * m6));
 OrientationZ = ((OreX * m7)) + ((OreY * m8)) + ((OreZ * m9));

Ax = asin(OrientationX) * (-180 / PI);//orienX actually
Ay = asin(OrientationY) * (180 / PI);//orienY actually

pidcontroller();

}

void pidcontroller () {
prev_errX = errorX;
prev_errY = errorY; 

errorX = Ax - Stable_AngleX;
errorY = Ay - Stable_AngleY;

//Defining "P" 
pidX_p = kp * errorX;
pidY_p = kp * errorY;

//Defining "D"
pidX_d = kd*((errorX - prev_errX)/dt);
pidY_d = kd*((errorY - prev_errY)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);
pidY_i = ki * (pidY_i + errorY * dt);

//Adding it all up
PIDX = pidX_p + pidX_i + pidX_d;
PIDY = pidY_p + pidY_i + pidY_d;
 
pwmY = ((PIDY * servoY_ratio) + servoY_offs);
pwmX = ((PIDX * servoX_ratio) + servoX_offs); 

//Servo outputs
servoX.write(pwmX);
servoY.write(pwmY);

}

void starting_sequence() {
  tone(BUZZ, 1050, 800);
  para.write(0);
  digitalWrite(LED_BLUE, HIGH);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
  servoX.write(servoXstart + servo_movementamount);
  delay(200);
  servoY.write(servoYstart + servo_movementamount);
  digitalWrite(teensyLED, HIGH);
  delay(200);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(200);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  tone(BUZZ, 1100, 300);
  servoX.write(servoXstart - servo_movementamount);
  delay(200);
  tone(BUZZ, 1150, 250);
  servoY.write(servoYstart - servo_movementamount);
  delay(200);
  tone(BUZZ, 1200, 200);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(500);
  
  
  
 }
 
void launchdetection() {  
  accel.readSensor();
  if (mode == 0 && accel.getAccelX_mss() > 11) {
  mode++;
  }
  if (mode == 1) {
  gyro.readSensor();
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  abortsequence();
  sdcardwrite();
  orientation_computation();
  
 }
}
void datadumpinSDCARD () {
  
  if (mode >= 1) { 
    if (bmp280.getMeasurements(temperature, pressure, altitude))  {  
      
    }
  }
 
Serial.print(" System Mode:  ");
Serial.println(mode);

}

void sdcardstart () { 
if (!SD.begin(chipSelect)) {
    Serial.println("SDcard loading failed");
   
    return;
  }
  Serial.println("SDcard is Successfully Loaded.");
  
  
  
}

void sdcardwrite () {
String datastring = "";

 datastring += "Pitch,"; 
 datastring += String(Ax);
 datastring += ",";

 datastring += "Yaw,";
 datastring += String(Ay);
 datastring += ",";
 
 datastring += "System_State,";
 datastring += String(mode);
 datastring += ",";
 
 datastring += "Z_Axis_Accel,";
 datastring += String(accel.getAccelX_mss());
 datastring += ",";

 datastring += "Time,";
 datastring += String(millis());
 datastring += ",";

 datastring += "Altitude,";
 datastring += String(altitude);
 datastring += ",";
 
 datastring += "Launch Altitude,";
 datastring += String(launchalt);
 datastring += ",";

 datastring += "PIDX,";
 datastring += String(PIDX);
 datastring += ",";

 datastring += "PIDY,";
 datastring += String(PIDY);
 datastring += ",";



 
  File dataFile = SD.open("flightlog1.txt", FILE_WRITE);
  
  if (dataFile) {
    if(currentTime - prevLog > logInterval){
    prevLog = currentTime;
    dataFile.println(datastring);
    }
    dataFile.close();
   
  }
  }

void burnoutdetction () { 
if (mode == 1 && accel.getAccelX_mss() <= 2) {
  mode++;
  digitalWrite(teensyLED, LOW);
  digitalWrite(LED_RED, HIGH);
  tone(BUZZ, 1200, 200);
  Serial.println("Burnout of engine Detected");
}

if (mode == 1 || mode == 2) {
  Serial.print(launchalt,"  ");
  Serial.println((altitude - launchalt));
  
}

if (mode == 2 && (altitude - launchalt) <= 5) {
  mode++;
}
  
if(mode == 3) {
  para.write(100);
  digitalWrite(LED_RED, LOW);
  Serial.println("ParaChute is Deployed");
  mode++;
}

if (mode == 4) {

}
}

void launchchecks() {
  mode == 0;
  delay(750);
  Serial.println("Flight Computer is Ready");
     int status;
  status = accel.begin();
  if (status < 0) {
    Serial.println("BMI088 Accelometer Initialization Error Occurred");
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("BMI088 Gyroscope Initialization Error Occurred"); 
    while (1) {}
  }
 

  float totalAccelVec = sqrt(sq(accel.getAccelZ_mss()) + sq(accel.getAccelY_mss()) + sq(accel.getAccelX_mss()));
  Ax = -asin(accel.getAccelZ_mss() / totalAccelVec);
  Ay = asin(accel.getAccelY_mss() / totalAccelVec);//for giving the initial orientation indication


  delay(500);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, LOW);
  
}
void abortsequence () {
  if (mode == 1 && (Ax > 45 || Ax < -45) || (Ay > 45 || Ay < -45)){ 
    Serial.println("Abort Detected");
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(teensyLED, LOW);
    tone(BUZZ, 1200, 400);
    mode++;
    mode++;
    mode++;
    mode++; //Immediate Shift to State 5
    Serial.print("Abort Sequence");
    para.write(100);
  }
}
