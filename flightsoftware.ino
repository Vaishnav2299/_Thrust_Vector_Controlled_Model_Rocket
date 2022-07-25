
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
double Ax, Ay;
double OrientationZ, OrientationY, OrientationX;

OrientationZ = 1;
OrientationY = 0;
OrientationX = 0;

//Angle of the Rocket on the launch pad which is pointing upwards
int Stable_AngleY = 0;//servoY
int Stable_AngleX = 0;//servoX

//Servo Offsets for centering the TVC Mount 
int servoY_offs = 65;
int servoX_offs = 125;

//Position of servos in the StartupSequence 
int servoYstart = servoY_offs;
int servoXstart = servoX_offs;

//The amount the servo moves/actuates in the starting function
int servo_movementamount = 12;

//Ratio Between Servo & TVC Mount
float servoX_ratio = 5;
float servoY_ratio = 5;


int BUZZ = 21;
int LED_GREEN = 6;
int LED_BLUE = 5;
int LED_RED = 2;        

Servo servoY;
Servo servoX;
Servo para;

double dt, currentTime, previousTime;

//SD CARD CS
const int chipSelect = BUILTIN_SDCARD;

float pidY_p, pidY_i, pidY_d;
float pidX_p, pidX_i, pidX_d;

//PID Gains
double ki = 0.01;
double kd= 0.0275;
double kp = 1.9;
 
// P-Constants
pidY_p = 0;
pidX_p = 0;

// I-Constants
pidY_i = 0;
pidX_i = 0;

// D-Constants
pidY_d = 0;
pidX_d = 0;

int mode;

float launchalt;
int a=1;

//Timer settings for log in Hz
unsigned long prevLog = 0;        
const long logInterval = 250;  

void setup(){
 
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  servoY.attach(28);// was 30 because of 4.1 its 28 
  servoX.attach(29);
  para.attach(7);
  bmp280.begin(BMP280_I2C_ALT_ADDR);              
  bmp280.startNormalConversion();
  
  pinMode(teensyLED, OUTPUT);
  starting_sequence();
  sdcardstart();
  launchchecks();
  a=1;
}
void loop() {
  
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
  
  GyRawZ = (gyro.getGyroX_rads());
  GyRawX = (gyro.getGyroY_rads());
  GyRawY = (gyro.getGyroZ_rads());
  

  //Integrating with time gives us the local orientation estimates
  GyAngleZ += GyRawZ * dt; 
  GyAngleX += GyRawX * dt;
  GyAngleY += GyRawY * dt;
  
  PrevGyroZ = RADGyroZ;
  PrevGyroX = RADGyroX;
  PrevGyroY = RADGyroY;
  
  RADGyroZ = GyAngleZ;
  RADGyroX = GyAngleX;
  RADGyroY = GyAngleY;
  
  DiffGyroZ = (RADGyroZ - PrevGyroZ);
  DiffGyroX = (RADGyroX - PrevGyroX);
  DiffGyroY = (RADGyroY - PrevGyroY);
  
  OreZ = OrientationZ;
  OreX = OrientationX;
  OreY = OrientationY;
  
  
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



 OrientationZ = ((OreX * m7)) + ((OreY * m8)) + ((OreZ * m9));
 OrientationX = ((OreX * m1)) + ((OreY * m2)) + ((OreZ * m3));
 OrientationY = ((OreX * m4)) + ((OreY * m5)) + ((OreZ * m6));
 
Ay = asin(OrientationY) * (180 / PI);//orienY actually
Ax = asin(OrientationX) * (-180 / PI);//orienX actually

pidcontroller();

}

void pidcontroller () {
prev_errY = errorY; 
prev_errX = errorX;

errorY = Ay - Stable_AngleY;
errorX = Ax - Stable_AngleX;


//"P" 
pidY_p = kp * errorY;
pidX_p = kp * errorX;

// "I"
pidY_i = ki * (pidY_i + errorY * dt);
pidX_i = ki * (pidX_i + errorX * dt);

//"D"
pidY_d = kd*((errorY - prev_errY)/dt);
pidX_d = kd*((errorX - prev_errX)/dt);



//Summing up PID values
PIDY = pidY_p + pidY_i + pidY_d;
PIDX = pidX_p + pidX_i + pidX_d;

pwmY = ((PIDY * servoY_ratio) + servoY_offs);
pwmX = ((PIDX * servoX_ratio) + servoX_offs); 

 
//Outputs To be sent to the TVC Actuator/Servo  
servoY.write(pwmY);
servoX.write(pwmX);


}

void starting_sequence() {
  
  para.write(0); // Locks the parachute pay into position
  digitalWrite(LED_GREEN, HIGH);
  servoY.write(servoXstart);
  servoX.write(servoYstart);
  delay(700);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  tone(BUZZ, 1050, 250);
  servoY.write(servoXstart + servo_movementamount);
  delay(500);
  servoX.write(servoYstart + servo_movementamount);
  delay(500);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  servoY.write(servoXstart);
  delay(500);
  servoX.write(servoYstart);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
  servoY.write(servoXstart - servo_movementamount);
  tone(BUZZ, 900, 200);
  delay(200);
  servoX.write(servoYstart - servo_movementamount);
  tone(BUZZ, 1050, 250);
  delay(200);
  servoY.write(servoXstart);
  delay(500);
  tone(BUZZ, 1400, 300);
  servoX.write(servoYstart);
  delay(600);
  Serial.print("Launch sequence Complete") ;
  
  
 }
 
void launchdetection() {  
  accel.readSensor();
  if (mode == 0 && accel.getAccelX_mss() > 11) {
  mode++;
  }
  if (mode == 1) {
  gyro.readSensor();
  //Reads the gyro only after upward acceleration is detected
  digitalWrite(LED_GREEN, LOW);
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
String flightdatastr = "";

 flightdatastr += "Pitch,"; 
 flightdatastr += String(Ax);
 flightdatastr += ",";

flightdatastr += "Yaw,";
flightdatastr += String(Ay);
flightdatastr += ",";
 
flightdatastr += "System_State,";
flightdatastr += String(mode);
flightdatastr += ",";
 
flightdatastr += "Z_Axis_Accel,";
flightdatastr += String(accel.getAccelX_mss());
flightdatastr += ",";

 flightdatastr += "Time,";
 flightdatastr += String(millis());
 flightdatastr += ",";

 flightdatastr += "Altitude,";
 flightdatastr += String(altitude);
 flightdatastr += ",";
 
 flightdatastr += "Launch Altitude,";
 flightdatastr += String(launchalt);
 flightdatastr += ",";

 flightdatastr += "PIDX,";
 flightdatastr += String(PIDX);
 flightdatastr += ",";

 flightdatastr += "PIDY,";
 flightdatastr += String(PIDY);
 flightdatastr += ",";



 
  File dataLoggingFile = SD.open("flightlog1.txt", FILE_WRITE);
  
  if (dataLoggingFile) {
    if(currentTime - prevLog > logInterval){
    prevLog = currentTime;
    dataLoggingFile.println(flightdatastr);
    }
    dataLoggingFile.close();
   
  }
  }

void burnoutdetction () { 
if (mode == 1 && accel.getAccelX_mss() <= 2) {
  mode++;
  digitalWrite(LED_RED, HIGH);
  tone(BUZZ, 1300, 300);
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
 

  float totAcVec = sqrt(sq(accel.getAccelZ_mss()) + sq(accel.getAccelY_mss()) + sq(accel.getAccelX_mss()));
  Ax = -asin(accel.getAccelZ_mss() / totAcVec);
  Ay = asin(accel.getAccelY_mss() / totAcVec);  // THIS GIVES THE INTITIAL ORIENTATION ESTIMATION OF THE ROCKET


  delay(700);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
  
}
void abortsequence () {
  if (mode == 1 && (Ax > 45 || Ax < -45) || (Ay > 45 || Ay < -45)){ 
    Serial.println("Abort Sequence Detected");
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
    tone(BUZZ, 1300, 500);
    mode++;
    mode++;
    mode++;
    mode++; //Immediate Shift to State 5
    Serial.print("Abort Sequence Started");
    para.write(100);
  }
}
