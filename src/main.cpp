#include <Arduino.h>
#include <Wire.h>
#include <LPS.h>
#include <Servo.h>


//Control variables
float recording_clock_speed=500.0; // Interval in ms between recordings of flight data
float prediction_time=0.25; // How far ahead the vehicle looks to determine necessary attitude corrections

//Other variables
float deltaPitch, deltaYaw, deltaRoll, pitch, yaw, roll, prevDeltaPitch, prevDeltaYaw, prevDeltaRoll, errorPitch, errorYaw, errorRoll;
float deltaTime, currentTime, previousTime, modTime;
float correctionPitch, correctionYaw, correctionRoll, targetPitch, targetYaw, targetRoll;
int elevonLeft, elevonRight, rudder;
float pressure;
int servopos;
int c=0;
#define LED_ON HIGH
#define LED_OFF LOW
Servo leftServo;
Servo rightServo;
Servo rudderServo;
LPS ps;

void setup() {
  // Control vriables
  delay(100);
  servopos=70;
  pinMode(2, OUTPUT);
  prevDeltaPitch=0;
  prevDeltaYaw=0;
  prevDeltaRoll=0;
  pitch=0;
  yaw=0;
  roll=0;
  targetPitch=0;
  targetYaw=0;
  targetRoll=0;
  leftServo.attach(6);
  rightServo.attach(7);
  rudderServo.attach(8);
  modTime=millis();
  Wire.begin(); // Initialize I2C communication
  Wire.setTimeout(100);
  Serial.begin(19200);               // Initialize comunication
  if (!ps.init())
  {
    delay(1);
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();
  Wire.beginTransmission(0x68);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); 
  delay(50);
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x02);
  Wire.endTransmission(true);
  delay(20); 
  Serial.print(1);
  while (c < 200) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    pitch = Wire.read() << 8 | Wire.read();
    yaw = Wire.read() << 8 | Wire.read();
    roll = Wire.read() << 8 | Wire.read();
    // Sum all readings
    errorPitch = errorPitch + (pitch/65.5);
    errorYaw = errorYaw + (yaw/65.5);
    errorRoll = errorRoll + (roll/65.5);
    c++;
  }
  Serial.print(2);
  //Divide the sum by 200 to get the error value
  errorPitch = errorPitch / 200;
  errorYaw = errorYaw / 200;
  errorRoll = errorRoll / 200;
  pitch=0;
  yaw=0;
  roll=0;
  Serial.print(3);
}

void loop() {
  // Example: Request data from I2C device
  previousTime=currentTime;        // Previous time is stored before the actual time read
  currentTime=millis();            // Current time actual time read
  deltaTime=(currentTime-previousTime)/1000.0;
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Gyro data first register address 0x43
  //Wire.endTransmission(false);
  delay(10);
  //Marker
  Wire.requestFrom(0x68, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  deltaPitch = ((Wire.read() << 8 | Wire.read()) / 65.5)-errorPitch;
  deltaYaw=((Wire.read() << 8 | Wire.read()) / 65.5)-errorYaw;
  deltaRoll=((Wire.read() << 8 | Wire.read()) /65.5)-errorRoll;
  Wire.endTransmission(true);
  
  // Update attitude data by trapezoidal integration of the gyro outputs
  pitch=pitch+(deltaPitch+prevDeltaPitch)/2.0*deltaTime;
  yaw=yaw+(deltaYaw+prevDeltaYaw)/2.0*deltaTime;
  roll=roll+(deltaRoll+prevDeltaRoll)/2.0*deltaTime;
  // Determine corrections based on predicted attitude in 0.25 seconds
  correctionPitch=3*(targetPitch-pitch-(deltaPitch*prediction_time));
  correctionYaw=(targetYaw-yaw-(deltaYaw*prediction_time));
  correctionRoll=3*(targetRoll-roll-(deltaRoll*prediction_time));

  //Constrain correction values
  if (correctionPitch>50.0) {correctionPitch=50.0;} else if (correctionPitch<-50.0){correctionPitch=-50.0;}
  if (correctionYaw>50.0) {correctionYaw=50.0;} else if (correctionYaw<-50.0){correctionYaw=-50.0;}
  if (correctionRoll>50.0) {correctionRoll=50.0;} else if (correctionRoll<-50.0){correctionRoll=-50.0;}

  //Mixer
  elevonLeft=90+int((((correctionPitch*-1.0)+correctionRoll)/2.0)/5*9);
  elevonRight=90+int((((correctionPitch*-1.0)-correctionRoll)/2.0)/5*9*-1);
  rudder=90+int(correctionYaw/5*-9);

  leftServo.write(elevonLeft);
  rightServo.write(elevonRight);
  rudderServo.write(rudder);
  
  if (currentTime-modTime>recording_clock_speed) {
    pressure = ps.readPressureMillibars();
    Serial.print('=');
    Serial.print(int(correctionPitch));
    Serial.print(' ');
    Serial.print(int(correctionRoll));
    Serial.print(" ");
    Serial.print(int(correctionYaw));
    if (digitalRead(2) == LED_ON) {
      digitalWrite(2, LED_OFF);
      servopos=110;
    }else{
      digitalWrite(2, LED_ON);
      servopos=70;
    }
    modTime=currentTime;
  }
}

// put function definitions here:
