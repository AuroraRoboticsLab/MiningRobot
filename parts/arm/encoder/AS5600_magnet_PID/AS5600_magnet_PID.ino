/*
 Talk to the AS5600 magnetic angle sensor, over I2C
 https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf


 Hookup:
  Arduino Uno pins A4 (SDA) & A5 (SCL), power (3.3VDC), ground.
*/
#include <Wire.h>
#include <Servo.h>

int servopin = 9; // servo white wire
Servo servolib;

int outpin = A0; // chip's analog output pin

const int I2Caddr = 0x36;


int target = 750;
int arm_angle = 1000; // servo command to arm ESC (also zero point of PID)


void setup() {
  Serial.begin(115200);
  
  Serial.println("AS5600 Magnet to servo angle: arming ESC");
  servolib.attach(servopin);
  servolib.writeMicroseconds(arm_angle);
  delay(3000); // arm esc
  Serial.println("ESC armed, starting PID!");


  pinMode(outpin,INPUT);

  Wire.begin();
  Wire.setWireTimeout(10000,true);
}

// Read a 16-bit big endian values from this I2C register.
//   If print==1, prints the bytes to the serial port. 
unsigned int readHex(const char *what,int reg, int print=1)
{
  if (print) Serial.print(what);

  Wire.beginTransmission(I2Caddr);
  Wire.write(reg); 
  Wire.endTransmission();

  unsigned char data[2] = {0,0};
  int n = Wire.requestFrom(I2Caddr,2);
  for (int i=0;i<n;i++) data[i] = Wire.read();
  if (print) for (int i=0;i<n;i++) {
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
  return (data[0]<<8)|(data[1]);
}

int last_error = 0; // for rate term
float smooth_rate = 0; // smoothed version, less noisy
int total_error = 0; // for integral term

void loop() {
  while (Serial.available()>0) {
    char c= Serial.read();
    if (c=='a') arm_angle=Serial.parseFloat();
    if (c=='t') target=Serial.parseFloat();
  }

  //int dn = analogRead(outpin); // potentiometer input
  //int us = map(dn, 0,1023, 900,2100);
  //servolib.writeMicroseconds(us);

  unsigned int ang=readHex("ANG ",0x0E,0);
  //unsigned int agc=readHex("AGC ",0x0A,1);
  unsigned int mag=readHex(" MAG ",0x1B,0);

  Serial.print(ang);
  Serial.print("\t");
  Serial.print(mag);
  Serial.print("\t");

  int error = target - ang; // in raw 4096th of rotation
  
  int cur_rate = error - last_error;
  last_error = error;
  smooth_rate = 0.75*smooth_rate + 0.25*cur_rate;

  if (error>30 || error<-30) { // big error, history corrupted
    total_error=0; 
  } else { // small error, add to history
    total_error += error;

    // Limit total windup to avoid oscillations
    int windup = 200; 
    if (total_error>+windup) total_error=+windup;
    if (total_error<-windup) total_error=-windup;
  }

  float Kp = 1.5; // microseconds of servo command per angle error
  float Kd = 10; // microseconds of command per angle per tick
  float Ki = 0.1; // correct total error
  float command = Kp * error + Kd*smooth_rate + Ki*total_error; // zero centered, units microseconds

  // Set power limit to 'tepid'
  int power=200;
  if (command>power) command=power;
  if (command<-power) command=-power;

  // Convert command to microseconds
  command += arm_angle; // add zero point for servo

  // Limit microseconds to plausible values
  if (command<800) command=800;
  if (command>2200) command=2200;
  Serial.print((int)target);
  Serial.print("\t");
  Serial.print((int)error);
  Serial.print("\t");
  Serial.print((int)smooth_rate);
  Serial.print("\t");
  Serial.print((int)total_error);
  Serial.print("\t");
  Serial.print((int)command);
  Serial.print("\t");
  servolib.writeMicroseconds((int)command);

  //unsigned int v=readHex(" AGC ",0x1A);
  Serial.println();
  delay(10); //<- run control loop at 100 Hz
}
