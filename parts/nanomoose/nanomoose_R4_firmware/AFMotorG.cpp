// Adafruit Motor shield library, made generic using analogWrite instead of PWM calls.
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
#include <Arduino.h>

#include "AFMotorG.h"


static uint8_t latch_state;

AFMotorController::AFMotorController(void) {
    TimerInitalized = false;
}

void AFMotorController::enable(void) {
  // setup the latch
  /*
  LATCH_DDR |= bit(LATCH);
  ENABLE_DDR |= bit(ENABLE);
  CLK_DDR |= bit(CLK);
  SER_DDR |= bit(SER);
  */
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0;

  latch_tx();  // "reset"

  //ENABLE_PORT &= ~bit(ENABLE); // enable the chip outputs!
  digitalWrite(MOTORENABLE, LOW);
}


void AFMotorController::latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~bit(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~bit(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~bit(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & bit(7-i)) {
      //SER_PORT |= bit(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~bit(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= bit(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= bit(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/

AF_DCMotor::AF_DCMotor(uint8_t num) {
  motornum = num;
  pwmpin = 0;

  MC.enable();

  switch (num) {
  case 1:
    pwmpin = 11;
    break;
  case 2:
    pwmpin = 3;
    break;
  case 3:
    pwmpin = 5;
    break;
  case 4:
    pwmpin = 6;
    break;
  default:
    pwmpin=0;
    break;
  }
  if (pwmpin != 0) {
    pinMode(pwmpin, OUTPUT);
    analogWrite(pwmpin, 0);
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= bit(a);
    latch_state &= ~bit(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~bit(a);
    latch_state |= bit(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~bit(a);     // A and B both low
    latch_state &= ~bit(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  if (pwmpin != 0) {
    analogWrite(pwmpin, speed);
  }
}


