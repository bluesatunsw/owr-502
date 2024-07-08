#include <Arduino.h>
#include "BLDCMotor.h"
#include "USBSerial.h"
#include "WSerial.h"
#include "common/base_classes/FOCMotor.h"
#include "communication/Commander.h"
#include "communication/SimpleFOCDebug.h"
#include "drivers/BLDCDriver6PWM.h"
#include "variant_B_G431B_ESC1.h"
#include "wiring_analog.h"

#define POLE_PAIRS_G60 14
#define KV_G60 55 
#define R_G60 1.1 // Ohms
#define L_G60 0.9 // mH

#define POLE_PAIRS_G80 21
#define KV_G80 30
#define R_G80 1.64 // Ohms
#define L_G80 1.1 // mH


BLDCMotor motor = BLDCMotor{POLE_PAIRS_G60, R_G60, KV_G60, L_G60};
BLDCDriver6PWM driver = BLDCDriver6PWM{A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL};


// Commander commander = Commander(Serial);
// void doMotor(char* cmd){commander.motor(&motor, cmd);}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.init();


  // Motor setup
  motor.useMonitoring(Serial);
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;
  // TODO: Fix
  motor.velocity_limit = 20;
  motor.monitor_variables = _MON_TARGET | _MON_VEL; 
  motor.init();
  // motor.initFOC();


  // subscribe motor to the commands
  // commander.add('M',doMotor,"motor");
}

void loop() {
  // commander.run();

  motor.loopFOC();
  motor.move(20.0);
  motor.monitor();
}