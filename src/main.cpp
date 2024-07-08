#include <Arduino.h>
#include "BLDCMotor.h"
#include "USBSerial.h"
#include "WSerial.h"
#include "common/base_classes/FOCMotor.h"
#include "common/time_utils.h"
#include "communication/Commander.h"
#include "communication/SimpleFOCDebug.h"
#include "current_sense/InlineCurrentSense.h"
#include "drivers/BLDCDriver6PWM.h"
#include "variant_B_G431B_ESC1.h"

#define POLE_PAIRS_G60 14
#define KV_G60 80 // 55, ADJUSTED, rad/s
#define R_G60 1.1 // Ohms

#define POLE_PAIRS_G80 21
#define KV_G80 50 // 30, ADJUSTED, rad/s
#define R_G80 1.64 // Ohms


BLDCMotor motor = BLDCMotor{POLE_PAIRS_G60, R_G60, KV_G60};
BLDCDriver6PWM driver = BLDCDriver6PWM{A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL};
InlineCurrentSense currentSense = InlineCurrentSense{0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT};

Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // "Driver" set up
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 24;
  driver.init();
  motor.linkDriver(&driver);

  // Motor setup
  motor.useMonitoring(Serial);
  motor.controller = MotionControlType::velocity_openloop;
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q  | _MON_CURR_Q | _MON_VEL | _MON_ANGLE; 
  motor.init();

  // Current sense
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense); 

  // Initialize FOC
  motor.initFOC();

  // subscribe motor to the commands
  commander.add('M',doMotor,"motor");
  _delay(1000);
}

void loop() {
  motor.move();

  motor.monitor();
  commander.run();
}