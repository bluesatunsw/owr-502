#include <Arduino.h>
#include "BLDCMotor.h"
#include "USBSerial.h"
#include "WSerial.h"
#include "common/base_classes/FOCMotor.h"
#include "common/time_utils.h"
#include "communication/Commander.h"
#include "communication/SimpleFOCDebug.h"
#include "current_sense/LowsideCurrentSense.h"
#include "drivers/BLDCDriver6PWM.h"
#include "variant_B_G431B_ESC1.h"

#define POLE_PAIRS_G60 14
#define KV_G60 25 // 55, ADJUSTED, rot/s
#define R_G60 4.6 // Ohms

#define POLE_PAIRS_G80 21
#define KV_G80 30 // 30, ADJUSTED, rot/s
#define R_G80 1.64 // Ohms


BLDCMotor motor = BLDCMotor{POLE_PAIRS_G80, R_G80, KV_G80};
BLDCDriver6PWM driver = BLDCDriver6PWM{A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL};
LowsideCurrentSense currentSense = LowsideCurrentSense{0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT};

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
  currentSense.linkDriver(&driver);
  motor.linkDriver(&driver);

  // Motor setup
  motor.useMonitoring(Serial);
  motor.controller = MotionControlType::velocity_openloop;
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE; 
  // motor.current_limit = 60;
  // motor.velocity_limit = 50;
  motor.init();

  // Current sense
  currentSense.init();
  motor.linkCurrentSense(&currentSense); 

  // Initialize FOC
  motor.initFOC();
  motor.monitor_downsample = 1;

  // subscribe motor to the commands
  commander.add('M',doMotor,"motor");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();

  commander.run();

  PhaseCurrent_s currents = currentSense.getPhaseCurrents();
  float current_magnitude = currentSense.getDCCurrent();
}