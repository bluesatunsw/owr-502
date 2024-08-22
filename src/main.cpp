#include <Arduino.h>
#include "WSerial.h"
#include "common/base_classes/FOCMotor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "communication/Commander.h"
#include "communication/SimpleFOCDebug.h"
#include "current_sense/LowsideCurrentSense.h"
#include "current_sense/hardware_api.h"
#include "drivers/BLDCDriver6PWM.h"
#include "variant_B_G431B_ESC1.h"
#include <HFIBLDCMotor.h>


#define POLE_PAIRS_G60 14
#define KV_G60 25 // 25, ADJUSTED, rot/s
#define R_G60 4.6 // Ohms
#define L_G60 0.00272 // H

#define POLE_PAIRS_G80 21
#define KV_G80 30 // 30, ADJUSTED, rot/s
#define R_G80 1.64 // Ohms


HFIBLDCMotor motor{POLE_PAIRS_G60, R_G60, KV_G60, L_G60};
BLDCDriver6PWM driver{A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL};
LowsideCurrentSense currentSense{0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT};

float _temp;
PhaseCurrent_s _currents;
float _dccurrent;
float _velocity;
float _position;
uint32_t time_prev = 0;

Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}
void process_hfi() {motor.process_hfi();} // override global weak symbol, runs in current sense ISR!

static float TempADC(float ADCVoltage) { // convert raw ADC voltage to UNCALIBRATED temp (C)
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F/298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;
	
	float T = 1.0F/((log(Rt/R25)/Beta)+RoomTempI);
	T = T - 273.15;

	return T;
}

float readVBUS() { return _readADCVoltageInline(A_VBUS, currentSense.params) * 10.32; } // Volts 
float readTemp() { return TempADC(_readADCVoltageInline(A_TEMPERATURE, currentSense.params)); } // C


void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // "Driver" set up
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24;
  motor.voltage_limit = driver.voltage_power_supply;
  driver.init();
  currentSense.linkDriver(&driver);
  motor.linkDriver(&driver);


  // Motor setup
  motor.useMonitoring(Serial);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE; 
  motor.hfi_v = 4;
  // motor.Lq // Highest inductance value
  // motor.Ld // Lowest inductance value
  
  motor.LPF_velocity.Tf = 1/(25*_2PI); 
  motor.LPF_angle.Tf =  1/(25*_2PI);
  motor.LPF_current_d.Tf = 1/(2000*_2PI);
  motor.LPF_current_q.Tf = 1/(2000*_2PI);
  // motor.PID_current_q.D = 0.001;
  // motor.PID_current_q.I = 0.2;
  // motor.PID_current_q.P = 0.1;

  // motor.PID_current_d.D = 0.001;
  // motor.PID_current_d.I = 0.2;
  // motor.PID_current_d.P = 0.1;

  motor.init();


  // Current sense
  currentSense.skip_align = true;
  currentSense.init();
  motor.linkCurrentSense(&currentSense); 

	// !!! The MXLEMMING observer sensor doesn't need sensor alignment
	// motor.sensor_direction= Direction::CW;
  // motor.zero_electric_angle = 0;

  // Initialize FOC
  motor.initFOC();

  motor.hfi_on = true;
  motor.sensor_direction = Direction::CCW;
  motor.current_setpoint.d = 0.00f;


  motor.monitor_downsample = 1000;
  char motor_id = 'M';
  commander.add(motor_id,doMotor,"motor");
  // configuring the monitoring to be well parsed by the webcontroller
  motor.monitor_start_char = motor_id; 
  motor.monitor_end_char = motor_id;
  _delay(1000);

    // driver.voltage_power_supply = readVBUS();
  motor.disable();
}


void loop() {
  uint32_t time_now = micros();
  if ((time_now-time_prev) > 1000){
    motor.move();
    time_prev = time_now;
    commander.run();
  }


  // driver.voltage_power_supply = readVBUS();
  // _currents = currentSense.getPhaseCurrents();
  // _dccurrent = currentSense.getDCCurrent();

  // motor.monitor();
}