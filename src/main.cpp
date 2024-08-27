#include <Arduino.h>
#include <communication/Commander.h>
#include <current_sense/LowsideCurrentSense.h>
#include <drivers/BLDCDriver6PWM.h>
#include <HFIBLDCMotor.h>
#include <SimpleCAN.h>
#include "Constants.h"


HFIBLDCMotor motor{config::currentMotor.PP, config::currentMotor.R, config::currentMotor.KV};
BLDCDriver6PWM driver{A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL};
LowsideCurrentSense currentSense{drvconstants::kShuntOhms, drvconstants::kADCGain, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT};

float _temp;
PhaseCurrent_s _currents;
float _dccurrent;
float _velocity;
float _position;
uint32_t time_prev = 0;

Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}
void process_hfi() {motor.process_hfi();} // override global weak symbol, runs in current sense ISR!

constexpr float TempADC(float ADCVoltage) { // convert raw ADC voltage to UNCALIBRATED temp (C)
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

  // Initialize comms
  motor.monitor_downsample = 1000;
  char motor_id = 'M';
  commander.add(motor_id,doMotor,"motor");
  // configuring the monitoring to be well parsed by the webcontroller
  motor.monitor_start_char = motor_id; 
  motor.monitor_end_char = motor_id;


  // Motor setup
  motor.useMonitoring(Serial);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE; 
  motor.hfi_v = drvconstants::kHFI_V;
  motor.hfi_gain1 = drvconstants::kHFI_Gain_Pos;
  motor.hfi_gain2 = drvconstants::kHFI_Gain_Vel;
  motor.voltage_sensor_align = drvconstants::kCurrentSenseAlignEffort_V;
  motor.error_saturation_limit = drvconstants::kHFI_AngDeltaDiscardThresh;
  motor.Ts = 1.0/drvconstants::kPWMFreqHz;
  motor.Lq = config::currentMotor.Lq; 
  motor.Ld = config::currentMotor.Ld; // Lowest one 
  

  motor.LPF_velocity.Tf = 1/(25*_2PI); 
  motor.LPF_angle.Tf =  1/(25*_2PI);
  motor.LPF_current_d.Tf = 1/(2000*_2PI);
  motor.LPF_current_q.Tf = 1/(2000*_2PI);
  motor.P_angle.P = 0.3f;
  motor.P_angle.I = 0.1f;
  motor.P_angle.D = 0.005f;
  motor.PID_velocity.P = 0.01f;
  motor.PID_velocity.I = 0.01f;
  motor.PID_velocity.D = 0.0f;

  // "Driver" set up
  driver.pwm_frequency = drvconstants::kPWMFreqHz;
  driver.voltage_power_supply = drvconstants::kNominalBusVoltage;
  motor.voltage_limit = driver.voltage_power_supply;
  driver.init();
  currentSense.linkDriver(&driver);
  motor.linkDriver(&driver);
  currentSense.init();
    
  // Current sense
  motor.linkCurrentSense(&currentSense); 

  motor.init();
  motor.initFOC();

  motor.hfi_on = true;
  motor.sensor_direction = Direction::CCW;
  motor.current_setpoint.d = 0.00f;
  motor.disable();

  _delay(1000);
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