// stepper_controller.ino
//
// Firmware for the Mellow FLY-DP5 that responds to motor controller CAN frames
// and drives the stepper motors accordingly.

#include <stm32f0xx_hal_can.h>
#include "hal_conf_extra.h"

#define BLINK_TIME_MS       300
#define FAST_BLINK_TIME_MS  150
#define SLOW_BLINK_TIME_MS  600

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan);
void blinkLED(int times, int blinkDur);

// stand-ins for the actual CAN IDs of the stepper motors
enum StepperId {
  STEPPER_A = 32, STEPPER_B, STEPPER_C
};

HAL_StatusTypeDef halStatus = {};
CAN_HandleTypeDef hcan = {};

///////////////////////////
// CAN-related functions //
///////////////////////////

struct CANFrame {
  // TODO: add fields
  uint32_t id {};
  uint32_t data {};
};

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
  __HAL_RCC_CAN1_CLK_ENABLE();
}

void initialiseCAN() {
  // TODO: load up hcan with a bunch of config parameters
  halStatus = HAL_CAN_Init(&hcan);
  // bedugging!
  if (halStatus == HAL_OK) {
    blinkLED(3, FAST_BLINK_TIME_MS);
  } else if (halStatus == HAL_ERROR) {
    blinkLED(1, FAST_BLINK_TIME_MS);
    blinkLED(2, SLOW_BLINK_TIME_MS);
    blinkLED(1, FAST_BLINK_TIME_MS);
  } else if (halStatus == HAL_BUSY) {
    blinkLED(1, SLOW_BLINK_TIME_MS);
    blinkLED(2, FAST_BLINK_TIME_MS);
    blinkLED(1, SLOW_BLINK_TIME_MS);
  } else if (halStatus == HAL_TIMEOUT) {
    blinkLED(6, FAST_BLINK_TIME_MS);
  } else {
    blinkLED(4, BLINK_TIME_MS);
  }

  // HAL_CAN_ConfigFilter()?
  // hal_status = HAL_CAN_Start(hcan);
}

// blocks until a CAN frame becomes available
CANFrame getFrame() {
  CANFrame frame;
  // generate a dummy frame
  // TODO: actually read from CAN!
  frame.id = 32;
  frame.data = 5;
  return frame;
}

/////////////////////
// Other functions //
/////////////////////

void initialiseBlinker(void) {
  pinMode(PC_6, OUTPUT);
  pinMode(PC_7, OUTPUT);
  pinMode(PC_8, OUTPUT);
  pinMode(PC_9, OUTPUT);
}

void blinkLED(int times, int blinkDur) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PC_6, HIGH);
    delay(blinkDur);
    digitalWrite(PC_6, LOW);
    delay(200);
  }
}

// indicate a fatal error
void blinkSOS() {
  for (int i = 0; i < 9; i++) {
    digitalWrite(PC_7, HIGH);
    delay(BLINK_TIME_MS * 3 / ((i == 3 || i == 4 || i == 5) ? 1 : 3));
    digitalWrite(PC_7, LOW);
    delay(BLINK_TIME_MS);
  }
  delay(800);
}

///////////////////////
// Arduino functions //
///////////////////////

void setup() {
  initialiseBlinker();
  delay(1000);
  initialiseCAN();
}

void loop() {
  //blinkSOS();
  CANFrame frame = getFrame();
  if (frame.id >= STEPPER_A && frame.id <= STEPPER_C) {
    blinkLED(frame.data, FAST_BLINK_TIME_MS);
  }
}
