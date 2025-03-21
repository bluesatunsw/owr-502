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
void blinkMorse(char *s);

// stand-ins for the actual CAN IDs of the stepper motors
enum StepperId {
  STEPPER_A = 32, STEPPER_B, STEPPER_C
};

HAL_StatusTypeDef halStatus = {};
CAN_HandleTypeDef hcan_ = {};

///////////////////////////
// CAN-related functions //
///////////////////////////

struct CANFrame {
  // TODO: add fields
  uint32_t id {};
  uint32_t data {};
};

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan_) {
  __HAL_RCC_CAN1_CLK_ENABLE();
}

void initialiseCAN() {
  hcan_.Instance = CAN;
  /* TODO: comb through CAN reference spec and check that these values all sensible for what we're doing */
  CAN_InitTypeDef *init = &(hcan_.Init);
  init->Prescaler = 1; /* "length of a time quantum", between 1 and 1024 */
  init->Mode = CAN_MODE_NORMAL;
  init->SyncJumpWidth = CAN_SJW_1TQ;
  init->TimeSeg1 = CAN_BS1_1TQ;
  init->TimeSeg2 = CAN_BS2_1TQ;
  init->TimeTriggeredMode = ENABLE;
  init->AutoBusOff = ENABLE;
  init->AutoWakeUp = ENABLE;
  init->AutoRetransmission = DISABLE;
  init->ReceiveFifoLocked = DISABLE;
  init->TransmitFifoPriority = DISABLE;

  halStatus = HAL_CAN_Init(&hcan_);

  // bedugging!
  if (halStatus == HAL_OK) {
    blinkMorse("ok");
  } else if (halStatus == HAL_ERROR) {
    blinkMorse("error");
  } else if (halStatus == HAL_BUSY) {
    blinkMorse("busy");
  } else if (halStatus == HAL_TIMEOUT) {
    blinkMorse("timeout");
  } else {
    blinkMorse("unknown");
  }

  // TODO: HAL_CAN_ConfigFilter()?
  
  halStatus = HAL_CAN_Start(&hcan_); // this currently errors, possibly we need to config filter
  
  if (halStatus == HAL_OK) {
    blinkMorse("ok");
  } else if (halStatus == HAL_ERROR) {
    blinkMorse("error");
  } else if (halStatus == HAL_BUSY) {
    blinkMorse("busy");
  } else if (halStatus == HAL_TIMEOUT) {
    blinkMorse("timeout");
  } else {
    blinkMorse("unknown");
  }
}

// blocks until a CAN frame becomes available
CANFrame getFrame() {
  // do we need to monitor both receive FIFOs? does the hardware automatically fill both or just one?
  while (HAL_CAN_GetRxFifoFillLevel(&hcan_, CAN_RX_FIFO0) == 0) {
    blinkLED(1, SLOW_BLINK_TIME_MS);
  };
  CAN_RxHeaderTypeDef frameHeader;
  uint8_t framePayload[8];
  halStatus = HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &frameHeader, framePayload);
  // TODO: check this works and copy the data into the CANFrame struct
  CANFrame frame = {};
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

// printf like it's 1844
void blinkMorse(char *s) {
  char c = s[0];
  const char *morse_lookup[] = {
    ".-",
    "-...",
    "-.-.",
    "-..",
    ".",
    "..-.",
    "--.",
    "....",
    "..",
    ".---",
    "-.-",
    ".-..",
    "--",
    "-.",
    "---",
    ".--.",
    "--.-",
    ".-.",
    "...",
    "-",
    "..-",
    "...-",
    ".--",
    "-..-",
    "-.--",
    "--..",
  };
  for (int i = 0;; i++) {
    c = s[i];
    if (c == '\0') break;
    if (c >= 'A' && c <= 'Z') {
      c = c - 'A';
    } else if (c >= 'a' && c <= 'z') {
      c = c - 'a';
    } else  {
      // invalid character
      c = 'x' - 'a';
    }
    const char *pattern = morse_lookup[c];
    while (*pattern != '\0') {
      if (*pattern == '-') {
        blinkLED(1, SLOW_BLINK_TIME_MS);
      } else {
        blinkLED(1, FAST_BLINK_TIME_MS);
      }
      pattern++;
    }
    delay(500);
  }
}

///////////////////////
// Arduino functions //
///////////////////////

void setup() {
  initialiseBlinker();
  initialiseCAN();
}

void loop() {
  // blinkSOS();
  CANFrame frame = getFrame();
  if (frame.id >= STEPPER_A && frame.id <= STEPPER_C) {
    blinkLED(frame.data, FAST_BLINK_TIME_MS);
  }
}
