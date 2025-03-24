/*
 * stepper_controller.ino
 *
 * Firmware for the Mellow FLY-DP5 (STM32F072RBT6) that responds to motor
 * controller CAN frames and drives stepper motors accordingly.
 */

#include "stepper_profiler.hpp"
#include "hal_conf_extra.h"

#include <stm32f0xx_hal_can.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* for CAN -- 1 Mb/s */
#define BITRATE   1000000

#define FILTER_HIGH(id)           ((id & 0x1FFFE000) >> 13)
#define FILTER_LOW(id)            (((id & 0x1FFF) << 3) | 0x4)
#define ESC1_ID_MASK              0x01F8003F /* just match address and magic */
#define FILTER_ESC1_MASK_HIGH     FILTER_HIGH(ESC1_ID_MASK)
#define FILTER_ESC1_MASK_LOW      FILTER_LOW(ESC1_ID_MASK)
#define FILTER_ESC1_ID_HIGH       FILTER_HIGH(0x00C00000) /* just magic */
#define FILTER_ESC1_ID_LOW(addr)  FILTER_LOW(addr) /* just address */

/* for steppers */
#define STEPS_PER_REVOLUTION      200
#define MICROSTEP_MULTIPLIER      8
 
/* current convention: clockwise rotations are positive
 * set this to -1 to switch convention */
#define ROTATION_POLARITY   1

#define X_EN      PC_2
#define X_STEP    PC_15
#define X_DIR     PC_14

#define Y_EN      PA_2
#define Y_STEP    PA_1
#define Y_DIR     PA_0

#define Z_EN      PA_6
#define Z_STEP    PA_5
#define Z_DIR     PA_4

/* mostly for debugging */
#define VERY_FAST_BLINK_TIME_MS   50
#define FAST_BLINK_TIME_MS        150
#define BLINK_TIME_MS             300
#define SLOW_BLINK_TIME_MS        600
#define VERY_SLOW_BLINK_TIME_MS   2000
 
/* Lowest 6 bits of the ext. CAN IDs of the stepper motors
 * as per the B-G431-ESC1 spec */
enum StepperId {
  STEPPER_A = 42, STEPPER_B, STEPPER_C
};

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan);
void blinkLED(int times, int blinkDur);
void blinkMorse(char *s);

struct CANFrame {
  uint32_t id;
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
};

struct CanTiming {
  uint32_t prescaler;
  uint32_t sjw;
  uint32_t tseg1;
  uint32_t tseg2;
};

HAL_StatusTypeDef halStatus = {};
CAN_HandleTypeDef hcan_ = {};

StepperProfiler stepperX(1600, 800);
StepperProfiler stepperY(1600, 800);
StepperProfiler stepperZ(1600, 800);

///////////////////////////////
// Stepper-related functions //
///////////////////////////////

void initialiseSteppers() {
  pinMode(X_EN, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);

  pinMode(Y_EN, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  
  pinMode(Z_EN, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);

  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);
  digitalWrite(Z_EN, LOW);
}

/* Moves the specified stepper posRadians from current position */
void moveStepperRelative(enum StepperId id, float posRadians) {
  posRadians *= ROTATION_POLARITY;

  unsigned stepperStep;
  unsigned stepperDir;
  switch (id) {
    case STEPPER_A:
      stepperStep = X_STEP;
      stepperDir = X_DIR;
      break;
    case STEPPER_B:
      stepperStep = Y_STEP;
      stepperDir = Y_DIR;
      break;
    case STEPPER_C:
      stepperStep = Z_STEP;
      stepperDir = Z_DIR;
      break;
    default:
      blinkMorse("badid");
      return;
  }
  if (posRadians < 0) {
    digitalWrite(stepperDir, HIGH);
  } else {
    digitalWrite(stepperDir, LOW);
  }
  // TODO: keep track of accumulated error
  // TODO: trapezoid motion
  // TODO: wtf is going on with the microstep stuff?
  // Nevermind!!! Evan is writing a motor class that will handle all this
  float numSteps = abs(posRadians * STEPS_PER_REVOLUTION * MICROSTEP_MULTIPLIER / (2 * PI));
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepperStep, HIGH);
    delay(1);
    digitalWrite(stepperStep, LOW);
    delay(1);
  }
}

///////////////////////////
// CAN-related functions //
///////////////////////////

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan_) {
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* initialise CAN pins (PB8 = CAN_RX, PB9 = CAN_TX) */
  GPIO_InitTypeDef CANRxTx = {
    .Pin = GPIO_PIN_8 | GPIO_PIN_9,
    .Mode = GPIO_MODE_AF_PP, /* not open-drain! the docs are wrong! */
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
    .Alternate = GPIO_AF4_CAN,
  };
  HAL_GPIO_Init(GPIOB, &CANRxTx);
}

/* taken from Aavin's SimpleCAN's BaseCAN.cpp */
CanTiming solveCanTiming(uint32_t clockFreq, uint32_t bitrate,
                         uint8_t multiplier) {
  // cf. http://www.bittiming.can-wiki.info/
  CanTiming timing = {};
  uint32_t baseQuanta = 16;
  uint32_t timeQuanta = baseQuanta;

  uint32_t offset = 0;
  bool found = false;

  // start at 16 and work outwards
  while (offset <= 9) {
    // Looking for a timeQuanta of between 8 and 25.
    timeQuanta = baseQuanta - offset;
    if (clockFreq % (bitrate * timeQuanta * multiplier) == 0) {
      found = true;
      break;
    }
    timeQuanta = baseQuanta + offset;
    if (clockFreq % (bitrate * timeQuanta * multiplier) == 0) {
      found = true;
      break;
    }
    offset += 1;
  }

  if (!found) {
    blinkMorse("nosolve");
  }

  timing.prescaler = clockFreq / (bitrate * timeQuanta);
  timing.sjw = 1;
  timing.tseg1 = uint32_t(0.875 * timeQuanta) - 1;

  float samplePoint = (1.0 + timing.tseg1) / timeQuanta;
  float samplePoint2 = (1.0 + timing.tseg1 + 1) / timeQuanta;

  if (abs(samplePoint2 - 0.875) < abs(samplePoint - 0.875)) {
    timing.tseg1 += 1;
    samplePoint = samplePoint2;
  }

  timing.tseg2 = timeQuanta - timing.tseg1 - 1;
#ifdef CAN_DEBUG
  _Serial->print("clockFreq:");
  _Serial->print(clockFreq);
  _Serial->print(", bitrate:");
  _Serial->print(bitrate);
  _Serial->print(", prescaler:");
  _Serial->print(timing.prescaler);
  _Serial->print(", timeQuanta:");
  _Serial->print(timeQuanta);
  _Serial->print(", nominalTimeSeg1:");
  _Serial->print(timing.tseg1);
  _Serial->print(", nominalTimeSeg2:");
  _Serial->print(timing.tseg2);
  _Serial->print(", samplePoint:");
  _Serial->println(samplePoint);

#endif
  return timing;
}

void blinkHalStatus(HAL_StatusTypeDef status) {
  if (status == HAL_OK) {
    blinkMorse("ok");
  } else if (status == HAL_ERROR) {
    if (hcan_.ErrorCode == HAL_CAN_ERROR_TIMEOUT) {
      blinkMorse("errtime");
    }
    blinkMorse("error");
  } else if (status == HAL_BUSY) {
    blinkMorse("busy");
  } else if (status == HAL_TIMEOUT) {
    blinkMorse("timeout");
  } else {
    blinkMorse("unknown");
  }
}

void initialiseCAN() {
  /* OK so turns out the clock we're working with is actually 48 MHz */
  CanTiming timing = solveCanTiming(48'000'000, BITRATE, 1);

  hcan_.Instance = CAN;
  CAN_InitTypeDef *init = &(hcan_.Init);
  init->Prescaler = timing.prescaler;
  init->Mode = CAN_MODE_NORMAL;
  init->SyncJumpWidth = (timing.sjw - 1) << CAN_BTR_SJW_Pos;
  init->TimeSeg1 = (timing.tseg1 - 1) << CAN_BTR_TS1_Pos;
  init->TimeSeg2 = (timing.tseg2 - 1) << CAN_BTR_TS2_Pos;
  init->TimeTriggeredMode = DISABLE;
  init->AutoBusOff = DISABLE;
  init->AutoWakeUp = DISABLE;
  init->AutoRetransmission = ENABLE;
  init->ReceiveFifoLocked = DISABLE;
  init->TransmitFifoPriority = DISABLE;

  halStatus = HAL_CAN_Init(&hcan_);

  /* Filters CAN frames using STM32 CAN filter hardware... like a boss */
  CAN_FilterTypeDef filterConfA = {
    .FilterIdHigh = FILTER_ESC1_ID_HIGH,
    .FilterIdLow = FILTER_ESC1_ID_LOW(STEPPER_A),
    .FilterMaskIdHigh = FILTER_ESC1_MASK_HIGH,
    .FilterMaskIdLow = FILTER_ESC1_MASK_LOW,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank = 0
  };
  halStatus = HAL_CAN_ConfigFilter(&hcan_, &filterConfA);

  CAN_FilterTypeDef filterConfB = {
    .FilterIdHigh = FILTER_ESC1_ID_HIGH,
    .FilterIdLow = FILTER_ESC1_ID_LOW(STEPPER_B),
    .FilterMaskIdHigh = FILTER_ESC1_MASK_HIGH,
    .FilterMaskIdLow = FILTER_ESC1_MASK_LOW,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 1,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank = 0
  };
  halStatus = HAL_CAN_ConfigFilter(&hcan_, &filterConfB);

  CAN_FilterTypeDef filterConfC = {
    .FilterIdHigh = FILTER_ESC1_ID_HIGH,
    .FilterIdLow = FILTER_ESC1_ID_LOW(STEPPER_C),
    .FilterMaskIdHigh = FILTER_ESC1_MASK_HIGH,
    .FilterMaskIdLow = FILTER_ESC1_MASK_LOW,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 2,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank = 0
  };
  halStatus = HAL_CAN_ConfigFilter(&hcan_, &filterConfC);

  halStatus = HAL_CAN_Start(&hcan_);
}

void sendFrame(CANFrame frameToSend) {
  CAN_TxHeaderTypeDef header = {
    .StdId = frameToSend.id, /* ignored */
    .ExtId = frameToSend.id,
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
  };
  uint32_t txMailbox;
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan_) == 0);
  halStatus = HAL_CAN_AddTxMessage(&hcan_, &header, frameToSend.data, &txMailbox);
}

/* if a CAN frame available, return the frame
 * otherwise immediately return a null frame */
CANFrame getFrame() {
  /* as per the filters we've set up, we expect all messages to go into FIFO0 */
  CANFrame frame = {0};
  if (HAL_CAN_GetRxFifoFillLevel(&hcan_, CAN_RX_FIFO0) == 0) return frame;
  CAN_RxHeaderTypeDef frameHeader;
  uint8_t framePayload[8];
  halStatus = HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &frameHeader, framePayload);
  if (halStatus == HAL_ERROR) return frame;
  frame.id = frameHeader.ExtId;
  frame.header = frameHeader;
  for (int i = 0; i < 8; i++) {
    frame.data[i] = framePayload[i];
  }
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

void blinkSOS() {
  for (int i = 0; i < 9; i++) {
    digitalWrite(PC_7, HIGH);
    delay(BLINK_TIME_MS * 3 / ((i == 3 || i == 4 || i == 5) ? 1 : 3));
    digitalWrite(PC_7, LOW);
    delay(BLINK_TIME_MS);
  }
  delay(800);
}

/* printf like it's 1844 */
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
  initialiseSteppers();
  initialiseCAN();
  stepperX.set_target(-3000.0);
  stepperY.set_target(-10000.0);
  stepperZ.set_target(10000.0);
}

void loop() {
  // TODO: transmit CAN status frames 50 times a second
  CANFrame frame = getFrame();
  switch (frame.id) {
    case STEPPER_A:
      //stepperX.set_target(frame.data);
      break;
    case STEPPER_B:
      //motorB.set_target(frame.data);
      break;
    case STEPPER_C:
      //motorC.set_target(frame.data);
      break;
    default:
      break;
  }
  auto[x_step, x_dir] = stepperX.update();
  digitalWrite(X_STEP, x_step);
  digitalWrite(X_DIR, x_dir);
  auto[y_step, y_dir] = stepperY.update();
  digitalWrite(Y_STEP, y_step);
  digitalWrite(Y_DIR, y_dir);
  auto[z_step, z_dir] = stepperZ.update();
  digitalWrite(Z_STEP, z_step);
  digitalWrite(Z_DIR, z_dir);

  /* if (frame.id == 0) { */
  /*   blinkLED(2, VERY_FAST_BLINK_TIME_MS); */
  /* } else { */
  /*   blinkLED(frame.data[0], FAST_BLINK_TIME_MS); */
  /* } */
}
