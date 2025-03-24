// stepper_controller.ino
//
// Firmware for the Mellow FLY-DP5 that responds to motor controller CAN frames
// and drives the stepper motors accordingly.

#include <stm32f0xx_hal_can.h>
#include "hal_conf_extra.h"

#ifndef PI
#define PI           3.14159265358979323846
#endif

#define BLINK_TIME_MS       300
#define FAST_BLINK_TIME_MS  150
#define SLOW_BLINK_TIME_MS  600
#define BITRATE             1000000

// current convention: clockwise rotations are positive
// set this to -1 to switch convention
#define ROTATION_POLARITY   1

#define STEPS_PER_REVOLUTION          200
#define MICROSTEP_MULTIPLIER          8
#define VELOCITY_RAMPUP
#define VELOCITY_RAMPDOWN
#define VELOCITY_MAX

#define VERY_FAST_BLINK_TIME_MS  50
#define VERY_SLOW_BLINK_TIME_MS  2000
  
#define X_EN      PC_2
#define X_STEP    PC_15
#define X_DIR     PC_14
#define X_CS      PC_13

#define Y_EN      PA_2
#define Y_STEP    PA_1
#define Y_DIR     PA_0

#define Z_EN      PA_6
#define Z_STEP    PA_5
#define Z_DIR     PA_4
#define Z_CS      PA_3

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


float stepperError[] = {0, 0, 0};

// stand-ins for the actual CAN IDs of the stepper motors
enum StepperId {
  STEPPER_A = 32, STEPPER_B, STEPPER_C
};

HAL_StatusTypeDef halStatus = {};
CAN_HandleTypeDef hcan_ = {};

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
  GPIO_InitTypeDef CANRx = {
    .Pin = GPIO_PIN_8,
    .Mode = GPIO_MODE_AF_OD,
    .Pull = GPIO_NOPULL,
    /* TODO: set speed dynamically based on bitrate/solved clock division? */
    .Speed = GPIO_SPEED_FREQ_MEDIUM, /* 4 to 10 MHz? */
    //.Speed = GPIO_SPEED_FREQ_LOW, /* up to 4 MHz? */
    .Alternate = GPIO_AF4_CAN,
  };
  GPIO_InitTypeDef CANTx = {
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_AF_OD,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_MEDIUM, /* 4 to 10 MHz? */
    //.Speed = GPIO_SPEED_FREQ_LOW, /* up to 4 MHz? */
    .Alternate = GPIO_AF4_CAN,
  };
  HAL_GPIO_Init(GPIOB, &CANRx);
  HAL_GPIO_Init(GPIOB, &CANTx);
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
  /* CAN "peripheral clock" is the APB clock (PCLK) */
  /* which is derived from SYSCLK (8 MHz) by default -- refer to clock tree */
  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq();
  CanTiming timing = solveCanTiming(clockFreq, BITRATE, 1);

  hcan_.Instance = CAN;
  CAN_InitTypeDef *init = &(hcan_.Init);
  init->Prescaler = timing.prescaler;
  //init->Mode = CAN_MODE_NORMAL;
  init->Mode = CAN_MODE_LOOPBACK;
  init->SyncJumpWidth = timing.sjw;
  init->TimeSeg1 = timing.tseg1;
  init->TimeSeg2 = timing.tseg2;
  init->TimeTriggeredMode = ENABLE;
  init->AutoBusOff = DISABLE;
  init->AutoWakeUp = DISABLE;
  init->AutoRetransmission = DISABLE;
  init->ReceiveFifoLocked = DISABLE;
  init->TransmitFifoPriority = DISABLE;

  halStatus = HAL_CAN_Init(&hcan_);

  /*
   * Currently, we're just trying to pick up all CAN frames to make sure that we
   * can actually pick them up, but for the actual driver we should configure
   * three separate filters (w/ CAN_FILTERMODE_IDLIST) corresponding to the
   * three stepper IDs.
   */
  CAN_FilterTypeDef filterConf = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 0,
    //.FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank = 0
  };

  halStatus = HAL_CAN_ConfigFilter(&hcan_, &filterConf);

  halStatus = HAL_CAN_Start(&hcan_);
  blinkHalStatus(halStatus);
}

/* blocks until a CAN frame becomes available */
/* untested */
CANFrame getFrame() {
  /* as per the filter(s) we've set up, we expect messages to go into FIFO0 */
  CANFrame frame = {};
  while (HAL_CAN_GetRxFifoFillLevel(&hcan_, CAN_RX_FIFO0) == 0) {
    blinkLED(1, VERY_SLOW_BLINK_TIME_MS);
  };
  CAN_RxHeaderTypeDef frameHeader;
  uint8_t framePayload[8];
  halStatus = HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &frameHeader, framePayload);
  blinkHalStatus(halStatus);
  if (halStatus == HAL_ERROR) return frame;
  blinkLED(10, VERY_FAST_BLINK_TIME_MS);
  frame.id = STEPPER_A;
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
  initialiseSteppers();
  moveStepperRelative(STEPPER_A, -PI);
  moveStepperRelative(STEPPER_A, PI);
  delay(1000);
  initialiseCAN();
}

void loop() {
  // blinkSOS();
  CANFrame frame = getFrame();
  if (frame.id >= STEPPER_A && frame.id <= STEPPER_C) {
    blinkLED(frame.data[7], FAST_BLINK_TIME_MS);
  }
}
