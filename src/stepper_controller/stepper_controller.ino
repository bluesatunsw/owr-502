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
#define BITRATE         1000000
/* how often the status of the steppers should be reported on CAN, in Hz */
#define FEEDBACK_FREQ   50

#define FILTER_HIGH(id)           (((id) & 0x1FFFE000) >> 13)
#define FILTER_LOW(id)            ((((id) & 0x1FFF) << 3) | 0x4)
#define ESC1_ID_MASK              0x01F8003F /* just match address and magic */
#define FILTER_ESC1_MASK_HIGH     FILTER_HIGH(ESC1_ID_MASK)
#define FILTER_ESC1_MASK_LOW      FILTER_LOW(ESC1_ID_MASK)
#define FILTER_ESC1_ID_HIGH       FILTER_HIGH(0x00C00000) /* just magic */
#define FILTER_ESC1_ID_LOW(addr)  FILTER_LOW(addr) /* just address */

#define CONTROL_REQUEST_INDEX     1

/* binary: b x 1s */
#define MASK(b)                   ((1ul << (b)) - 1)
/* hi, lo  inclusive */
#define GET_BIT_RANGE(x, hi, lo)  ((x & MASK(hi + 1) & ~MASK(lo)) >> lo)

/* for steppers */
constexpr int kStepsPerRev{200};
constexpr int kMicrostepMulti{8};
constexpr int kTicksPerRev{kStepsPerRev*kMicrostepMulti};

/* current convention: clockwise rotations are positive
 * set this to -1 to switch convention */
#define ROTATION_POLARITY   1

#define X_EN      PC_2
#define X_STEP    PC_15
#define X_DIR     PC_14
#define X_UART    PC_13

#define Y_EN      PA_2
#define Y_STEP    PA_1
#define Y_DIR     PA_0
#define Y_UART    PC_3

#define Z_EN      PA_6
#define Z_STEP    PA_5
#define Z_DIR     PA_4
#define Z_UART    PA_3

/* for the stepper motor profiling */
// TODO: make this run properly at higher speeds -- can't go above 1.5x this speed
constexpr int kMaxSpeed{kTicksPerRev};
constexpr int kMaxAcceleration{kTicksPerRev};

/* mostly for debugging */
#define VERY_FAST_BLINK_TIME_MS   50
#define FAST_BLINK_TIME_MS        150
#define BLINK_TIME_MS             300
#define SLOW_BLINK_TIME_MS        600
#define VERY_SLOW_BLINK_TIME_MS   2000
 
/* Lowest 6 bits of the ext. CAN IDs of the stepper motors
 * as per the B-G431-ESC1 spec */
enum StepperId {
  STEPPER_A = 10, STEPPER_B, STEPPER_C
};

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void blinkLED(int times, int blinkDur);
void blinkMorse(char *s);

// i feel like we should have a project-wide library or header file for these definitions
// TODO (LOW PRIORITY): better class inheritance
enum ControlMode {
  Disabled = 0,
  Voltage = 1,
  Torque = 2,
  Velocity = 3,
  Position = 4,
  VelocityOpenLoop = 5,
  PositionOpenLoop = 6
};

enum Priority {
  Exceptional = 0,
  Immediate = 1,
  Fast = 2,
  High = 3,
  Nominal = 4,
  Low = 5,
  Slow = 6,
  Optional = 7
};

struct ESC1FrameId {
  bool is_valid : 1;
  unsigned priority : 3;
  unsigned anonymous : 1;
  unsigned magic : 6;
  unsigned api_page : 2;
  unsigned api_index : 10;
  unsigned reserved : 1;
  unsigned address : 6;
};

struct ESC1ControlRequest {
  struct ESC1FrameId frameId;
  ControlMode control_mode : 8;
  uint32_t setpoint : 32; /* cast to float */
};

struct CanTiming {
  uint32_t prescaler;
  uint32_t sjw;
  uint32_t tseg1;
  uint32_t tseg2;
};

union FloatBitsConverter {
  float f;
  uint32_t u;
};

HAL_StatusTypeDef halStatus = {};
CAN_HandleTypeDef hcan_ = {};
TIM_HandleTypeDef htim_ = {};

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
  init->AutoRetransmission = DISABLE;
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

/* really should be a ESC1FrameId class method */
ESC1FrameId extCANToESC1(uint32_t extCANId) {
  ESC1FrameId esc1FrameId = {
    .is_valid = 0,
    .priority = GET_BIT_RANGE(extCANId, 28, 26),
    .anonymous = GET_BIT_RANGE(extCANId, 25, 25),
    .magic = GET_BIT_RANGE(extCANId, 24, 19),
    .api_page = GET_BIT_RANGE(extCANId, 18, 17),
    .api_index = GET_BIT_RANGE(extCANId, 16, 7),
    .reserved = GET_BIT_RANGE(extCANId, 6, 6),
    .address = GET_BIT_RANGE(extCANId, 5, 0)
  };
  esc1FrameId.is_valid = (esc1FrameId.magic == 0x18);
  return esc1FrameId;
}

uint32_t ESC1ToExtCAN(ESC1FrameId id) {
  uint32_t extId = 0;
  if (!id.is_valid) return extId;
  extId = id.address
        | (1u << 6) /* reserved */
        | id.api_index << 7
        | id.api_page << 17
        | (0x18 << 19) /* magic */
        | id.anonymous << 25
        | id.priority << 26;
  return extId;
}

/* as above but with more suitable type */
/* not sure how to make this more extensible (elegantly handle different commands) but now's not the time */
ESC1ControlRequest getControlRequestFrame() {
  ESC1ControlRequest controlRequest = {0};
  if (HAL_CAN_GetRxFifoFillLevel(&hcan_, CAN_RX_FIFO0) == 0) return controlRequest;
  CAN_RxHeaderTypeDef frameHeader;
  uint8_t framePayload[8];
  FloatBitsConverter floatConverter;
  halStatus = HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &frameHeader, framePayload);
  if (halStatus == HAL_ERROR) return controlRequest;
  if (frameHeader.IDE != CAN_ID_EXT) return controlRequest;
  if (frameHeader.RTR != CAN_RTR_DATA) return controlRequest;
  controlRequest.frameId = extCANToESC1(frameHeader.ExtId);
  if (!controlRequest.frameId.is_valid) return controlRequest;
  /* only listen to API page 1 commands */
  if (controlRequest.frameId.api_page != 1) return controlRequest;
  switch (controlRequest.frameId.api_index) {
   case CONTROL_REQUEST_INDEX:
    /* expect 5 bytes of data for this type of request */
    if (frameHeader.DLC < 5) return controlRequest;
    controlRequest.control_mode = (enum ControlMode)framePayload[0];
    floatConverter.u = framePayload[1] << 24 | framePayload[2] << 16 | framePayload[3] << 8 | framePayload[4];
    controlRequest.setpoint = floatConverter.f;
    break;
   default:
    controlRequest.frameId.is_valid = 0;
  }
  return controlRequest;
}

/* sends a CAN "feedback" frame reporting the position and velocity of the given stepper motor with the specified "address" */
void reportStatus(uint8_t address, StepperProfiler stepper) {
  ESC1FrameId esc1FrameId = {
    .is_valid = 1,
    .priority = Nominal,
    .anonymous = 0,
    .magic = 0x18,
    .api_page = 2,
    .api_index = 1,
    .reserved = 1,
    .address = address
  };
  uint32_t extId = ESC1ToExtCAN(esc1FrameId);
  uint8_t frameData[8];
  /* i hope these casts work */
  FloatBitsConverter floatConverter;
  floatConverter.f = stepper.get_position();
  uint32_t position = floatConverter.u;
  floatConverter.f = stepper.get_velocity();
  uint32_t velocity = floatConverter.u;
  frameData[0] = GET_BIT_RANGE(position, 31, 24);
  frameData[1] = GET_BIT_RANGE(position, 23, 16);
  frameData[2] = GET_BIT_RANGE(position, 15, 8);
  frameData[3] = GET_BIT_RANGE(position, 7, 0);
  frameData[4] = GET_BIT_RANGE(velocity, 31, 24);
  frameData[5] = GET_BIT_RANGE(velocity, 23, 16);
  frameData[6] = GET_BIT_RANGE(velocity, 15, 8);
  frameData[7] = GET_BIT_RANGE(velocity, 7, 0);
  CAN_TxHeaderTypeDef header = {
    .StdId = extId, /* ignored */
    .ExtId = extId,
    .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
  };
  uint32_t txMailbox;
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan_) == 0);
  halStatus = HAL_CAN_AddTxMessage(&hcan_, &header, frameData, &txMailbox);
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

StepperProfiler stepperX(kMaxSpeed, kMaxAcceleration);
StepperProfiler stepperY(kMaxSpeed, kMaxAcceleration);
StepperProfiler stepperZ(kMaxSpeed, kMaxAcceleration);

unsigned long nextReportTime = 0;

void setup() {
  initialiseBlinker();
  initialiseSteppers();
  initialiseCAN();
}

/* tight loop! */
void loop() {
  ESC1ControlRequest frame = getControlRequestFrame();
  /* support position control only */
  if (frame.frameId.is_valid && frame.control_mode == Position) {
    switch (frame.frameId.address) {
     case STEPPER_A:
      stepperX.set_target(frame.setpoint * ROTATION_POLARITY);
      break;
     case STEPPER_B:
      stepperY.set_target(frame.setpoint * ROTATION_POLARITY);
      break;
     case STEPPER_C:
      stepperZ.set_target(frame.setpoint * ROTATION_POLARITY);
      break;
     default:
      break;
    }
  }

  /* update step and direction as per internal stepper profile */
  auto[x_step, x_dir] = stepperX.update();
  digitalWrite(X_STEP, x_step);
  digitalWrite(X_DIR, x_dir);
  auto[y_step, y_dir] = stepperY.update();
  digitalWrite(Y_STEP, y_step);
  digitalWrite(Y_DIR, y_dir);
  auto[z_step, z_dir] = stepperZ.update();
  digitalWrite(Z_STEP, z_step);
  digitalWrite(Z_DIR, z_dir);

  /* transmit CAN status frames 50 times a second */
  if (nextReportTime < millis()) {
    reportStatus(STEPPER_A, stepperX);
    reportStatus(STEPPER_B, stepperY);
    reportStatus(STEPPER_C, stepperZ);
    if (nextReportTime == 0) {
      nextReportTime = millis();
    }
    while (nextReportTime < millis()) {
      nextReportTime += 1000 / FEEDBACK_FREQ;
    }
  }
}
