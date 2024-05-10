#ifndef __SUBARU_LEVORG_VNX_H__
#define __SUBARU_LEVORG_VNX_H__

/* #define DEBUG_MODE */

// Receive Only Two CAN Ids
#define CAN_ID_CCU 0x390
#define CAN_ID_TCU 0x174

#define CAN_ID_SCU 0x048  // Shift Position
#define CAN_ID_MCU 0x139  // Speed
#define CAN_ID_ECU 0x040  // Accelerator position

// CCU and TCU STATUS
enum cu_status {
    ENGINE_STOP,
    NOT_READY,
    READY,
    IDLING_STOP_ON,
    IDLING_STOP_OFF
};

// STATUS
enum status {
    PROCESSING,
    CANCELLED,
    FAILED,
    SUCCEEDED
};

// MODE
enum debug_mode {
    NORMAL,
    DEBUG,
    CANDUMP
};

// SHIFT
#define P 0x04
#define R 0x03
#define N 0x02
#define D 0x01

// Threshold for change I -> S mode
#define ACCEL_THRESHOLD 25

// Time to return I-MODE
#define S_MODE_TIME_LIMIT 1

extern enum debug_mode DebugMode;

// for Calculate Check Sum
#define SUM_CHECK_DIVIDER 365


#define MAX_RETRY 2

#endif /* __SUBARU_LEVORG_VNX_H_ */
