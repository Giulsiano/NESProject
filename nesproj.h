/**
Header file containing the common declarations and things used by sensors
**/
#ifndef NESPROJ_H_
#define NESPROJ_H_  1

#include "contiki.h"
#include "sys/etimer.h"
#include "stdio.h" // printf(), memcpy()
#include "stdlib.h"
#include "dev/button-sensor.h"
#include "net/rime/rime.h"
#include "net/linkaddr.h"
#include "string.h"
#include "limits.h"     // INT_MIN
#include "dev/leds.h"
#include "stdbool.h"    //true, false macro

#define MAX_RETRANSMISSIONS 5

// Rime node addresses for each node
#define CU_ADDR_0   3
#define CU_ADDR_1   0

#define DOOR_ADDR_0 1
#define DOOR_ADDR_1 0

#define GATE_ADDR_0 2
#define GATE_ADDR_1 0

#define RMT_ADDR_0 4
#define RMT_ADDR_1 0

// Message length, equal for all messages
#define MSG_LEN     8

// Channels used for runicast and broadcast communications
#define RU_CH 144
#define BC_CH 129

// How long leds have to blink
#define BLINK_PERIOD    CLOCK_SECOND*2
#define SMPL_TEMP_PERIOD_SECONDS    10
#define SMPL_TEMP_PERIOD    CLOCK_SECOND*SMPL_TEMP_PERIOD_SECONDS

// Application message and function to manage it
typedef struct msg_t {
    uint8_t hdr;
    uint16_t payload;
} msg_t;

enum msg_hdr_t{
    TEMP_MSG = 0x0F,
    LIGHT_MSG = 0x0A,
    CMD_MSG = 0x00
};

uint8_t get_header (msg_t* msg);
uint16_t get_payload (msg_t* msg);
void set_header (msg_t* msg, uint8_t hdr_data);
void set_payload (msg_t* msg, uint16_t payload);
struct msg_t set_message (uint8_t hdr, uint16_t payload);
struct msg_t get_message_from (void* raw_data);

#define COMMAND_NUMBER 6
enum user_command {
               NO_CMD = 0,
               ALARM_ON_OFF = 1,
               GATE_UN_LOCK = 2,
               ENTRANCE_OPEN_CLOSE = 3,
               TEMP_AVG = 4,
               EXT_LIGHT = 5,
               HVAC_ON_OFF = 6
};

enum alarm_state {
    DISABLED,
    ENABLED,
    ENABLING
};

enum onoff_state {
    OFF,
    ON
};

enum entrance_state {
    CLOSED,
    MOVING
};

enum lock_state {
    LOCKED,
    UNLOCKED,
    LOCKING
};

enum message {
    ALARM_ENABLED,
    ALARM_DISABLED,
    ALARM_ENABLING,
    GATE_LOCK,
    GATE_UNLOCK,
    ENTRANCE_OPEN,
    ENTRANCE_CLOSE,
    GET_TEMP,
    GET_LIGHT,
};
#endif
