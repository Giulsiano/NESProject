#include "nesproj.h"

uint8_t get_header (msg_t* msg){
    return msg->hdr;
}

uint16_t get_payload (msg_t* msg){
    return msg->payload;
}

void set_header (msg_t* msg, uint8_t hdr_data){
    msg->hdr = hdr_data;
}

void set_payload (msg_t* msg, uint16_t payload){
    msg->payload = payload;
}

struct msg_t set_message (uint8_t hdr, uint16_t payload){
    msg_t msg;
    msg.hdr = hdr;
    msg.payload = payload;
    return msg;
}

struct msg_t get_message_from (void* raw_data){
    return *((msg_t*) raw_data);
}
