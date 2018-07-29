#include "nesproj.h"
#include "string.h"
#include "sys/stimer.h"
#include "sys/etimer.h"
#include "stdarg.h"

// Wait for command period and the maximum number the user can press the button for
#define CMD_PERIOD  CLOCK_SECOND*4
#define MAX_BUTTON_PRESS    COMMAND_NUMBER
#define MONITOR_PAUSE   CLOCK_SECOND*2
#define GATE_ACK_MASK   0x02
#define DOOR_ACK_MASK   0x01
#define ALL_ACK_MASK    (GATE_ACK_MASK | DOOR_ACK_MASK)

// Checking if a message is from door or gate node
#define IS_FROM_DOOR() last_sender[0] == DOOR_ADDR_0 && last_sender[1] == DOOR_ADDR_1
#define IS_FROM_GATE() last_sender[0] == GATE_ADDR_0 && last_sender[1] == GATE_ADDR_1

// Rime address for this node
linkaddr_t cu_addr = {{CU_ADDR_0, CU_ADDR_1}};
uint8_t last_sender[2];

PROCESS(button_process, "Central Unit Button Process");
PROCESS(main_process, "Central Unit Main Process");
PROCESS(msg_process, "Central Unit Message Manager");
PROCESS(monitor_process, "Central Unit Monitor Manager");

AUTOSTART_PROCESSES(&main_process, &button_process, &monitor_process, &msg_process);

// Custom events this node has to manage
static process_event_t valid_cmd_ev;
static process_event_t sensor_msg_ev;
static process_event_t update_monitor_ev;
static process_event_t update_state_ev;

// Node state and command to issue
enum user_command cmd_issued;
enum alarm_state alarm_state;
enum entrance_state entrance_state;
enum lock_state gate_lock_state;

// light and temperature
int light;
int temperature;

// Message for updating the UI
enum monitor_message {
    PRINT_ISSUED_COMMAND,
    PRINT_MENU,
    PRINT_TEMP,
    PRINT_LIGHT,
    PRINT_WAIT_CLOSE,
    PRINT_WAIT_TEMP,
    PRINT_FULL_QUEUE,
    PRINT_ALARM_ACTIVE,
    PRINT_ALARM_DISABLED,
    PRINT_ALARM_ENABLING,
    PRINT_COMMAND_NOT_VALID,
    PRINT_UNLOCK_GATE,
    PRINT_LOCKING_GATE,
    PRINT_LOCKED_GATE,
    PRINT_ENTRANCE_OPEN,
    PRINT_ENTRANCE_CLOSED,
    PRINT_LIGHT_REQUESTED
};

//Definition of the receiving & sending callback functions
static void broadcast_recv (struct broadcast_conn *c, const linkaddr_t *from){
	last_sender[0] = (uint8_t) from->u8[0];
	last_sender[1] = (uint8_t) from->u8[1];
	process_post(NULL, sensor_msg_ev, packetbuf_dataptr());
}

static void runicast_recv (struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
    last_sender[0] = (uint8_t) from->u8[0];
    last_sender[1] = (uint8_t) from->u8[1];
    process_post(NULL, sensor_msg_ev, packetbuf_dataptr());
}

static void broadcast_sent( struct broadcast_conn *c, int status, int num_tx){
    // Do nothing
}

static void runicast_sent (struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	// Do nothing
}

static void runicast_timedout (struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	// Do nothing
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent};
static const struct runicast_callbacks runicast_calls = {runicast_recv,
                                                         runicast_sent,
                                                         runicast_timedout
                                                     };
// Connection data structures
struct broadcast_conn broadcast;
struct runicast_conn runicast;

// Send the message to the node specified by rime_addr_0 and rime_addr_1
uint8_t send_uc_msg(msg_t* msg, uint32_t size, linkaddr_t dest_addr){
	if(!runicast_is_transmitting(&runicast)) {
		linkaddr_t recv = dest_addr;
		packetbuf_copyfrom(msg, size);
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS );
	}
    else {
        return 1;
	}
    return 0;
}

// Send broadcast message
uint8_t send_bc_msg(void* msg, uint32_t size){
	if(!runicast_is_transmitting(&runicast)) {
		packetbuf_copyfrom(msg, size);
        broadcast_send(&broadcast);
	}
    else {
        return 1;
	}
    return 0;
}

PROCESS_THREAD(main_process, ev, data){
    PROCESS_BEGIN();

	static enum message out_msg;
    static enum monitor_message mon_msg;
    static msg_t msg;
    static struct etimer monitor_timer;

    // Init state
    linkaddr_set_node_addr(&cu_addr);
    update_monitor_ev = process_alloc_event();
    update_state_ev = process_alloc_event();
    cmd_issued = NO_CMD;
    alarm_state = DISABLED;
    entrance_state = CLOSED;
    gate_lock_state = UNLOCKED;
    light = INT_MIN;
    temperature = INT_MAX;
    mon_msg = PRINT_MENU;

    process_post(&monitor_process, update_monitor_ev, (void*) mon_msg);
	while (true){
		PROCESS_WAIT_EVENT();
        // A valid command has been issued
        if (ev == valid_cmd_ev){
			cmd_issued = (enum message) data;
            process_post(&monitor_process, update_monitor_ev, (void*) PRINT_ISSUED_COMMAND);
            if (alarm_state == ENABLED && cmd_issued != ALARM_ON_OFF){
                cmd_issued = NO_CMD;
                mon_msg = PRINT_COMMAND_NOT_VALID;
            }
            else {
                switch (cmd_issued) {
                    case ALARM_ON_OFF:
                        out_msg = (alarm_state == ENABLED) ? ALARM_DISABLED : ALARM_ENABLED;
                        break;

                    case GATE_UN_LOCK:
                        if (entrance_state == CLOSED){
                            out_msg = (gate_lock_state == UNLOCKED) ? GATE_LOCK : GATE_UNLOCK;
                        }
                        else {
                            cmd_issued = NO_CMD;
                            mon_msg = PRINT_WAIT_CLOSE;
                        }
                        break;

                    case ENTRANCE_OPEN_CLOSE:
                        if (gate_lock_state == LOCKED){
                            cmd_issued = NO_CMD;
                            mon_msg = PRINT_UNLOCK_GATE;
                        }
                        else if (entrance_state == CLOSED){
                            out_msg = ENTRANCE_OPEN;
                            mon_msg = PRINT_ENTRANCE_OPEN;
                        }
                        else {
                            cmd_issued = NO_CMD;
                            mon_msg = PRINT_WAIT_CLOSE;
                        }
                        break;

                    case TEMP_AVG:
                        out_msg = GET_TEMP;
                        break;

                    case EXT_LIGHT:
                        out_msg = GET_LIGHT;
                        break;

                    case HVAC_ON_OFF:
                        cmd_issued = NO_CMD;
                        mon_msg = PRINT_COMMAND_NOT_VALID;
                        break;

                    default:
                        cmd_issued = NO_CMD;
                        mon_msg = PRINT_COMMAND_NOT_VALID;
                        break;
                }
            }
            if (cmd_issued != NO_CMD){
                process_post(&msg_process, PROCESS_EVENT_MSG, (void*) out_msg);
            }
            else {
                etimer_set(&monitor_timer, MONITOR_PAUSE);
                process_post(&monitor_process, update_monitor_ev, (void*) mon_msg);
            }
		}

        // A message from message process has been received
        if (ev == update_state_ev){
            etimer_set(&monitor_timer, MONITOR_PAUSE);
            msg = get_message_from(data);
            if (msg.hdr == CMD_MSG){
                switch (msg.payload){
                    case ALARM_ENABLING:
                        alarm_state = ENABLING;
                        mon_msg = PRINT_ALARM_ENABLING;
                        break;

                    case ALARM_DISABLED:
                    case ALARM_ENABLED:
                        alarm_state = (msg.payload == ALARM_ENABLED) ? ENABLED : DISABLED;
                        mon_msg = (alarm_state == ENABLED) ? PRINT_ALARM_ACTIVE :
                                                             PRINT_ALARM_DISABLED;
                        break;

                    case GATE_LOCK:
                    case GATE_UNLOCK:
                        gate_lock_state = (msg.payload == GATE_LOCK) ? LOCKED : UNLOCKED;
                        mon_msg = PRINT_LOCKED_GATE;
                        break;

                    case ENTRANCE_OPEN:
                    case ENTRANCE_CLOSE:
                        if (msg.payload == ENTRANCE_OPEN){
                            entrance_state = MOVING;
                            mon_msg = PRINT_ENTRANCE_OPEN;
                        }
                        else {
                            entrance_state = CLOSED;
                            mon_msg = PRINT_ENTRANCE_CLOSED;
                        }
                        break;

                    case GET_LIGHT:
                        mon_msg = PRINT_LIGHT_REQUESTED;
                        break;

                    default:
                        printf("\nMessage not recognized\n");
                        break;
                }
            }
            else if (msg.hdr == LIGHT_MSG){
                light = msg.payload;
                mon_msg = PRINT_LIGHT;
            }
            else if (msg.hdr == TEMP_MSG){
                if (msg.payload == (uint16_t) INT_MIN){
                    mon_msg = PRINT_WAIT_TEMP;
                }
                else {
                    temperature = msg.payload;
                    mon_msg = PRINT_TEMP;
                }
            }
            process_post(&monitor_process, update_monitor_ev, (void*) mon_msg);
        }
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&monitor_timer)){
            mon_msg = PRINT_MENU;
            process_post(&monitor_process, update_monitor_ev, (void*) mon_msg);
        }
	}
	PROCESS_END();

	return 0;
}

PROCESS_THREAD(button_process, ev, data){
	static uint8_t button_count = 0;
	static struct etimer button_timer;

    PROCESS_BEGIN();

	// Event fired when the user has issued a valid command
    valid_cmd_ev = process_alloc_event();

	SENSORS_ACTIVATE(button_sensor);//Button sensor activation
	while (true){
        PROCESS_WAIT_EVENT();
		if (ev == sensors_event && data == &button_sensor) {
            if (button_count == 0)
                etimer_set(&button_timer, CMD_PERIOD);
            else
                etimer_restart(&button_timer);
            ++button_count;

            // Check if user has pressed the button too many times
			if(button_count > MAX_BUTTON_PRESS) {
				button_count = 0;
				etimer_stop(&button_timer);
			} else
				etimer_restart(&button_timer);
		}
        // Send the command issued
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&button_timer)){
			if (process_post(&main_process, valid_cmd_ev, (void*) (int) button_count) != PROCESS_ERR_OK) {
                process_post(&monitor_process, update_monitor_ev, (void*) PRINT_FULL_QUEUE);
			}
            button_count = 0;
		}
	}
	SENSORS_DEACTIVATE(button_sensor);
	PROCESS_END();
	return 0;
}

PROCESS_THREAD(msg_process, ev, data){
    static msg_t msg;
    static enum message main_msg;
    static struct stimer temp_smpl_timer;
    static struct stimer wait_temp_avg;
    static linkaddr_t dest_addr;
    static bool is_temp_ready = false;

    PROCESS_EXITHANDLER(broadcast_close(&broadcast));
    PROCESS_EXITHANDLER(runicast_close(&runicast));
    PROCESS_BEGIN();

    // Init
    static uint8_t closed_entrance_bit = 0x0;
    static uint8_t alarm_on_bit = 0x0;
    update_state_ev = process_alloc_event();
    sensor_msg_ev = process_alloc_event();
    stimer_set(&wait_temp_avg, 5*SMPL_TEMP_PERIOD_SECONDS);
    broadcast_open(&broadcast, BC_CH, &broadcast_call);
    runicast_open(&runicast, RU_CH, &runicast_calls);

    while (true) {
        PROCESS_WAIT_EVENT();
        if (ev == sensor_msg_ev){
            msg = get_message_from(data);
            if (msg.hdr == CMD_MSG){
                switch (msg.payload){
                    case ALARM_ENABLED:
                    case ALARM_DISABLED:
                        if (IS_FROM_DOOR()){
                            alarm_on_bit |= DOOR_ACK_MASK;
                        }
                        if (IS_FROM_GATE()){
                            alarm_on_bit |= GATE_ACK_MASK;
                        }
                        if (alarm_on_bit == ALL_ACK_MASK){
                            process_post(&main_process, update_state_ev, (void*) &msg);
                            alarm_on_bit = 0x0;
                        }
                        break;

                    case ENTRANCE_CLOSE:
                        if (IS_FROM_DOOR()){
                            closed_entrance_bit |= DOOR_ACK_MASK;
                        }
                        if (IS_FROM_GATE()){
                            closed_entrance_bit |= GATE_ACK_MASK;
                        }
                        if (closed_entrance_bit == ALL_ACK_MASK){
                            process_post(&main_process, update_state_ev, (void*) &msg);
                            closed_entrance_bit = 0x0;
                        }
                        break;

                    default:
                        process_post(&main_process, update_state_ev, (void*) &msg);
                        break;
                }
            }
            else {
                process_post(&main_process, update_state_ev, (void*) &msg);
            }
        }
        else if (ev == PROCESS_EVENT_MSG){
            main_msg = (enum message) data;

            // Check if it makes sense to request a new message from temperature
            // sensor or it is a waste of energy
            if (main_msg == GET_TEMP){
                // INT_MIN means temperature has been requested before 50s have
                // passed since the start of the network
                if (stimer_expired(&wait_temp_avg) == 0){
                    msg.hdr = TEMP_MSG;
                    msg.payload = (uint16_t) INT_MIN;
                    process_post(&main_process, update_state_ev, (void*) &msg);
                }
                else {
                    // Set the second timer which expires in 10s, that is the
                    // temperature sampling time
                    stimer_set(&temp_smpl_timer, SMPL_TEMP_PERIOD_SECONDS);
                    msg.hdr = CMD_MSG;
                    msg.payload = main_msg;
                    dest_addr.u8[0] = DOOR_ADDR_0;
                    dest_addr.u8[1] = DOOR_ADDR_1;
                    send_uc_msg(&msg, sizeof(msg), dest_addr);
                }
                if (is_temp_ready == true){
                    if (stimer_expired(&temp_smpl_timer) != 0){
                        stimer_restart(&temp_smpl_timer);
                        msg.hdr = CMD_MSG;
                        msg.payload = main_msg;
                        dest_addr.u8[0] = DOOR_ADDR_0;
                        dest_addr.u8[1] = DOOR_ADDR_1;
                        send_uc_msg(&msg, sizeof(msg), dest_addr);
                    }
                    else {
                        // It isn't needed a new request to the node since it will
                        // respond the same thing as before
                        msg.hdr = TEMP_MSG;
                        msg.payload = temperature;
                        process_post(&main_process, update_state_ev, (void*) &msg);
                    }
                }
            }
            else {
                msg.hdr = CMD_MSG;
                msg.payload = main_msg;
                // See if it is a broadcast or runicast message to be sent
                switch (main_msg){
                    case ALARM_ENABLED:
                        // this is the case when the user sets the alarm while
                        // the entrance is moving. ALARM_ENABLING means it will be
                        // enabled when the entrance is close, since ALARM_ENABLED
                        // message has just been sent it is not needed to send it
                        // again
                        if (alarm_state == ENABLING){
                            msg.payload = ALARM_ENABLING;
                            process_post(&main_process, update_state_ev, (void*) &msg);
                        }
                        else send_bc_msg(&msg, sizeof(msg));
                        break;

                    case ENTRANCE_OPEN:
                    case ALARM_DISABLED:
                        send_bc_msg(&msg, sizeof(msg));
                        break;

                    case GET_LIGHT:
                    case GATE_LOCK:
                    case GATE_UNLOCK:
                        dest_addr.u8[0] = GATE_ADDR_0;
                        dest_addr.u8[1] = GATE_ADDR_1;
                        send_uc_msg(&msg, sizeof(msg), dest_addr);

                        // Since the ack is implicit in the runicast call, there
                        // is the need to update the state of the node with this
                        // call
                        process_post(&main_process, update_state_ev, (void*) &msg);
                        break;

                    default:
                        break;
                }
            }
        }
    }
    PROCESS_END();
    return 0;
}

void print_framed (int count, ...){
    va_list strings;
    const char* frame = "#############################################";
    uint8_t i = 0;

    va_start(strings, count);
    printf("%s\n", frame);
    for (i = 0; i < count; ++i){
        printf("%s\n", va_arg(strings, char*));
    }
    printf("%s\n", frame);
    va_end(strings);
}

void print_framed_int_value(int value, const char* str){
    const char* frame = "#############################################";
    printf("%s\n", frame);
    printf("%s: %d\n", str, value);
    printf("%s\n", frame);
}

PROCESS_THREAD(monitor_process, ev, data){
    static enum monitor_message mon_msg;
    PROCESS_BEGIN();

    while(true){
        PROCESS_WAIT_EVENT();
        if (ev == update_monitor_ev) {
            mon_msg = (enum monitor_message) data;
            switch (mon_msg){
                case PRINT_ENTRANCE_CLOSED:
                    print_framed(1, "Entrance has been CLOSED");
                    break;

                case PRINT_ISSUED_COMMAND:
                    print_framed_int_value((int) cmd_issued,
                                                "Command issued");
                    break;

                case PRINT_MENU:
                    printf("\nAvailable commands are:\n");
                    printf("1. %s alarm signal\n", alarm_state == DISABLED ? "Turn ON" : "Turn OFF");
                    if (alarm_state == DISABLED){
                        if (entrance_state == CLOSED){
                            printf("2. %s the gate\n", (gate_lock_state == UNLOCKED) ? "LOCK" : "UNLOCK");
                            printf("%s\n", "3. OPEN and CLOSE door and gate");
                        }
                        printf("4. Average internal temperature of the last 50 seconds\n");
                        printf("5. External light value\n");
                    }
                    break;

                case PRINT_TEMP:
                    print_framed_int_value(temperature,
                                           "Average temperature of last 50 seconds:");
                    break;

                case PRINT_ALARM_ENABLING:
                    print_framed(2, "Alarm is enabling on nodes.",
                                    "Wait the entrance to close");
                    break;

                case PRINT_LOCKING_GATE:
                    print_framed(1, "Gate is locking. Wait for it to close.");
                    break;

                case PRINT_LIGHT:
                    print_framed_int_value(light, "Light measure:");
                    break;

                case PRINT_WAIT_CLOSE:
                    print_framed(2, "Wait door and/or gate to close",
                                    "Then issue this command again");
                    break;

                case PRINT_WAIT_TEMP:
                    print_framed(2, "Please wait a minute for the node",
                                "to collect enough samples");
                    break;

                case PRINT_FULL_QUEUE:
                    print_framed(1, "Too many command issued");
                    break;

                case PRINT_ALARM_ACTIVE:
                    print_framed(1, "ALARM IS ACTIVE");
                    break;

                case PRINT_UNLOCK_GATE:
                    print_framed(1, "Unlock the gate first");
                    break;

                case PRINT_ALARM_DISABLED:
                    print_framed(1, "ALARM HAS BEEN DISABLED");
                    break;

                case PRINT_COMMAND_NOT_VALID:
                    print_framed(1, "UNKNOWN OR INVALID COMMAND");
                    break;

                case PRINT_LOCKED_GATE:
                    print_framed(1, "Gate is LOCKED");
                    break;

                case PRINT_ENTRANCE_OPEN:
                    print_framed(1, "Entrance is OPENING");
                    break;

                case PRINT_LIGHT_REQUESTED:
                    print_framed(1, "Light requested");
                    break;

                default:
                    printf("%s: Error. Monitor command unrecognized", __func__);
                    break;
            }
        }
    }
    PROCESS_END();
    return 0;
}
