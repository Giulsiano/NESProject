#include "nesproj.h"
#include "dev/sht11/sht11-sensor.h"
#include "stdint.h"
#include "sys/timer.h"

// Sampling temperature period
#ifndef SMPL_TEMP_PERIOD
#define SMPL_TEMP_PERIOD    CLOCK_SECOND*10
#endif

// How many temperature sample to get from the sensor
#define SMPL_NUM    5

static process_event_t message_from_cu;
static process_event_t alarm_event;
static process_event_t start_opening;
static process_event_t end_opening;
static process_event_t send_msg;
static process_event_t get_temp;
static process_event_t toggle_light;

enum entrance_state door_state;
enum alarm_state alarm_state;
enum onoff_state previous_light_state;
enum onoff_state light_state;

// Process declarations
PROCESS(msg_process, "Door Node Message Manager Process");
PROCESS(alarm_process, "Door Node Alarm Process");
PROCESS(openclose_process, "Door Node Opening Process");
PROCESS(temp_process, "Door Temperature Sampling Process");
PROCESS(button_process, "Door Node Button Process");
PROCESS(main_process, "Door Main Process");

// Missing processes have to be spawned by other ones
AUTOSTART_PROCESSES(&msg_process, &temp_process, &button_process, &main_process);

// Circular array where to store temperature samples
#define CQUEUE_LEN  SMPL_NUM
int cqueue[CQUEUE_LEN];
uint8_t cqueue_idx = 0;
uint32_t sample_num = 0;
bool queue_filled = false;

// Address of this node
linkaddr_t door_addr = {{DOOR_ADDR_0, DOOR_ADDR_1}};

// Callbacks for Rime to work
static void broadcast_recv (struct broadcast_conn *c, const linkaddr_t *from) {
    // Ignores messages from any node except for CU
	if (from->u8[0] == CU_ADDR_0 && from->u8[1] == CU_ADDR_1) {
		process_post(&msg_process, message_from_cu, packetbuf_dataptr());
	}
}

static void recv_runicast (struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
    // Ignores messages from any node except for CU
    if(from->u8[0] == CU_ADDR_0 && from->u8[1] == CU_ADDR_1) {
		process_post(&msg_process, message_from_cu, packetbuf_dataptr());
	}
}

static void sent_runicast (struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
    // Do nothing
}

static void timedout_runicast (struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
    // Do nothing
}

// Data structure for the rime communication primitives
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct runicast_callbacks runicast_calls = {recv_runicast,
                                                         sent_runicast,
                                                         timedout_runicast
                                                     };
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;

// Functions to manage the queue
void cqueue_init (){
    uint8_t i;
    for (i = 0; i < CQUEUE_LEN; ++i){
        cqueue[i] = INT_MIN;
    }
}

void cqueue_insert (int v){
    cqueue[cqueue_idx] = v;
    cqueue_idx = (cqueue_idx + 1) % SMPL_NUM;
    if (++sample_num > SMPL_NUM){
        queue_filled = true;
    }
}

int get_avg_temp (){
    int sum = 0;
    if (queue_filled == true){
        uint8_t i;
        for (i = 0; i < SMPL_NUM; ++i){
            sum += cqueue[i];
        }
    }
    else {
        return INT_MIN;
    }
    return sum/SMPL_NUM;
}

uint8_t msg2cu (msg_t *msg){
    if(!runicast_is_transmitting(&runicast)) {
		linkaddr_t recv;
		packetbuf_copyfrom((void*) msg, sizeof(msg_t));
		recv.u8[0] = CU_ADDR_0;
		recv.u8[1] = CU_ADDR_1;
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
	}
    else {
        return 1;
    }
    return 0;
}

// set the status of the led by using current node status
void set_leds (){
    if (light_state == OFF){
        leds_on(LEDS_RED);
        leds_off(LEDS_GREEN);
    }
    else {
        leds_on(LEDS_GREEN);
        leds_off(LEDS_RED);
    }
}

PROCESS_THREAD(main_process, ev, data){
    PROCESS_BEGIN();
    static msg_t msg;

    // Init
    linkaddr_set_node_addr(&door_addr);
    alarm_state = DISABLED;
    light_state = OFF;
    previous_light_state = OFF;
    door_state = OFF;
    send_msg = process_alloc_event();
    set_leds();

    while (true){
        PROCESS_WAIT_EVENT();
        if (ev == sensors_event && data == &button_sensor){
            previous_light_state = light_state;
            light_state = (light_state == OFF) ? ON : OFF;
            set_leds();
        }
        if (ev == alarm_event){
            msg = get_message_from(data);
            if (door_state == MOVING){
                if (msg.payload == ALARM_DISABLED){
                    alarm_state = DISABLED;
                }
                else {
                    // it can be only ALARM_ENABLED
                    alarm_state = ENABLING;
                    msg.payload = ALARM_ENABLING;
                }
            }
            switch (alarm_state) {
                case DISABLED:
                    process_start(&alarm_process, NULL);
                    msg.payload = ALARM_ENABLED;
                    alarm_state = ENABLED;
                    break;

                case ENABLED:
                    process_exit(&alarm_process);
                    msg.payload = ALARM_DISABLED;
                    alarm_state = DISABLED;
                    leds_off(LEDS_ALL);
                    set_leds();
                    break;

                case ENABLING:
                    // There is no need to send a confirm to the cu because
                    // it is sent after the alarm is enabled after the door is
                    // close
                    break;

                default:
                    printf("%s: Error. Unrecognized payload: %d", __func__, msg.payload);
                    break;
            }
            process_post(&msg_process, send_msg, (void*) &msg);
        }
        if (ev == start_opening && alarm_state == DISABLED && door_state == CLOSED){
            door_state = MOVING;
            process_start(&openclose_process, NULL);
        }
        if (ev == end_opening){
            // Send the message
            door_state = CLOSED;
            msg.hdr = CMD_MSG;
            msg.payload = ENTRANCE_CLOSE;
            process_post(&msg_process, send_msg, (void*) &msg);
            if (alarm_state == ENABLING){
                alarm_state = ENABLED;
                msg.payload = ALARM_ENABLED;
                process_start(&alarm_process, NULL);
                process_post(&msg_process, send_msg, (void*) &msg);
            }
        }
        if (ev == get_temp){
            msg.hdr = TEMP_MSG;
            msg.payload = get_avg_temp();
            process_post(&msg_process, send_msg, (void*) &msg);
        }
    }

    PROCESS_END();
    return 0;
}

PROCESS_THREAD(openclose_process, ev, data){
    static struct etimer blink_timer;
    static struct etimer wait_guest;
    static struct timer blink_period;

    PROCESS_BEGIN();

    end_opening = process_alloc_event();
    etimer_set(&wait_guest, CLOCK_SECOND*14);
    while (true){
        PROCESS_WAIT_EVENT();
        if (etimer_expired(&wait_guest)){
            break;
        }
    }
    timer_set(&blink_period, CLOCK_SECOND*16);
    leds_on(LEDS_BLUE);
    etimer_set(&blink_timer, BLINK_PERIOD);
    do {
        PROCESS_WAIT_EVENT();
        if (etimer_expired(&blink_timer)){
            etimer_restart(&blink_timer);
            leds_toggle(LEDS_BLUE);
        }
    } while (timer_expired(&blink_period) == 0);
    leds_off(LEDS_BLUE);
    process_post(&main_process, end_opening, NULL);
    PROCESS_END();
    return 0;
}

PROCESS_THREAD(alarm_process, ev, data){
    static struct etimer blink_period;

    PROCESS_BEGIN();
    leds_off(LEDS_ALL);
    etimer_set(&blink_period, BLINK_PERIOD);
    while (true) {
        leds_toggle(LEDS_ALL);
        PROCESS_WAIT_EVENT();
        if (etimer_expired(&blink_period)){
            etimer_restart(&blink_period);
        }
    }

    PROCESS_END();
    return 0;
}

PROCESS_THREAD(button_process, ev, data){
	PROCESS_BEGIN();

    toggle_light = process_alloc_event();
	SENSORS_ACTIVATE(button_sensor);

	while(true) {
		PROCESS_WAIT_EVENT();
        if (ev == sensors_event && data == &button_sensor){
            process_post(&main_process, toggle_light, NULL);
        }
	}

	SENSORS_DEACTIVATE(button_sensor);
	PROCESS_END();
	return 0;
}

PROCESS_THREAD(temp_process, ev, data){
	PROCESS_BEGIN();

	static struct etimer sample_timer;
	etimer_set(&sample_timer, SMPL_TEMP_PERIOD);

	while(true) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sample_timer));
			SENSORS_ACTIVATE(sht11_sensor);
			cqueue_insert(((sht11_sensor.value(SHT11_SENSOR_TEMP) / 10 - 396) / 10));
			SENSORS_DEACTIVATE(sht11_sensor);
			etimer_reset(&sample_timer);
	}
	PROCESS_END();
	return 0;
}

PROCESS_THREAD(msg_process, ev, data){
    static struct etimer free_radio;
    static msg_t msg;

    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_EXITHANDLER(runicast_close(&runicast);)
    PROCESS_BEGIN();

    // Init
    alarm_event = process_alloc_event();
    start_opening = process_alloc_event();
    get_temp = process_alloc_event();
    message_from_cu = process_alloc_event();
    broadcast_open(&broadcast, BC_CH, &broadcast_call);
    runicast_open(&runicast, RU_CH, &runicast_calls);
    linkaddr_set_node_addr(&door_addr);

    while(true){
        PROCESS_WAIT_EVENT();
        if (ev == send_msg){
            msg = get_message_from(data);

            // Can't define a stand alone function due to Contiki restriction
            // on using PROCESS_WAIT_UNTIL macro
            while(msg2cu(&msg) == 1){
                // Wait a quarter of second before retry
                etimer_set(&free_radio, CLOCK_SECOND >> 2);
                PROCESS_WAIT_UNTIL(etimer_expired(&free_radio));
                etimer_stop(&free_radio);
            }
        }
        if (ev == message_from_cu){
            msg = get_message_from(data);
            if (msg.hdr == CMD_MSG){
                switch (msg.payload){
                    case ALARM_DISABLED:
                    case ALARM_ENABLED:
                        process_post(&main_process, alarm_event, (void*) &msg);
                        break;

                    case ENTRANCE_OPEN:
                        process_post(&main_process, start_opening, NULL);
                        break;

                    case GET_TEMP:
                        process_post(&main_process, get_temp, NULL);
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
