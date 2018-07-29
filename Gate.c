#include "nesproj.h"
#include "dev/light-sensor.h"
#include "sys/timer.h"

// Custom event enqueued for this node
static process_event_t message_from_cu;
static process_event_t alarm_event;
static process_event_t start_opening;
static process_event_t end_opening;
static process_event_t send_msg;
static process_event_t lock_unlock_ev;
static process_event_t get_light;

// Node state
enum lock_state lock_state;
enum alarm_state alarm_state;
enum entrance_state gate_state;

linkaddr_t gate_addr = {{GATE_ADDR_0, GATE_ADDR_1}};

PROCESS(msg_process, "Gate Node Message Manager Process");
PROCESS(alarm_process, "Gate Node Alarm Process");
PROCESS(openclose_process, "Gate Node Opening Process");
PROCESS(main_process, "Gate Main Process");

// Missing processes have to be spawned by other ones
AUTOSTART_PROCESSES(&msg_process, &main_process);

// For rime communication
static struct broadcast_conn broadcast;
static struct runicast_conn runicast;

// Callbacks for Rime to work
static void broadcast_recv (struct broadcast_conn *c, const linkaddr_t *from) {
    // Ignores messages from any node except for CU
	if (from->u8[0] == CU_ADDR_0 && from->u8[1] == CU_ADDR_1) {
		process_post(&msg_process, message_from_cu, packetbuf_dataptr());
	}
}

static void recv_runicast (struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
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

void set_leds (){
    if (lock_state == LOCKED){
        leds_on(LEDS_RED);
        leds_off(LEDS_GREEN);
    }
    else {
        leds_on(LEDS_GREEN);
        leds_off(LEDS_RED);
    }
}

uint8_t msg2cu (msg_t *msg){
    if (!runicast_is_transmitting(&runicast)){
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

PROCESS_THREAD(main_process, ev, data){
    static msg_t msg;
    static int32_t light_value;
    static struct etimer sensor_timer;
    PROCESS_BEGIN();

    // Init
    linkaddr_set_node_addr(&gate_addr);
    alarm_event = process_alloc_event();
    start_opening = process_alloc_event();
    end_opening = process_alloc_event();
    send_msg = process_alloc_event();
    alarm_state = DISABLED;
    gate_state = CLOSED;
    lock_state = UNLOCKED;
    set_leds();

    while (true){
        PROCESS_WAIT_EVENT();
        if (ev == alarm_event){
            msg = get_message_from(data);
            if (gate_state == MOVING){
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
        if (ev == start_opening && gate_state == CLOSED &&
                                   lock_state == UNLOCKED &&
                                   alarm_state == DISABLED) {
                gate_state = MOVING;
                process_start(&openclose_process, NULL);
        }
        if (ev == end_opening){
            gate_state = CLOSED;
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
        if (ev == get_light){
            // Need to wait a while for the sensor to initialize
            SENSORS_ACTIVATE(light_sensor);
            etimer_set(&sensor_timer, CLOCK_SECOND/10);
        }
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&sensor_timer)){
            // Sample the light and send it
            light_value = 10*light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/7;
            SENSORS_DEACTIVATE(light_sensor);
            msg.hdr = LIGHT_MSG;
            msg.payload = light_value;
            process_post(&msg_process, send_msg, (void*) &msg);
        }
        if (ev == lock_unlock_ev && gate_state == CLOSED){
            lock_state = (lock_state == LOCKED) ? UNLOCKED : LOCKED;
            set_leds();
        }
    }

    PROCESS_END();
    return 0;
}

PROCESS_THREAD(openclose_process, ev, data){
    static struct etimer blink_timer;
    static struct timer blink_period;

    PROCESS_BEGIN();

    end_opening = process_alloc_event();
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

PROCESS_THREAD(msg_process, ev, data){
    static msg_t msg;
    static struct timer free_radio;

    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_EXITHANDLER(runicast_close(&runicast);)
    PROCESS_BEGIN();

    // Init
    alarm_event = process_alloc_event();
    start_opening = process_alloc_event();
    lock_unlock_ev = process_alloc_event();
    broadcast_open(&broadcast, BC_CH, &broadcast_call);
    runicast_open(&runicast, RU_CH, &runicast_calls);
    linkaddr_set_node_addr(&gate_addr);

    while(true){
        PROCESS_WAIT_EVENT();
        msg = get_message_from(data);
        if (ev == send_msg){
            // Can't define a stand alone function due to Contiki restriction
            // on using PROCESS_WAIT_UNTIL macro
            // Wait a quarter of second before retry
            timer_set(&free_radio, CLOCK_SECOND >> 2);
            while(msg2cu(&msg) == 1){
                PROCESS_WAIT_UNTIL(timer_expired(&free_radio));
                timer_restart(&free_radio);
            }
        }
        if (ev == message_from_cu){
            if (msg.hdr == CMD_MSG){
                switch (msg.payload){
                    case ALARM_ENABLED:
                    case ALARM_DISABLED:
                        process_post(&main_process, alarm_event, (void*) &msg);
                        break;

                    case ENTRANCE_OPEN:
                        process_post(&main_process, start_opening, NULL);
                        break;

                    case GATE_LOCK:
                    case GATE_UNLOCK:
                        process_post(&main_process, lock_unlock_ev, NULL);
                        break;

                    case GET_LIGHT:
                        process_post(&main_process, get_light, NULL);
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
