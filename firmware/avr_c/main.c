/*
 * Bare-Metal AVR-C — Line Follower, Obstacle Detection & Fire Extinguisher Robot
 * MCU: ATmega328P @ 16 MHz (Arduino Nano V3)
 * Timers: Timer0 = Motor PWM | Timer1 = Non-Blocking Ultrasonic (0.5 us/tick) | Timer2 = 1 ms Tick
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

// ======================== CONFIGURATION ========================
#define USE_ULTRASONIC_OBS   1      // 1 = HC-SR04 (Trig PD4, Echo PD2); 0 = 3-pin IR on PD2
#define LINE_IS_HIGH         1      // 1 = black line reads > IR_THRESHOLD; 0 = < IR_THRESHOLD
#define EXT_ACTIVE_LOW       0      // 0 = Logic-Level N-MOSFET (Active-HIGH); 1 = Active-LOW Relay

#define IR_THRESHOLD         512
#define FLAME_ANALOG_TRIG    400
#define FLAME_ADC_SAMPLES    8
#define OBS_STOP_CM          15.0f

#define OBS_INTERVAL_MS      60UL
#define FLAME_DEBOUNCE_MS    75UL
#define EXT_HOLD_MS          1000UL

#define BASE_SPEED           140
#define MAX_SPEED            220

// ======================== PIN BITMASKS ========================
// L298N Motor Driver
#define ENA_PIN   PD5   // OC0B (Left Speed PWM)
#define ENB_PIN   PD6   // OC0A (Right Speed PWM)
#define IN1_PIN   PD7   // Left Dir A
#define IN2_PIN   PB0   // Left Dir B
#define IN3_PIN   PB1   // Right Dir A
#define IN4_PIN   PB2   // Right Dir B

// Obstacle & Flame Sensors
#define OBS_ECHO_PIN   PD2   // INT0 (Echo or 3-pin IR OUT)
#define OBS_TRIG_PIN   PD4   // HC-SR04 Trigger
#define FLAME_DO_PIN   PD3   // INT1 (Active-LOW)

// Extinguisher & Status LED
#define EXT_PIN        PB4   // D12 (MOSFET Gate / Relay IN)
#define LED_PIN        PB5   // D13

// ADC Channels
static const uint8_t IR_CHANNELS[5] = {0, 1, 2, 3, 6}; // A0, A1, A2, A3, A6
static const int8_t  IR_WEIGHTS[5]  = {-2, -1, 0, 1, 2};
#define FLAME_ADC_CH   7                               // A7

// ======================== PID & RUNTIME STATE ========================
static const float Kp = 35.0f;
static const float Ki = 0.0f;
static const float Kd = 18.0f;

static float last_error = 0.0f;
static float integral   = 0.0f;

static volatile uint32_t sys_millis        = 0;
static volatile uint16_t echo_start_ticks  = 0;
static volatile uint16_t echo_width_ticks  = 0;
static volatile bool     echo_new_data     = false;
static volatile bool     ir_obs_irq_flag   = false;

static volatile bool     flame_irq_flag    = false;
static volatile uint32_t last_flame_irq_ms = 0;

// ======================== INTERRUPTS ========================
// Timer2 Compare Match A — 1 ms system tick
ISR(TIMER2_COMPA_vect) {
    sys_millis++;
}

// INT0 (PD2) — Non-blocking HC-SR04 rising/falling edge OR 3-pin IR falling edge
ISR(INT0_vect) {
#if USE_ULTRASONIC_OBS
    if (PIND & (1 << OBS_ECHO_PIN)) {
        echo_start_ticks = TCNT1;
    } else {
        echo_width_ticks = TCNT1 - echo_start_ticks;
        echo_new_data = true;
    }
#else
    ir_obs_irq_flag = true;
#endif
}

// INT1 (PD3) — Active-LOW flame trigger with software debounce
ISR(INT1_vect) {
    uint32_t now = sys_millis;
    if ((now - last_flame_irq_ms) >= FLAME_DEBOUNCE_MS) {
        flame_irq_flag = true;
        last_flame_irq_ms = now;
    }
}

static uint32_t get_millis(void) {
    uint32_t ms;
    cli();
    ms = sys_millis;
    sei();
    return ms;
}

// ======================== HARDWARE INIT ========================
static void gpio_init(void) {
    // Outputs on PORTD: ENA(PD5), ENB(PD6), IN1(PD7)
    DDRD |= (1 << ENA_PIN) | (1 << ENB_PIN) | (1 << IN1_PIN);
#if USE_ULTRASONIC_OBS
    DDRD  |= (1 << OBS_TRIG_PIN);
    PORTD &= ~(1 << OBS_TRIG_PIN);
#endif
    // Inputs on PORTD: OBS_ECHO(PD2), FLAME_DO(PD3)
    DDRD &= ~((1 << OBS_ECHO_PIN) | (1 << FLAME_DO_PIN));

    // Outputs on PORTB: IN2(PB0), IN3(PB1), IN4(PB2), EXT(PB4), LED(PB5)
    DDRB |= (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN) | (1 << EXT_PIN) | (1 << LED_PIN);
}

static void timer0_pwm_init(void) {
    // Fast PWM, Non-Inverting on OC0A (PD6/ENB) and OC0B (PD5/ENA), Prescaler 64 (~976 Hz)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 0;
    OCR0B = 0;
}

static void timer1_ultrasonic_init(void) {
    // Free-running 16-bit timer, Prescaler = 8 -> 2 MHz (0.5 us per tick)
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
}

static void timer2_millis_init(void) {
    // CTC mode, Prescaler = 64 -> 250 kHz; OCR2A = 249 -> 1 kHz (1 ms)
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A  = 249;
    TIMSK2 = (1 << OCIE2A);
}

static void adc_init(void) {
    ADMUX  = (1 << REFS0);                                         // AVcc = 5V
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 125 kHz
}

static void ext_int_init(void) {
#if USE_ULTRASONIC_OBS
    // INT0 (PD2): Any logical change (ISC01 = 0, ISC00 = 1) for rising + falling edges
    EICRA = (EICRA | (1 << ISC00)) & ~(1 << ISC01);
#else
    // INT0 (PD2): Falling edge (ISC01 = 1, ISC00 = 0) for Active-LOW 3-pin IR
    EICRA = (EICRA | (1 << ISC01)) & ~(1 << ISC00);
#endif
    // INT1 (PD3): Falling edge (ISC11 = 1, ISC10 = 0) for Active-LOW flame alert
    EICRA = (EICRA | (1 << ISC11)) & ~(1 << ISC10);

    EIMSK |= (1 << INT0) | (1 << INT1);
}

static uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// ======================== ACTUATOR CONTROL ========================
static void set_extinguisher(bool on) {
#if EXT_ACTIVE_LOW
    if (on) PORTB &= ~(1 << EXT_PIN);
    else    PORTB |=  (1 << EXT_PIN);
#else
    if (on) PORTB |=  (1 << EXT_PIN);
    else    PORTB &= ~(1 << EXT_PIN);
#endif
}

static void stop_motors(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB &= ~((1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN));
    OCR0B = 0; // ENA
    OCR0A = 0; // ENB
}

static void set_motors(int16_t left_speed, int16_t right_speed) {
    if (left_speed >= 0) {
        PORTD |=  (1 << IN1_PIN);
        PORTB &= ~(1 << IN2_PIN);
        OCR0B = (uint8_t)left_speed;
    } else {
        PORTD &= ~(1 << IN1_PIN);
        PORTB |=  (1 << IN2_PIN);
        OCR0B = (uint8_t)(-left_speed);
    }

    if (right_speed >= 0) {
        PORTB |=  (1 << IN3_PIN);
        PORTB &= ~(1 << IN4_PIN);
        OCR0A = (uint8_t)right_speed;
    } else {
        PORTB &= ~(1 << IN3_PIN);
        PORTB |=  (1 << IN4_PIN);
        OCR0A = (uint8_t)(-right_speed);
    }
}

// ======================== SENSOR LOGIC ========================
static uint16_t read_flame_adc_avg(void) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < FLAME_ADC_SAMPLES; i++) {
        sum += adc_read(FLAME_ADC_CH);
    }
    return (uint16_t)(sum / FLAME_ADC_SAMPLES);
}

static bool is_fire_detected(void) {
    bool digital_fire   = ((PIND & (1 << FLAME_DO_PIN)) == 0);
    uint16_t analog_avg = read_flame_adc_avg();
    bool analog_fire    = (analog_avg < FLAME_ANALOG_TRIG);

    bool confirmed = (digital_fire && analog_fire) || (analog_avg < (FLAME_ANALOG_TRIG - 80));

    if (flame_irq_flag) {
        flame_irq_flag = false;
        if (digital_fire || analog_fire) confirmed = true;
    }
    return confirmed;
}

static void update_obstacle_state(uint32_t now, uint32_t *last_ping_ms, bool *obs_present) {
#if USE_ULTRASONIC_OBS
    if ((now - *last_ping_ms) >= OBS_INTERVAL_MS) {
        *last_ping_ms = now;
        PORTD &= ~(1 << OBS_TRIG_PIN);
        _delay_us(2);
        PORTD |= (1 << OBS_TRIG_PIN);
        _delay_us(10);
        PORTD &= ~(1 << OBS_TRIG_PIN);
    }

    if (echo_new_data) {
        cli();
        uint16_t ticks = echo_width_ticks;
        echo_new_data = false;
        sei();

        // 30,000 ticks at 0.5 us/tick = 15 ms (~2.5 m timeout)
        if (ticks == 0 || ticks > 30000) {
            *obs_present = false;
        } else {
            float dist_cm = (float)ticks / 116.0f; // (ticks * 0.5 us) / 58.0
            *obs_present = (dist_cm > 0.5f && dist_cm <= OBS_STOP_CM);
        }
    }
#else
    bool pin_low = ((PIND & (1 << OBS_ECHO_PIN)) == 0);
    *obs_present = pin_low || ir_obs_irq_flag;
    if (!pin_low) ir_obs_irq_flag = false;
#endif
}

static int16_t clamp_i16(int16_t val, int16_t min_v, int16_t max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static void follow_line_pid(void) {
    int8_t active_count = 0;
    int8_t weighted_sum = 0;

    for (uint8_t i = 0; i < 5; i++) {
        uint16_t raw = adc_read(IR_CHANNELS[i]);
#if LINE_IS_HIGH
        bool on_line = (raw > IR_THRESHOLD);
#else
        bool on_line = (raw < IR_THRESHOLD);
#endif
        if (on_line) {
            weighted_sum += IR_WEIGHTS[i];
            active_count++;
        }
    }

    float error = (active_count > 0)
                    ? ((float)weighted_sum / (float)active_count)
                    : ((last_error >= 0.0f) ? 2.5f : -2.5f);

    integral += error;
    if (integral > 50.0f)  integral = 50.0f;
    if (integral < -50.0f) integral = -50.0f;

    float derivative = error - last_error;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    last_error = error;

    int16_t left_spd  = clamp_i16((int16_t)(BASE_SPEED + correction), -MAX_SPEED, MAX_SPEED);
    int16_t right_spd = clamp_i16((int16_t)(BASE_SPEED - correction), -MAX_SPEED, MAX_SPEED);

    set_motors(left_spd, right_spd);
}

// ======================== MAIN ========================
int main(void) {
    gpio_init();
    timer0_pwm_init();
    timer1_ultrasonic_init();
    timer2_millis_init();
    adc_init();
    ext_int_init();

    stop_motors();
    set_extinguisher(false);
    sei();

    uint32_t last_obs_ping_ms   = 0;
    uint32_t ext_hold_until_ms  = 0;
    bool obstacle_present       = false;

    while (1) {
        uint32_t now = get_millis();

        // PRIORITY 1: Fire Detection & Extinguisher Hold
        if (is_fire_detected()) {
            ext_hold_until_ms = now + EXT_HOLD_MS;
        }

        if (now < ext_hold_until_ms) {
            stop_motors();
            set_extinguisher(true);
            PORTB |= (1 << LED_PIN);
            continue;
        } else {
            set_extinguisher(false);
            PORTB &= ~(1 << LED_PIN);
        }

        // PRIORITY 2: Non-Blocking Obstacle Detection
        update_obstacle_state(now, &last_obs_ping_ms, &obstacle_present);
        if (obstacle_present) {
            stop_motors();
            continue;
        }

        // PRIORITY 3: 5-IR PID Line Following
        follow_line_pid();
    }
}