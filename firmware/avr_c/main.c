/*
 * Bare-Metal AVR-C — Line Follower, Obstacle & Fire Detection Robot
 * MCU: ATmega328P @ 16 MHz (Arduino Nano V3)
 * Toolchain: avr-gcc / Microchip Studio / PlatformIO (bare-metal)
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
#define RELAY_ACTIVE_LOW     1      // 1 = active-LOW relay module; 0 = active-HIGH / MOSFET

#define IR_THRESHOLD         512
#define FLAME_ANALOG_TRIG    400
#define OBS_STOP_CM          15.0f
#define OBS_INTERVAL_MS      60UL

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
#define OBS_ECHO_PIN   PD2   // INT0 / Echo (or IR OUT)
#define OBS_TRIG_PIN   PD4   // HC-SR04 Trigger
#define FLAME_DO_PIN   PD3   // INT1 (Active-LOW)

// Optional Extinguisher & Heartbeat LED
#define EXT_PIN        PB4   // D12
#define LED_PIN        PB5   // D13

// ADC Channels
static const uint8_t IR_CHANNELS[5] = {0, 1, 2, 3, 6}; // A0, A1, A2, A3, A6
static const int8_t  IR_WEIGHTS[5]  = {-2, -1, 0, 1, 2};
#define FLAME_ADC_CH   7                               // A7

// ======================== PID & STATE ========================
static const float Kp = 35.0f;
static const float Ki = 0.0f;
static const float Kd = 18.0f;

static float last_error = 0.0f;
static float integral   = 0.0f;

static volatile uint32_t sys_millis = 0;
static volatile bool flame_irq_flag = false;

// ======================== INTERRUPTS ========================
// Timer2 Compare Match A — 1 ms system tick
ISR(TIMER2_COMPA_vect) {
    sys_millis++;
}

// External Interrupt 1 (PD3 / FLAME_DO) — Falling edge
ISR(INT1_vect) {
    flame_irq_flag = true;
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
    // Outputs on PORTD: ENA(PD5), ENB(PD6), IN1(PD7), OBS_TRIG(PD4)
    DDRD |= (1 << ENA_PIN) | (1 << ENB_PIN) | (1 << IN1_PIN) | (1 << OBS_TRIG_PIN);
    // Inputs on PORTD: OBS_ECHO(PD2), FLAME_DO(PD3)
    DDRD &= ~((1 << OBS_ECHO_PIN) | (1 << FLAME_DO_PIN));

    // Outputs on PORTB: IN2(PB0), IN3(PB1), IN4(PB2), EXT(PB4), LED(PB5)
    DDRB |= (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN) | (1 << EXT_PIN) | (1 << LED_PIN);

    // Ensure TRIG starts LOW
    PORTD &= ~(1 << OBS_TRIG_PIN);
}

static void timer0_pwm_init(void) {
    // Fast PWM, Non-Inverting on OC0A (PD6/ENB) and OC0B (PD5/ENA)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    // Prescaler = 64 -> 16 MHz / (64 * 256) = 976.56 Hz
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 0;
    OCR0B = 0;
}

static void timer1_ultrasonic_init(void) {
    // Normal mode, Prescaler = 8 -> 2 MHz (0.5 us per tick)
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
    // Reference = AVcc (5V), right-adjusted
    ADMUX = (1 << REFS0);
    // Enable ADC, Prescaler = 128 -> 125 kHz ADC clock
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static void ext_int_init(void) {
    // INT1 (PD3) triggers on falling edge (ISC11 = 1, ISC10 = 0)
    EICRA = (EICRA | (1 << ISC11)) & ~(1 << ISC10);
    EIMSK |= (1 << INT1);
}

static uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// ======================== ACTUATOR CONTROL ========================
static void set_extinguisher(bool on) {
#if RELAY_ACTIVE_LOW
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
    // Left Motor: IN1 (PD7), IN2 (PB0), ENA (OCR0B)
    if (left_speed >= 0) {
        PORTD |=  (1 << IN1_PIN);
        PORTB &= ~(1 << IN2_PIN);
        OCR0B = (uint8_t)left_speed;
    } else {
        PORTD &= ~(1 << IN1_PIN);
        PORTB |=  (1 << IN2_PIN);
        OCR0B = (uint8_t)(-left_speed);
    }

    // Right Motor: IN3 (PB1), IN4 (PB2), ENB (OCR0A)
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
static bool is_fire_detected(void) {
    bool digital_fire = ((PIND & (1 << FLAME_DO_PIN)) == 0);
    bool analog_fire  = (adc_read(FLAME_ADC_CH) < FLAME_ANALOG_TRIG);

    if (!digital_fire && !analog_fire) {
        flame_irq_flag = false;
        return false;
    }
    return (flame_irq_flag || digital_fire || analog_fire);
}

static bool check_obstacle(void) {
#if USE_ULTRASONIC_OBS
    // Send 10 us trigger pulse on PD4
    PORTD &= ~(1 << OBS_TRIG_PIN);
    _delay_us(2);
    PORTD |= (1 << OBS_TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << OBS_TRIG_PIN);

    // Wait for Echo HIGH (timeout ~15 ms = 30,000 ticks at 0.5 us/tick)
    TCNT1 = 0;
    while (!(PIND & (1 << OBS_ECHO_PIN))) {
        if (TCNT1 > 30000) return false;
    }

    // Measure Echo HIGH duration
    TCNT1 = 0;
    while (PIND & (1 << OBS_ECHO_PIN)) {
        if (TCNT1 > 30000) return false;
    }
    uint16_t ticks = TCNT1;

    // Distance (cm) = (ticks * 0.5 us) / 58.0 = ticks / 116.0
    float dist_cm = (float)ticks / 116.0f;
    return (dist_cm > 0.5f && dist_cm <= OBS_STOP_CM);
#else
    // Standard 3-pin IR obstacle sensor outputs LOW when obstacle detected
    return ((PIND & (1 << OBS_ECHO_PIN)) == 0);
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

    float error = 0.0f;
    if (active_count > 0) {
        error = (float)weighted_sum / (float)active_count;
    } else {
        error = (last_error >= 0.0f) ? 2.5f : -2.5f;
    }

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
    sei(); // Enable global interrupts

    uint32_t last_obs_ms = 0;
    bool obstacle_present = false;

    while (1) {
        // PRIORITY 1: Fire / Flame Detection
        if (is_fire_detected()) {
            stop_motors();
            set_extinguisher(true);
            PORTB |= (1 << LED_PIN);
            _delay_ms(100);
            continue;
        } else {
            set_extinguisher(false);
            PORTB &= ~(1 << LED_PIN);
            flame_irq_flag = false;
        }

        // PRIORITY 2: Obstacle Detection (polled every 60 ms)
        uint32_t now = get_millis();
        if ((now - last_obs_ms) >= OBS_INTERVAL_MS) {
            last_obs_ms = now;
            obstacle_present = check_obstacle();
        }

        if (obstacle_present) {
            stop_motors();
            continue;
        }

        // PRIORITY 3: 5-IR PID Line Following
        follow_line_pid();
    }
}