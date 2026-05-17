/*
 * tea_factory_strict_embedded_c.c
 * Target: ATmega328P (Arduino UNO R3)
 * Toolchain: avr-gcc + avrdude
 *
 * Strict Embedded C (register-level), no Arduino framework APIs.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/* ==================== Pin Mapping ====================
 * D2  (PD2) -> Flame sensor digital output (active LOW)
 * D3  (PD3) -> DHT22 data
 * D6  (PD6) -> Relay FAN
 * D7  (PD7) -> Relay PUMP
 * D8  (PB0) -> Active buzzer
 * D9  (PB1) -> Green LED
 * D10 (PB2) -> Yellow LED
 * D11 (PB3) -> Red LED
 * A0  (PC0) -> MQ-2 analog output
 * A4  (PC4) -> I2C SDA (MPU6050 + LCD I2C)
 * A5  (PC5) -> I2C SCL (MPU6050 + LCD I2C)
 * D0/D1      -> Hardware UART for SIM800L (recommended for strict C)
 */

/* ==================== Thresholds ==================== */
#define GAS_THRESHOLD            150U
#define TEMP_THRESHOLD_X10       350   /* 35.0C */
#define VIBRATION_THRESHOLD      15000L

/* ==================== Timing ==================== */
#define LOOP_INTERVAL_MS         500UL
#define SMS_INTERVAL_MS          60000UL
#define LED_BLINK_FAST_MS        200UL
#define LED_BLINK_SLOW_MS        500UL
#define LCD_ALERT_BLINK_MS       500UL

/* ==================== Addresses ==================== */
#define MPU6050_ADDR             0x68
#define LCD_I2C_ADDR             0x27

/* ==================== LCD PCF8574 Mapping ====================
 * P0=RS P1=RW P2=EN P3=BL P4=D4 P5=D5 P6=D6 P7=D7
 */
#define LCD_RS                   0x01
#define LCD_RW                   0x02
#define LCD_EN                   0x04
#define LCD_BL                   0x08

/* ==================== Hazard Mask ==================== */
#define HAZARD_NONE              0x00
#define HAZARD_FIRE              (1U << 0)
#define HAZARD_SMOKE             (1U << 1)
#define HAZARD_HIGH_TEMP         (1U << 2)
#define HAZARD_VIBRATION         (1U << 3)

/* Relay polarity: set to 1 for active HIGH relay modules, 0 for active LOW modules */
#define RELAY_ACTIVE_HIGH        1

/* ==================== Globals ==================== */
volatile uint32_t g_millis = 0;

static uint16_t gasValue = 0;
static bool flameDetected = false;
static int16_t temperatureX10 = 0;
static int16_t humidityX10 = 0;
static int32_t vibrationMetric = 0;

static uint8_t hazardMask = HAZARD_NONE;
static uint8_t previousHazardMask = HAZARD_NONE;

static uint32_t lastLoopMs = 0;
static uint32_t lastSmsMs = 0;
static bool blinkSlowState = false;
static bool blinkFastState = false;
static bool lcdBlinkState = false;

/* ==================== Timer 0: 1ms tick ==================== */
ISR(TIMER0_COMPA_vect) {
    g_millis++;
}

static uint32_t millis(void) {
    uint32_t t;
    cli();
    t = g_millis;
    sei();
    return t;
}

static void timer0_init_1ms(void) {
    TCCR0A = (1 << WGM01);                 /* CTC */
    TCCR0B = (1 << CS01) | (1 << CS00);    /* /64 prescaler */
    OCR0A = 249;                           /* 16MHz/64=250kHz, 250 counts => 1ms */
    TIMSK0 = (1 << OCIE0A);
}

/* ==================== UART ==================== */
static void uart_init_9600(void) {
    uint16_t ubrr = 103; /* 16MHz, 9600 */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_tx(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
}

static void uart_print(const char *s) {
    while (*s) {
        uart_tx(*s++);
    }
}

static void uart_println(const char *s) {
    uart_print(s);
    uart_tx('\r');
    uart_tx('\n');
}

/* ==================== ADC ==================== */
static void adc_init(void) {
    ADMUX = (1 << REFS0); /* AVcc reference, ADC0 default */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* /128 */
}

static uint16_t adc_read(uint8_t ch) {
    ADMUX = (uint8_t)((ADMUX & 0xF0) | (ch & 0x0F));
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {
    }
    return ADC;
}

/* ==================== TWI (I2C) ==================== */
static void twi_init_100k(void) {
    TWSR = 0x00;
    TWBR = 72; /* ~100kHz */
}

static bool twi_start(uint8_t addressRW) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }

    TWDR = addressRW;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }

    uint8_t status = TWSR & 0xF8;
    return (status == 0x18 || status == 0x40);
}

static void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static bool twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
    return ((TWSR & 0xF8) == 0x28);
}

static uint8_t twi_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) {
    }
    return TWDR;
}

static uint8_t twi_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
    return TWDR;
}

/* ==================== LCD over I2C ==================== */
static void lcd_i2c_write(uint8_t value) {
    if (!twi_start((LCD_I2C_ADDR << 1) | 0)) {
        return;
    }
    twi_write(value);
    twi_stop();
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_i2c_write(data | LCD_EN | LCD_BL);
    _delay_us(1);
    lcd_i2c_write((data & (uint8_t)~LCD_EN) | LCD_BL);
    _delay_us(50);
}

static void lcd_write4(uint8_t nibble, bool rs) {
    uint8_t data = (uint8_t)((nibble & 0xF0) | LCD_BL);
    if (rs) {
        data |= LCD_RS;
    }
    lcd_i2c_write(data);
    lcd_pulse_enable(data);
}

static void lcd_send_cmd(uint8_t cmd) {
    lcd_write4(cmd & 0xF0, false);
    lcd_write4((uint8_t)((cmd << 4) & 0xF0), false);
    _delay_us(50);
}

static void lcd_send_data(uint8_t data) {
    lcd_write4(data & 0xF0, true);
    lcd_write4((uint8_t)((data << 4) & 0xF0), true);
    _delay_us(50);
}

static void lcd_init(void) {
    _delay_ms(50);

    lcd_write4(0x30, false);
    _delay_ms(5);
    lcd_write4(0x30, false);
    _delay_us(150);
    lcd_write4(0x30, false);
    _delay_us(150);
    lcd_write4(0x20, false); /* 4-bit mode */

    lcd_send_cmd(0x28); /* 4-bit, 2 line, 5x8 */
    lcd_send_cmd(0x0C); /* display ON, cursor OFF */
    lcd_send_cmd(0x06); /* increment cursor */
    lcd_send_cmd(0x01); /* clear */
    _delay_ms(2);
}

static void lcd_clear(void) {
    lcd_send_cmd(0x01);
    _delay_ms(2);
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(addr);
}

static void lcd_print(const char *s) {
    while (*s) {
        lcd_send_data((uint8_t)*s++);
    }
}

/* ==================== MPU6050 ==================== */
static void mpu_write_reg(uint8_t reg, uint8_t value) {
    if (!twi_start((MPU6050_ADDR << 1) | 0)) {
        return;
    }
    twi_write(reg);
    twi_write(value);
    twi_stop();
}

static void mpu_init(void) {
    mpu_write_reg(0x6B, 0x00); /* Wake up */
    _delay_ms(100);
}

static void mpu_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    if (!twi_start((MPU6050_ADDR << 1) | 0)) {
        *ax = 0;
        *ay = 0;
        *az = 16384;
        return;
    }
    twi_write(0x3B);

    if (!twi_start((MPU6050_ADDR << 1) | 1)) {
        *ax = 0;
        *ay = 0;
        *az = 16384;
        return;
    }

    uint8_t axh = twi_read_ack();
    uint8_t axl = twi_read_ack();
    uint8_t ayh = twi_read_ack();
    uint8_t ayl = twi_read_ack();
    uint8_t azh = twi_read_ack();
    uint8_t azl = twi_read_nack();
    twi_stop();

    *ax = (int16_t)((axh << 8) | axl);
    *ay = (int16_t)((ayh << 8) | ayl);
    *az = (int16_t)((azh << 8) | azl);
}

/* ==================== DHT22 ==================== */
static bool dht_wait_level(bool high, uint16_t timeoutUs) {
    while (timeoutUs--) {
        bool pinHigh = (PIND & (1 << PD3)) != 0;
        if (pinHigh == high) {
            return true;
        }
        _delay_us(1);
    }
    return false;
}

static bool dht22_read(int16_t *tempX10, int16_t *humX10) {
    uint8_t data[5] = {0, 0, 0, 0, 0};

    /* Start signal */
    DDRD |= (1 << PD3);   /* output */
    PORTD &= ~(1 << PD3); /* low */
    _delay_ms(2);
    PORTD |= (1 << PD3);  /* high */
    _delay_us(30);
    DDRD &= ~(1 << PD3);  /* input */

    if (!dht_wait_level(false, 120)) return false;
    if (!dht_wait_level(true, 120)) return false;
    if (!dht_wait_level(false, 120)) return false;

    for (uint8_t i = 0; i < 40; i++) {
        if (!dht_wait_level(true, 120)) return false;
        _delay_us(35);

        data[i / 8] <<= 1;
        if (PIND & (1 << PD3)) {
            data[i / 8] |= 1;
        }

        if (!dht_wait_level(false, 120)) return false;
    }

    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        return false;
    }

    uint16_t rawHum = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    uint16_t rawTmp = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);

    *humX10 = (int16_t)rawHum;
    if (rawTmp & 0x8000) {
        rawTmp &= 0x7FFF;
        *tempX10 = -(int16_t)rawTmp;
    } else {
        *tempX10 = (int16_t)rawTmp;
    }

    return true;
}

/* ==================== Hardware Output Control ==================== */
static void gpio_init(void) {
    DDRD &= ~(1 << PD2);                          /* Flame input */
    DDRD |= (1 << PD6) | (1 << PD7);              /* Relays */
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); /* buzzer + LEDs */

#if RELAY_ACTIVE_HIGH
    PORTD &= ~((1 << PD6) | (1 << PD7));
#else
    PORTD |= (1 << PD6) | (1 << PD7);
#endif

    PORTB &= ~((1 << PB0) | (1 << PB2) | (1 << PB3));
    PORTB |= (1 << PB1);
}

static void set_relay_fan(bool on) {
#if RELAY_ACTIVE_HIGH
    if (on) PORTD |= (1 << PD6);
    else PORTD &= ~(1 << PD6);
#else
    if (on) PORTD &= ~(1 << PD6);
    else PORTD |= (1 << PD6);
#endif
}

static void set_relay_pump(bool on) {
#if RELAY_ACTIVE_HIGH
    if (on) PORTD |= (1 << PD7);
    else PORTD &= ~(1 << PD7);
#else
    if (on) PORTD &= ~(1 << PD7);
    else PORTD |= (1 << PD7);
#endif
}

static void set_buzzer(bool on) {
    if (on) PORTB |= (1 << PB0);
    else PORTB &= ~(1 << PB0);
}

/* ==================== GSM SMS ==================== */
static void gsm_init(void) {
    uart_println("AT");
    _delay_ms(400);
    uart_println("AT+CMGF=1");
    _delay_ms(400);
    uart_println("AT+CSCS=\"GSM\"");
    _delay_ms(400);
}

static void gsm_send_sms(const char *phone, const char *message) {
    uart_println("AT+CMGF=1");
    _delay_ms(300);

    uart_print("AT+CMGS=\"");
    uart_print(phone);
    uart_println("\"");
    _delay_ms(500);

    uart_print(message);
    uart_tx(26); /* Ctrl+Z */
    _delay_ms(4000);
}

/* ==================== Application Logic ==================== */
static void read_all_sensors(void) {
    gasValue = adc_read(0);
    flameDetected = ((PIND & (1 << PD2)) == 0);

    int16_t t = temperatureX10;
    int16_t h = humidityX10;
    if (dht22_read(&t, &h)) {
        temperatureX10 = t;
        humidityX10 = h;
    }

    int16_t ax = 0, ay = 0, az = 16384;
    mpu_read_accel(&ax, &ay, &az);
    vibrationMetric = labs((long)ax) + labs((long)ay) + labs((long)az - 16384L);
}

static void evaluate_hazards(void) {
    hazardMask = HAZARD_NONE;

    if (flameDetected) {
        hazardMask |= HAZARD_FIRE;
    }
    if (gasValue > GAS_THRESHOLD) {
        hazardMask |= HAZARD_SMOKE;
    }
    if (temperatureX10 > TEMP_THRESHOLD_X10) {
        hazardMask |= HAZARD_HIGH_TEMP;
    }
    if (vibrationMetric > VIBRATION_THRESHOLD) {
        hazardMask |= HAZARD_VIBRATION;
    }
}

static const char *get_alert_message(void) {
    if ((hazardMask & HAZARD_FIRE) && (hazardMask & HAZARD_SMOKE)) {
        return "CRITICAL: FIRE + SMOKE detected. Pump and fan active.";
    }
    if (hazardMask & HAZARD_FIRE) {
        return "ALERT: Fire detected. Pump activated.";
    }
    if (hazardMask & HAZARD_SMOKE) {
        return "ALERT: Smoke/gas high. Exhaust fan activated.";
    }
    if (hazardMask & HAZARD_HIGH_TEMP) {
        return "Warning: High temperature in processing area.";
    }
    if (hazardMask & HAZARD_VIBRATION) {
        return "Warning: Abnormal machine vibration detected.";
    }
    return "System warning";
}

static void update_leds_and_buzzer(uint32_t nowMs) {
    if (hazardMask == HAZARD_NONE) {
        PORTB |= (1 << PB1);                /* Green ON */
        PORTB &= ~((1 << PB2) | (1 << PB3));/* Yellow/Red OFF */
        set_buzzer(false);
        return;
    }

    if ((hazardMask & HAZARD_FIRE) || (hazardMask & HAZARD_SMOKE)) {
        if (blinkFastState) PORTB |= (1 << PB3);
        else PORTB &= ~(1 << PB3);

        PORTB &= ~(1 << PB1); /* Green OFF */
        if (hazardMask & (HAZARD_HIGH_TEMP | HAZARD_VIBRATION)) PORTB |= (1 << PB2);
        else PORTB &= ~(1 << PB2);

        set_buzzer(true);
        (void)nowMs;
        return;
    }

    PORTB &= ~((1 << PB1) | (1 << PB3)); /* Green/Red OFF */
    if (blinkSlowState) PORTB |= (1 << PB2);
    else PORTB &= ~(1 << PB2);

    if (hazardMask & HAZARD_HIGH_TEMP) {
        bool beep = ((nowMs / 1000UL) % 10UL) == 0;
        set_buzzer(beep);
    } else if (hazardMask & HAZARD_VIBRATION) {
        uint32_t slot = nowMs % 5000UL;
        bool beep2 = (slot < 200UL) || (slot > 450UL && slot < 650UL);
        set_buzzer(beep2);
    } else {
        set_buzzer(false);
    }
}

static void update_relays(void) {
    set_relay_fan((hazardMask & HAZARD_SMOKE) != 0);
    set_relay_pump((hazardMask & HAZARD_FIRE) != 0);
}

static void update_lcd(void) {
    char line1[17];
    char line2[17];

    if (hazardMask == HAZARD_NONE) {
        int16_t tInt = (int16_t)(temperatureX10 / 10);
        int16_t hInt = (int16_t)(humidityX10 / 10);

        snprintf(line1, sizeof(line1), "T:%2dC H:%2d G:%3u", tInt, hInt, gasValue);
        snprintf(line2, sizeof(line2), "V:%5ld SAFE", vibrationMetric);

        lcd_set_cursor(0, 0);
        lcd_print("                ");
        lcd_set_cursor(0, 0);
        lcd_print(line1);

        lcd_set_cursor(0, 1);
        lcd_print("                ");
        lcd_set_cursor(0, 1);
        lcd_print(line2);
        return;
    }

    if (!lcdBlinkState) {
        lcd_set_cursor(0, 0);
        lcd_print("                ");
        lcd_set_cursor(0, 1);
        lcd_print("                ");
        return;
    }

    lcd_set_cursor(0, 0);
    lcd_print("                ");
    lcd_set_cursor(0, 1);
    lcd_print("                ");

    if ((hazardMask & HAZARD_FIRE) && (hazardMask & HAZARD_SMOKE)) {
        lcd_set_cursor(0, 0);
        lcd_print("FIRE + SMOKE");
        lcd_set_cursor(0, 1);
        lcd_print("EMERGENCY");
    } else if (hazardMask & HAZARD_FIRE) {
        lcd_set_cursor(0, 0);
        lcd_print("FIRE ALERT");
        lcd_set_cursor(0, 1);
        lcd_print("PUMP ACTIVE");
    } else if (hazardMask & HAZARD_SMOKE) {
        lcd_set_cursor(0, 0);
        lcd_print("SMOKE ALERT");
        lcd_set_cursor(0, 1);
        lcd_print("FAN ACTIVE");
    } else if (hazardMask & HAZARD_HIGH_TEMP) {
        lcd_set_cursor(0, 0);
        lcd_print("HIGH TEMP");
        lcd_set_cursor(0, 1);
        lcd_print("CHECK AREA");
    } else if (hazardMask & HAZARD_VIBRATION) {
        lcd_set_cursor(0, 0);
        lcd_print("VIBRATION ALERT");
        lcd_set_cursor(0, 1);
        lcd_print("MAINTENANCE");
    }
}

static void try_send_alert_sms(void) {
    const char phoneNumber[] = "+94719157948"; /* TODO: replace */

    if (hazardMask == HAZARD_NONE) {
        return;
    }

    bool changed = (hazardMask != previousHazardMask);
    bool intervalElapsed = (millis() - lastSmsMs) >= SMS_INTERVAL_MS;

    if (changed || intervalElapsed) {
        gsm_send_sms(phoneNumber, get_alert_message());
        lastSmsMs = millis();
    }
}

int main(void) {
    gpio_init();
    timer0_init_1ms();
    adc_init();
    twi_init_100k();
    uart_init_9600();

    sei();

    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("System Starting");
    lcd_set_cursor(0, 1);
    lcd_print("Please wait...");

    mpu_init();
    gsm_init();

    _delay_ms(1200);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Tea Factory");
    lcd_set_cursor(0, 1);
    lcd_print("Safety Ready");

    while (1) {
        uint32_t now = millis();

        if ((now - lastLoopMs) < LOOP_INTERVAL_MS) {
            continue;
        }
        lastLoopMs = now;

        blinkSlowState = ((now / LED_BLINK_SLOW_MS) % 2U) != 0U;
        blinkFastState = ((now / LED_BLINK_FAST_MS) % 2U) != 0U;
        lcdBlinkState = ((now / LCD_ALERT_BLINK_MS) % 2U) != 0U;

        read_all_sensors();
        evaluate_hazards();
        update_relays();
        update_leds_and_buzzer(now);
        update_lcd();
        try_send_alert_sms();

        previousHazardMask = hazardMask;
    }
}
