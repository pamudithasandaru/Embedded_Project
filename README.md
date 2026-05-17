# Tea Factory Safety Monitor (Strict Embedded C on ATmega328P)

## 1) Project Overview
This project runs on Arduino UNO R3 hardware (ATmega328P) using strict register-level Embedded C.
It monitors:
- Flame (digital)
- Smoke or gas level (MQ-2 analog)
- Temperature and humidity (DHT22)
- Vibration (MPU6050 accelerometer)

It provides:
- Local alerts on LCD, LEDs, buzzer
- Automatic relay actions (fan and pump)
- Remote SMS alerts using SIM800L

Main firmware file:
- tea_factory_strict_embedded_c.c

## 2) Important Safety Notice
This is a student prototype for lab or educational demonstration only.
Do not use as a certified fire safety system in real factories.

Always:
- Keep a real fire extinguisher available during flame tests
- Use very small controlled flame only for sensor demonstration
- Keep water away from electronics
- Disconnect power before changing wiring
- Avoid mains voltage in this prototype setup

## 3) Hardware List
Required:
- Arduino UNO R3
- MQ-2 gas or smoke sensor module
- Flame sensor module (digital output)
- DHT22 sensor
- MPU6050 module (I2C)
- 16x2 LCD with I2C backpack (PCF8574)
- SIM800L GSM module + micro SIM with SMS enabled
- 2-channel 5V relay module
- Buzzer (active)
- 3 LEDs (Green, Yellow, Red)
- Resistors for LEDs (typically 220 ohm)
- Breadboard and jumper wires

Recommended:
- External stable supply for SIM800L (about 4.0V to 4.2V, high current capable)
- Common ground wiring between all modules

## 4) Pin Mapping (Firmware Mapping)
Digital and analog mapping used by firmware:
- D2 (PD2): Flame sensor digital output (active LOW)
- D3 (PD3): DHT22 data
- D6 (PD6): Relay channel for fan
- D7 (PD7): Relay channel for pump
- D8 (PB0): Buzzer
- D9 (PB1): Green LED
- D10 (PB2): Yellow LED
- D11 (PB3): Red LED
- A0 (PC0): MQ-2 analog output
- A4 (PC4): I2C SDA (MPU6050 + LCD)
- A5 (PC5): I2C SCL (MPU6050 + LCD)
- D0/D1 UART: SIM800L serial link (recommended for strict C)

## 5) Wiring Guide
## 5.1 Ground and Power First
1. Connect UNO GND to breadboard ground rail.
2. Connect all module grounds to the same ground rail.
3. Connect UNO 5V to 5V modules that are 5V compatible (LCD module, relay module, some sensors).
4. Power SIM800L from a separate stable supply (not directly from UNO 5V pin).
5. Connect SIM800L GND to UNO GND (common reference is mandatory).

## 5.2 Sensor Wiring
MQ-2:
- VCC -> 5V
- GND -> GND
- AO -> A0

Flame sensor (digital mode):
- VCC -> 5V
- GND -> GND
- DO -> D2

DHT22:
- VCC -> 5V
- GND -> GND
- DATA -> D3
- Add 10k pull-up resistor from DATA to 5V

MPU6050:
- VCC -> 5V (or 3.3V depending on module regulator)
- GND -> GND
- SDA -> A4
- SCL -> A5

## 5.3 LCD I2C Wiring
16x2 LCD with I2C backpack:
- VCC -> 5V
- GND -> GND
- SDA -> A4
- SCL -> A5

If LCD does not display text:
- change LCD_I2C_ADDR in code from 0x27 to 0x3F

## 5.4 Output Wiring
Relay module:
- VCC -> 5V
- GND -> GND
- IN1 -> D6 (Fan)
- IN2 -> D7 (Pump)

Buzzer:
- Positive -> D8
- Negative -> GND

LEDs (with current limiting resistors):
- Green LED anode -> D9 through 220 ohm resistor, cathode -> GND
- Yellow LED anode -> D10 through 220 ohm resistor, cathode -> GND
- Red LED anode -> D11 through 220 ohm resistor, cathode -> GND

## 5.5 SIM800L Wiring (UART)
Recommended strict C wiring for this firmware:
- SIM800L TX -> UNO D0 (RX)
- UNO D1 (TX) -> SIM800L RX through voltage divider

Important:
- UNO TX is 5V logic, SIM800L RX is lower voltage tolerant. Use divider.
- Disconnect SIM800L from D0 and D1 before uploading firmware.
- Reconnect SIM800L after successful upload.

## 6) Software Configuration
File to edit:
- tea_factory_strict_embedded_c.c

Update these values before final use:
1. Phone number in try_send_alert_sms:
- Replace +94XXXXXXXXX with your real number in international format.

2. Relay polarity:
- RELAY_ACTIVE_HIGH = 1 for active-HIGH relays
- RELAY_ACTIVE_HIGH = 0 for active-LOW relays

3. LCD address:
- LCD_I2C_ADDR = 0x27 (common)
- Change to 0x3F if required

4. Threshold tuning:
- GAS_THRESHOLD
- TEMP_THRESHOLD_X10
- VIBRATION_THRESHOLD

## 7) Build and Flash (Windows)
Open terminal in project folder and run:

Step 1 compile:
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os tea_factory_strict_embedded_c.c -o tea_factory_strict_embedded_c.elf

Step 2 create hex:
avr-objcopy -O ihex -R .eeprom tea_factory_strict_embedded_c.elf tea_factory_strict_embedded_c.hex

Step 3 flash:
avrdude -c arduino -p m328p -P COM5 -b 115200 -U flash:w:tea_factory_strict_embedded_c.hex:i

Replace COM5 with your actual UNO port.

Find COM port:
- Device Manager -> Ports (COM and LPT) -> Arduino UNO (COMx)

## 8) First Power-On Checklist
Before power:
- Verify no short circuits
- Verify module VCC and GND are correct
- Verify SIM800L has separate stable power and common ground
- Verify D0 and D1 are disconnected during upload

After power:
- LCD should show startup text
- Green LED should indicate normal state
- Relays should remain in safe default state

## 9) Safe Testing Procedure
Run tests in this order.

Test A: Idle normal condition
- No flame, no smoke, normal room temperature
Expected:
- Green LED ON
- Buzzer OFF
- LCD shows live values
- Relays OFF

Test B: Flame sensor test
- Bring small controlled flame near flame sensor briefly
Expected:
- Fire alert on LCD
- Red LED blinking
- Pump relay ON
- Buzzer active
- SMS sent

Test C: Smoke test
- Use very small smoke source safely near MQ-2
Expected:
- Smoke alert on LCD
- Fan relay ON
- Red LED behavior and buzzer alert
- SMS sent

Test D: Temperature test
- Warm DHT22 sensor gently (do not overheat)
Expected:
- High temp warning state
- Yellow LED behavior
- Warning buzzer pattern

Test E: Vibration test
- Tap or shake mounted MPU6050 gently
Expected:
- Vibration warning state
- Yellow LED blinking
- Vibration warning behavior

Test F: Combined hazard test
- Trigger flame and smoke together briefly
Expected:
- Emergency state
- Both relay actions ON
- Critical SMS message

## 10) Precautions During Testing
Electrical:
- Do not power SIM800L from weak supply
- Do not run relay loads beyond rated voltage or current
- Keep all grounds common

Thermal and fire:
- Keep flame small and far from wires
- Keep flammable material away
- Keep extinguisher nearby

Mechanical and water:
- Secure modules to avoid short circuits
- Keep water pump demo isolated from electronics

Operational:
- Test one hazard at a time first
- Keep a notebook of baseline sensor readings
- Tune thresholds to reduce false alarms

## 11) Troubleshooting
No upload:
- Check COM port
- Disconnect SIM800L from D0 D1 while flashing
- Check bootloader baud and cable quality

LCD blank:
- Verify A4 SDA and A5 SCL
- Try LCD_I2C_ADDR 0x3F
- Check LCD contrast potentiometer

No SMS:
- Verify SIM card is active and has balance
- Verify antenna attached
- Verify SIM800L supply is strong enough
- Verify phone number format

Relays reversed:
- Switch RELAY_ACTIVE_HIGH setting

Unstable sensor values:
- Improve grounding
- Shorten wiring
- Add decoupling capacitors
- Warm-up MQ-2 sensor before measurement

## 12) Good Practices for Report and Demo
- Mention this is strict register-level Embedded C firmware
- Show direct use of AVR peripherals: GPIO, ADC, UART, TWI, timer interrupt
- Include threshold calibration table from your own test values
- Include risk note that prototype is non-certified and educational

## 13) File Summary
- tea_factory_strict_embedded_c.c: Main strict Embedded C firmware
- build_and_flash_strict_c.txt: Quick command reference
- README.md: This full setup and operation guide
