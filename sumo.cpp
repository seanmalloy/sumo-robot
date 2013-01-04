// Mini sumo program.
// Controls a Solarbotics sumovore robot with atmel brain board.

#define F_CPU 8000000UL   // CPU runs at 8 MHz

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

void setup();
void leds_off();
void startup_delay();
void delay_ms(uint16_t);
void delay_us(uint16_t);
uint8_t adc(uint8_t);
void adc_init();
void detect_enemy();
void pwm_init();
void move(int, int);
void forward_right_turn();
void forward_left_turn();
void backward_right_turn();
void backward_left_turn();
void turn_around();
void find_mode();
void detect_line();
void move_left(int direction, int speed);
void move_right(int direction, int speed);
void update_lights(uint8_t rear_detect); 

// PD0 - left IR sensor
// PD1 - right IR sensor

// PC0 - far left edge sensor
// PC1 - center left edge sensor
// PC2 - center edge sensor
// PC3 - center right edge sensor
// PC4 - far right edge sensor
// PC5 - Rear sharp IR sensor(I2C)

// PB5 - right motor direction
// PB4 - left motor direction
// PB2 - right motor enable(OCR1B)
// PB1 - left motor enable(OCR1A)

// PD2 - LED1
// PD3 - LED2
// PD4 - LED3
// PD5 - LED4
// PD6 - LED5

// Speed values for OCR1A and OCR1B.
//
// Stop    -- 0x0000 (0)
// Minimum -- 0x3fff (16,383)
// Medium  -- 0x9fff (40,959)
// Maximum -- 0xffff (65,5535)

int main()
{
    
    setup();

    while(1) {
        detect_enemy();
        detect_line();
    }

    return 0;

}


// Inital setup routine. Should be called once near the top
// of main().
void setup()
{

    // Setup motors for output.
    DDRB |= _BV(PB1) | _BV(PB2) | _BV(PB4) | _BV(PB5);

    // Setup LEDs for output.
    DDRD  = _BV(PD2) | _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6);

    // Setup IR sensors for input. 0 means input.
    DDRD &= ~_BV(PD0) & ~_BV(PD1);

    // Line sensors for input.
    DDRC = 0x00;

    // Disable I2C.
    TWCR &= ~_BV(TWEN);

    leds_off();
    startup_delay();
    leds_off();
    adc_init();
    pwm_init();

}


// Turns off red LED1 to LED5 on brainboard.
void leds_off()
{

    PORTD |= _BV(PD2) | _BV(PD3) | _BV(PD4) | _BV(PD5) | _BV(PD6);

}


// Delay 5 seconds. Turn on one LED each second.
void startup_delay()
{

    PORTD &= ~_BV(PD2);
    delay_ms(1000);
    PORTD &= ~_BV(PD3);
    delay_ms(1000);
    PORTD &= ~_BV(PD4);
    delay_ms(1000);
    PORTD &= ~_BV(PD5);
    delay_ms(1000);
    PORTD &= ~_BV(PD6);
    delay_ms(1000);

}


// Wait ms milliseconds.
void delay_ms(uint16_t ms)
{

    while (ms) {
        _delay_ms(1);
        ms--;
    }

}


// Wait us microseconds???
void delay_us(uint16_t us)
{

    while (us) {
        _delay_us(1);
        us--;
    }

}


// ADMUX Register
//     * MUX0 bit  - ???
//     * MUX1 bit  - ???
//     * MUX2 bit  - ???
//     * MUX3 bit  - ???
//     * ???? bit  - ??? read only bit ???
//     * ADLAR bit - left adjust result
//     * REFS0 bit - ??? voltage reference selection ???
//     * REFS1 bit - ??? voltage reference selection ???
//
// ADCSRA Register
//     * ADPS0 bit - ???
//     * ADPS1 bit - ???
//     * ADPS2 bit - ???
//     * ADIE bit  - interrupt enable
//     * ADIF bit  - set to 1 when conversion completes
//     * ADFR bit  - 1 enable free running mode
//     * ADSC bit  - 1 start conversion
//     * ADEN bit  - 1 enables conversion, 0 disables conversion
//
// Result Registers
//     * ADCH - ???
//     * ADCL - ???
uint8_t adc( uint8_t channel )
{

    // Set for 8-bit results for the desired channel number.

    ADMUX  = ( 1 << ADLAR ) | ( 1 << REFS0 ) | channel;

    // Start conversion.
    ADCSRA |= ( 1 << ADSC  );

    // Wait for the conversion to complete.
    while ( ADCSR & ( 1 << ADSC ) );

    // Retrun the result from the ADCH register
    return ADCH;

}


// Enable the analog to digital converter.
void adc_init()
{

    // Enable analog to digital converter(ADEN).
    // Set prescalar division factor to 64(ADPS1 and ADPS2).
    ADCSRA |= (1 << ADEN) | _BV(ADPS1) | _BV(ADPS2);
    delay_us(64);

}


// Dectect the enemy using the forward IR sensors and move the
// robot toward the enemy.
void detect_enemy()
{

    // Reading a 0 mean the enemy was detected.
    uint8_t irsensors = PIND & ( _BV(PD0) | _BV(PD1) ); 

    // possibly optional ... used for debuging.
    // delay_ms(10);

    // Read rear enemy sensor.
    // uint8_t rear_detect = adc( 5 );

    // Display rear sensor value on LEDs. Used for debuging.
    // update_lights(rear_detect);

    if (irsensors == 0) {
        // Enemy is straight ahead. Turn
        // both motors on full blast.

        move(1, 0xffff);

    } else if (irsensors == 2) {
        // Enemy is to the left.
        move_left(1, 0xffff);

    } else if (irsensors == 1) {
        // Enemy is to the right.
        move_right(1, 0xffff);

    } else {
        // No enemy in the front.
        find_mode();
        
        /* if(rear_detect > 64) {
            // Enemy in back.
            turn_around();
            move(1, 0xffff);

        } else {
            // No enemy in front or back.
            find_mode();
        } */

    }

} 

// Used for displaying sharp IR sensor reading on LEDs.
void update_lights(uint8_t rear_detect) {
    
        uint8_t temp = ~rear_detect;
        temp = temp >> 3;    
        temp = temp << 2;
        PORTD &= 0x83;
        PORTD |= temp;

}


// Setup fast 16-bit fast PWM mode using Timer/Counter1.
void pwm_init()
{

    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(CS10) | _BV(WGM13) | _BV(WGM12);
    ICR1   = 65535;

    // Default to motors off.
    OCR1A = 0x0000;
    OCR1B = 0x0000;

}


// Functions to control robot movement.
void move(int direction, int speed)
{

    if (direction >= 0) {

        // Set motor direction to forward.
        PORTB |= _BV(PB4) | _BV(PB5);

    } else {

        // Set motor direction to reverse.
        PORTB &= ~(_BV(PB4) | _BV(PB5));

    }

    // Left motor on.
    OCR1A = speed;

    // Right motor on.
    OCR1B = speed;

}


void forward_right_turn(int speed)
{

    move(1, speed);
    delay_ms(75);

    // Right motor off.
    OCR1B = 0x0000;

    delay_ms(600);

    // Both motors on.
    move(1, speed);
}


void forward_left_turn(int speed)
{

    move(1, speed);
    delay_ms(75);

    // Left motor off.
    OCR1A = 0x0000;

    delay_ms(600);
    move(1, speed);

}


void backward_right_turn(int speed)
{

    move(-1, speed);
    delay_ms(75);

    // Right motor off.
    OCR1B = 0x0000;

    delay_ms(600);
    move(-1, speed);

}


void backward_left_turn(int speed)
{

    move(-1, speed);
    delay_ms(75);

    // Left motor off.
    OCR1A = 0x0000;

    delay_ms(600);
    move(-1, speed);

}


void turn_around()
{

    // Stop.
    move(0, 0);

    // Set right motor direction to forward.
    PORTB |= _BV(PB5);

    // Set left motor direction to backward.
    PORTB &= ~(_BV(PB4));

    // Turn both motors on full blast.
    OCR1A = 0xffff;
    OCR1B = 0xffff;
    
    // Delay.
    delay_ms(335);

    // Stop.
    move(0, 0);

}


void find_mode()
{

    // Possible speed value for OCR1A/B.
    move(1, 0x3a98);
    // move(1, 0x2ee0);

}


void detect_line()
{

    // White line detection threshold.
    uint16_t white_threshold = 64;

    // Read line sensors.
    uint16_t line_sensor_fl  = adc( 0 ); // far left
    uint16_t line_sensor_cl  = adc( 1 ); // center left
    uint16_t line_sensor_ce  = adc( 2 ); // center
    uint16_t line_sensor_cr  = adc( 3 ); // center right
    uint16_t line_sensor_fr  = adc( 4 ); // far right

    // Stop motors if white line is detected.
    if ((line_sensor_fl < white_threshold) ||
        (line_sensor_cl < white_threshold) ||
        (line_sensor_ce < white_threshold) ||
        (line_sensor_cr < white_threshold) ||
        (line_sensor_fr < white_threshold)) {
    
        // Stop both motors.
        move(0, 0);

        // Reverse.
        move(-1, 0xffff); 
        delay_ms(50);

        // Turn around 180 degrees.
        turn_around();

    }

}


void move_right(int direction, int speed)
{

    if (direction >= 0) {

        // Set motor direction to forward.
        PORTB |= _BV(PB4) | _BV(PB5);

    } else {

        // Set motor direction to reverse.
        PORTB &= ~(_BV(PB4) | _BV(PB5));

    }

    // Left motor on.
    OCR1A = speed;

    // Right motor off.
    OCR1B = 0x0000;



}


void move_left(int direction, int speed)
{

    if (direction >= 0) {

        // Set motor direction to forward.
        PORTB |= _BV(PB4) | _BV(PB5);

    } else {

        // Set motor direction to reverse.
        PORTB &= ~(_BV(PB4) | _BV(PB5));

    }

    // Left motor off.
    OCR1A = 0x0000;

    // Right motor on.
    OCR1B = speed;

}
