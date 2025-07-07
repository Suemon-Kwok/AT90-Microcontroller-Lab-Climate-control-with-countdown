#     AT90 Microcontroller Lab Climate control with countdown

Exercise 2 – Climate control with countdown

Draw a state diagram and write a C program to realise a climate control on the AT90 as follows.

1. Configure three switches (PORTA 0-2) as inputs.

2. Configure the 7-segment display (PORTC) as output to show the countdown timer.

3. Configure the light (PB6), heater (PB7), and fan (PB5) as output devices.

4. Setup the ADC for single conversion mode, 8-bit resolution, frequency divider of 128.

5. When powered on, the 7-segment display flashes. While the display flashes, the user can set
a countdown timer:

a. Read the analog values from two potentiometers and process them:

i. Potentiometer 1 (ADC2) controls the tens digit of the countdown timer.

ii. Potentiometer 2 (ADC1) controls the units digit of the countdown timer.

b. The 7-segment display shows the current countdown value:

i. The upper 4 bits represent the tens digit.

ii. The lower 4 bits represent the units digit.

iii. Only decimal values (0-9) are allowed.

iv. The display updates dynamically as the user adjusts the potentiometers.

6. When PA0 is switched on, the light (PB6) turns on, and the countdown starts.

a. When the countdown reaches 0, the light turns off automatically, regardless of the
switch state.

7. The same logic applies to the other two devices:

a. PA1 controls the fan (PB5).

b. PA2 controls the heater (PB7).

8. If all switches (PA0, PA1, PA2) are turned off during the countdown, the countdown is
aborted immediately.

9. Afterwards, the program repeats from step 5.


/*

*	GccApplication1.c

*

*	Created: 27/03/2025 2:38:49 pm

*	Author :

*/

#define F_CPU 8000000UL  // 8MHz clock speed

#include <avr/io.h>

#include <avr/interrupt.h>

#include <util/delay.h>

// State definitions

/*

 Defines three system states:
 
 STATE_SETUP → Set countdown timer using potentiometers.
 
 STATE_COUNTDOWN → Active countdown, controlling light, fan, and heater.
 
 STATE_FINISHED → Countdown reaches zero, devices turn off.
 
 */

typedef enum {
    STATE_SETUP,         // Initial state for setting the countdown timer
    STATE_COUNTDOWN,     // Running the countdown
    STATE_FINISHED       // Countdown finished
} system_state_t;

// LED and switch pin definitions

/*

*Switches (PA0–PA2) control the respective devices.

*Devices (PB5–PB7) are outputs controlling the fan, light, and heater.

*/

#define FAN_PIN       PB5

#define LIGHT_PIN     PB6

#define HEATER_PIN    PB7

#define SWITCH_LIGHT  PA0  // Controls PB6 (Light)

#define SWITCH_FAN    PA1  // Controls PB5 (Fan)

#define SWITCH_HEATER PA2  // Controls PB7 (Heater)

// 7-segment display patterns for digits 0-9

// Common anode 7-segment display (0 turns segment on) const uint8_t SEGMENT_PATTERNS[10] = {

0b11000000,  // 0

0b11111001,  // 1

0b10100100,  // 2

0b10110000,  // 3

0b10011001,  // 4

0b10010010,  // 5

0b10000010,  // 6

0b11111000,  // 7

0b10000000,  // 8     0b10010000   // 9

};

// Global variables

volatile system_state_t current_state = STATE_SETUP; volatile uint8_t countdown_value = 0;  // In seconds (tens and units) volatile uint8_t tens_digit = 0;       // 0-9 volatile uint8_t units_digit = 0;      // 0-9

volatile uint8_t display_flash_state = 0; volatile uint16_t timer_counter = 0;

// Function prototypes

/*

*Declares functions for:

*Initialization (GPIO, ADC, timer).

*Countdown processing (potentiometer readings, display control, switch detection).

State transitions (starting/stopping countdown).

*/

void init_gpio(void); void init_adc(void); void init_timer(void); void read_potentiometers(void);

void update_display(uint8_t tens, uint8_t units, uint8_t flashing); void handle_switches(void);

void start_countdown(void); void stop_countdown(void);

/*

*Functionality

*Initializes GPIO, ADC, and timers.

*Reads potentiometer input for countdown time.

*Displays countdown on 7-segment display (flashing mode).

*Transitions to countdown state when a switch is activated.

*/ int main(void) {     // Initialize peripherals     init_gpio();     init_adc();     init_timer();

// Enable global interrupts     sei();

// Main loop     while (1) {         switch (current_state) {             case STATE_SETUP:
            // Read potentiometers and update countdown value                 read_potentiometers();

            // Update display with flashing effect                 update_display(tens_digit, units_digit, 1);

            // Check if any switch is turned on to start countdown                 handle_switches();                 break;

        case STATE_COUNTDOWN:
            // Update display (without flashing)
            update_display(tens_digit, units_digit, 1);  // 1 means display on                 // Update outputs based on current switch states - active high logic                 if ((PINA & (1 << SWITCH_LIGHT))) {
            PORTB |= (1 << LIGHT_PIN);  // Turn on light
}
 else {
     PORTB &= ~(1 << LIGHT_PIN); // Turn off light
                }

                if ((PINA & (1 << SWITCH_FAN))) {
                    PORTB |= (1 << FAN_PIN);    // Turn on fan
                }
                else {
                    PORTB &= ~(1 << FAN_PIN);   // Turn off fan
                }

                if ((PINA & (1 << SWITCH_HEATER))) {
                    PORTB |= (1 << HEATER_PIN); // Turn on heater
                }
                else {
                    PORTB &= ~(1 << HEATER_PIN); // Turn off heater
                }

                // Check if all switches turned off to abort countdown
                if ((PINA & ((1 << SWITCH_LIGHT) | (1 << SWITCH_FAN) | (1 << SWITCH_HEATER))) == 0) {
                    stop_countdown();
                    current_state = STATE_SETUP;
                }                 break;

            case STATE_FINISHED:
                // Turn off all devices
                PORTB &= ~((1 << LIGHT_PIN) | (1 << HEATER_PIN) | (1 << FAN_PIN));                 current_state = STATE_SETUP;  // Return to setup state                 break;
        }
    }

    return 0;  // Will never reach here }
    // Initialize GPIO pins
    /*
    *Configures switches as inputs.
    *Configures 7-segment display as output.
    *Configures devices (fan, light, heater) as outputs and initializes them as OFF.
    */
    void init_gpio(void) {
        // Set PORTA 0-2 as inputs with pull-ups disabled (we'll use external switches)
        DDRA &= ~((1 << SWITCH_LIGHT) | (1 << SWITCH_FAN) | (1 << SWITCH_HEATER));
        PORTA &= ~((1 << SWITCH_LIGHT) | (1 << SWITCH_FAN) | (1 << SWITCH_HEATER)); // Disable pullups

        // Set PORTC as output for 7-segment display
        DDRC = 0xFF;  // All pins as outputs
        PORTC = 0xFF; // All segments off initially (assuming common anode)

        // Set PORTB 5-7 as outputs for devices
        DDRB |= ((1 << FAN_PIN) | (1 << LIGHT_PIN) | (1 << HEATER_PIN));
        PORTB &= ~((1 << FAN_PIN) | (1 << LIGHT_PIN) | (1 << HEATER_PIN)); // All devices off initially
    }
    // Initialize ADC
    /*
    *Uses AVCC (5V) as reference voltage.
    *Left-adjusts ADC result for 8-bit mode.
    Sets prescaler to 128 for proper ADC timing
    */
    void init_adc(void) {
        // Set reference voltage to AVCC, left adjust result for 8-bit resolution
        ADMUX = (1 << REFS0) | (1 << ADLAR);

        // Enable ADC, set prescaler to 128 (8MHz/128 = 62.5kHz)
        ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    }
    // Initialize timer for display flashing and countdown
    /*
    *Uses Timer/Counter1 in CTC mode for countdown timing.
    *Sets prescaler to 1024.
    Generates an interrupt every 0.5 seconds
    */
    void init_timer(void) {
        // Set up Timer/Counter1 for 1 second intervals
        // Using CTC mode (Clear Timer on Compare Match)
        TCCR1B |= (1 << WGM12);  // CTC mode

        // Set prescaler to 1024
        TCCR1B |= (1 << CS12) | (1 << CS10);  // Prescaler 1024

        // Set compare match value for 0.5 second (flashing rate)
        // 8MHz / 1024 = 7812.5 ticks per second
        // For 0.5 second, we need 7812.5 * 0.5 = 3906 ticks
        OCR1A = 3906;

        // Enable Timer/Counter1 compare match A interrupt
        TIMSK1 |= (1 << OCIE1A);
    }
    // Read potentiometer values and update countdown digits
    /*
    *Reads ADC value from potentiometer 1 (tens digit).
    *Maps the ADC range (0–255) to digits 0–9.
    Similar logic applies for units digit using ADC1.
    */
    void read_potentiometers(void) {
        // Read potentiometer 1 (tens digit)
        ADMUX = (ADMUX & 0xF0) | 2;  // Select ADC2     ADCSRA |= (1 << ADSC);       // Start conversion     while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete     uint8_t tens_value = ADCH;   // Read 8-bit result (left adjusted)

        // Map to 0-9 range
        tens_digit = tens_value / 26;  // 255 / 10 ≈ 25.5, so divide by 26     if (tens_digit > 9) tens_digit = 9;

        // Read potentiometer 2 (units digit)
        ADMUX = (ADMUX & 0xF0) | 1;  // Select ADC1     ADCSRA |= (1 << ADSC);       // Start conversion     while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete     uint8_t units_value = ADCH;  // Read 8-bit result (left adjusted)

        // Map to 0-9 range
        units_digit = units_value / 26;  // 255 / 10 ≈ 25.5, so divide by 26     if (units_digit > 9) units_digit = 9;

        // Update countdown value
        countdown_value = tens_digit * 10 + units_digit;
    }
    // Update the 7-segment display
    /*
    *Updates the 7-segment display with current countdown time.
    *If flashing is enabled, display turns on and off alternately.
    */
    void update_display(uint8_t tens, uint8_t units, uint8_t display_on) {
        if (display_on) {
            // For 7-segment display - directly use digits for display
            // Upper 4 bits for tens digit, lower 4 bits for units digit
            PORTC = (tens << 4) | units;
        }
        else {
            // Display off when flashing
            PORTC = 0xFF;  // All segments off (common anode)
        }
    }
    // Handle switch inputs and state transitions
    /*
    *Checks if a switch is activated to start countdown.
    *Transitions system state to STATE_COUNTDOWN.
    */
    void handle_switches(void) {
        // If we're in setup mode and any switch is turned on     if (current_state == STATE_SETUP) {
            // Check if any switch is active (now pulled high = ON, pulled low = OFF)
            // This inverts the previous logic to fix the flipped switch issue
        if ((PINA & ((1 << SWITCH_LIGHT) | (1 << SWITCH_FAN) | (1 << SWITCH_HEATER))) > 0) {

            // Start the countdown             start_countdown();

            // Set outputs based on switch states - active high logic
            if ((PINA & (1 << SWITCH_LIGHT))) {
                PORTB |= (1 << LIGHT_PIN);  // Turn on light (PB6)
            }
            else {
                PORTB &= ~(1 << LIGHT_PIN); // Make sure light is off
            }

            if ((PINA & (1 << SWITCH_FAN))) {
                PORTB |= (1 << FAN_PIN);    // Turn on fan (PB5)
            }
            else {
                PORTB &= ~(1 << FAN_PIN);   // Make sure fan is off
            }

            if ((PINA & (1 << SWITCH_HEATER))) {
                PORTB |= (1 << HEATER_PIN); // Turn on heater (PB7)
            }
            else {
                PORTB &= ~(1 << HEATER_PIN); // Make sure heater is off
            }
        }
    } }
    // Start the countdown void start_countdown(void) {     current_state = STATE_COUNTDOWN;
    timer_counter = 0;  // Reset timer counter
}

// Stop the countdown void stop_countdown(void) {     current_state = STATE_SETUP;

PORTB &= ~((1 << LIGHT_PIN) | (1 << HEATER_PIN) | (1 << FAN_PIN));  // Turn off all devices

}

// Timer/Counter1 compare match A interrupt service routine

/*

*Flashes display in setup mode.

*Decrements countdown every second.

*Transitions to STATE_FINISHED when countdown reaches zero.

*/

ISR(TIMER1_COMPA_vect) {

    // Toggle display flash state in setup mode     if (current_state == STATE_SETUP) {         display_flash_state = !display_flash_state;
}

// Increment timer counter (2 counts = 1 second since we're at 0.5s per interrupt)     timer_counter++;

// If we've reached 1 second     if (timer_counter >= 2) {         timer_counter = 0;

    // If we're in countdown mode, decrement countdown         if (current_state == STATE_COUNTDOWN && countdown_value > 0) {             countdown_value--;

        // Update tens and units digits             tens_digit = countdown_value / 10;             units_digit = countdown_value % 10;

        // Check if countdown has finished             if (countdown_value == 0) {                 current_state = STATE_FINISHED;
            }
        }
    }
}




        


            
