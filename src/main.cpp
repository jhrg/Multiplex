
/**
 * @brief Use the Atmel 328 to drive multiplexed nixie tube display
 */
#include <Arduino.h>

#define CLOCK_1HZ 2  // Pin D2, INT0
#define TIMER_INTERRUPT_TEST_PIN B10000000  // Pin D7

// Set HIGH when the 1 second interrupt been triggered by the clock
volatile byte tick = LOW;
volatile bool get_time = false;
volatile bool update_display = false;

/**
 * @brief Record that one second has elapsed
 */
void timer_1HZ_tick_ISR() {
    static int tick_count = 0;

    tick = HIGH;
    tick_count++;

    update_display = true;
}

void setup() {
    // Enable some minor, chatty, messages on the serial line.
    Serial.begin(BAUD_RATE);
    Serial.println("boot");

    // Initialize all I/O pins to output, then set up the inputs/interrupts
    DDRD = B11111111;  // D0 - D7
    DDRC = B00111111;  // A0 - A5, bit 6 is RST, 7 is undefined
    DDRB = B00111111;  // D8 - D13, bits 6,7 are for the crystall

    // Initialize all GPIO pins to LOW
    PORTD = B00000000;
    PORTC = B00000000;
    PORTB = B00000000;

#if 0
    // blank the display
    digit_0 = -1;
    digit_1 = -1;
    digit_2 = -1;
    digit_3 = -1;
    digit_4 = -1;
    digit_5 = -1;

    // These don't need to be set to zero to blank, the -1 digit value
    // blanks the whole tube. But the code might as well start out with
    // rational values.
    d0_rhdp = 0;
    d1_rhdp = 0;
    d2_rhdp = 0;
    d3_rhdp = 0;
    d4_rhdp = 0;
    d5_rhdp = 0;

    // State machine initial conditions:
    // start up as if the display has cycled once through already
    blanking = true;
    digit = 1;
#endif

    cli();  // stop interrupts

    // This is used for the 1Hz pulse from the clock that triggers
    // time updates.
    pinMode(CLOCK_1HZ, INPUT_PULLUP);

    // time_1Hz_tick() sets a flag that is tested in loop()
    attachInterrupt(digitalPinToInterrupt(CLOCK_1HZ), timer_1HZ_tick_ISR, RISING);

    // Set up timer 2 - controls the display multiplexing

    // set timer2 interrupt at 950uS. Toggles between 950 and 50 uS
    TCCR2A = 0;  // set entire TCCR2A register to 0
    TCCR2B = 0;  // same for TCCR0B
    TCNT2 = 0;   // initialize counter value to 0

    // set compare match register for 900uS increments
    // OCR2A = 224;  // = [(16*10^6 / 64 ) * 0.000 900] - 1; (must be <256)

    // turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set CS22 bit for 64 pre-scaler --> B00000100
    TCCR2B |= (1 << CS22);
    // enable timer compare interrupt; TIMSK: Timer interrupt mask
    TIMSK2 |= (1 << OCIE2A);

    sei();  // start interrupts
}

ISR(TIMER2_COMPA_vect) {
    static bool blanking = true;
    // See https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
#if TIMER_INTERRUPT_DIAGNOSTIC
    PORTD |= TIMER_INTERRUPT_TEST_PIN;
#endif

    // If the current state is blanking, stop blanking and enter digit display state
    if (blanking) {
#if TIMER_INTERRUPT_DIAGNOSTIC_2
        avg_blanking_time += micros() - start;
        avg_blanking_time = avg_blanking_time >> 1;
        start = micros();
#endif
        // State is not blanking
        blanking = false;

        PORTD |= _BV(PORTD6);

        // Set the timer to illuminate the digit (e.g., for 900uS)
        // OCR: Output Compare Register
        OCR2A = 224;
    } else {
#if TIMER_INTERRUPT_DIAGNOSTIC_2
        avg_display_time += micros() - start;
        avg_display_time = avg_display_time >> 1;
        start = micros();
#endif
        // blank_display
        PORTD &= ~_BV(PORTD6);

        // State is blanking
        blanking = true;

        // Set the timer to blank for, e.g., 100uS. See above
        OCR2A = 24;
    }

#if TIMER_INTERRUPT_DIAGNOSTIC
    PORTD &= ~TIMER_INTERRUPT_TEST_PIN;
#endif
}

void old_setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);
    Serial.println("boot");

    // Set all ports to be outputs
    // Initialize all I/O pins to output, then one for the analog input
    DDRD = B11111111;
    DDRC = B00111111;
    DDRB = B00111111;

    pinMode(A0, INPUT);

    // Use Timer1 for the HV PS control signal
    // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
    // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    // Set the pre-scaler at 1 (31.25 kHz) and WGM bit 2
    TCCR1B = _BV(WGM12) | _BV(CS10);

    // Start out with low voltage
    OCR1A = 0x010;  // 9-bit resolution --> 0x0000 - 0x01FF
}

void loop() {
}
