#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

#define OUT_PIN PB3
#define DATA_PIN PD2
#define LATCH_PIN PD3
#define CLOCK_PIN PD4
#define BUZZER_PIN PD5

#define UP_PIN PB0
#define DOWN_PIN PB1
#define START_PIN PB2

bool state = false;
unsigned int time = 45;
unsigned int elapsed;
bool timer_enabled = false;

unsigned int NUMS_MAP[10] = {
    //abcdefg.
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111
};

void pinHigh(volatile uint8_t *port, int pin) {
  *port |= (1 << pin);
}

void pinLow(volatile uint8_t *port, int pin) {
  *port &= ~(1 << pin);
}

void init_led() {
  DDRD |= ((1 << DATA_PIN) | (1 << CLOCK_PIN) | (1 << LATCH_PIN));
}

void init_buzzer() {
  DDRD |= (1 << BUZZER_PIN);
  // Configure counter for PWM. Enable PWM, set fast PWM, use external crystal
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00) | (1 << CS02) | (1 << CS01) | (1 << CS00);
}

void init_buttons() {
  DDRB &= (~(1 << UP_PIN) & ~(1 << DOWN_PIN) & ~(1 << START_PIN));
  PORTB |= ((1 << START_PIN) | (1 << DOWN_PIN) | (1 << START_PIN));
}

void init_out() {
  DDRB |= (1 << OUT_PIN);
  PORTB &= ~(1 << OUT_PIN);
}

void setup() {
  PORTB = 0b00000000;
  
  init_buttons();
  init_led();
  init_buzzer();
  init_out();
}

void put_byte(int n, bool invert) {
  int bits = NUMS_MAP[n];
  if (invert) {
    bits = ~bits;
  }

  for (int i=0; i < 8; i++) {
    //Output the data on DS line according to the
    //Value of MSB
    if (bits & 0b10000000) {
        //MSB is 1 so output high
        pinHigh(&PORTD, DATA_PIN);
    } else {
        //MSB is 0 so output high
        pinLow(&PORTD, DATA_PIN);
    }
    pinHigh(&PORTD, CLOCK_PIN);
    pinLow(&PORTD, CLOCK_PIN);
    bits = bits << 1;  //Now bring ne0t bit at MSB position
  }
}

void swow_number(int n) {

  pinLow(&PORTD, LATCH_PIN);

  put_byte(n % 10, false);
  put_byte(n / 10, false);
  put_byte(n / 100, false);
  //Now all 8 bits have been transferred to shift register
  //Move them to output latch at one
  pinHigh(&PORTD, LATCH_PIN);
}

bool button_up_pressed() {
  return (PINB & (1 << UP_PIN)) == 0;
}

bool button_down_pressed() {
  return (PINB & (1 << DOWN_PIN)) == 0;
}

bool button_start_pressed() {
  return (PINB & (1 << START_PIN)) == 0;
}

void inc_time() {
  if (time < 99) {
    time++;
  }
}

void dec_time() {
  if (time > 0) {
    time--;
  }
}

const int Note_C  = 239;
const int Note_CS = 225;
const int Note_D  = 213;
const int Note_DS = 201;
const int Note_E  = 190;
const int Note_F  = 179;
const int Note_FS = 169;
const int Note_G  = 159;
const int Note_GS = 150;
const int Note_A  = 142;
const int Note_AS = 134;
const int Note_B  = 127;

void TinyTone(unsigned char divisor, unsigned char octave, unsigned long duration)
{
  // Start PWM output
  TCCR0B = (1 << CS01); 
  OCR0B  = divisor-1;
  delay(duration);
  // Stop PWM output
  TCCR0B = 0x90;
}

void tick() {
  if (elapsed > 0) {
    elapsed--;
  } else {
    timer_enabled = false;
    pinLow(&PORTB, OUT_PIN);
    TinyTone(Note_A, 4, 2000);
    TinyTone(Note_G, 4, 2000);
    TinyTone(Note_F, 4, 2000);
    TinyTone(Note_E, 4, 2000);
    TinyTone(Note_D, 4, 2000);
  }

  swow_number(elapsed);
  _delay_ms(1000);
}

void loop() {
  if (timer_enabled) {
    tick();
  } else {
    if (button_up_pressed()) {
      inc_time();
    }
    if (button_down_pressed()) {
      dec_time();
    }
    if (button_start_pressed()) {
        timer_enabled = true;
        elapsed = time;
        pinHigh(&PORTB, OUT_PIN);
    }
    
    swow_number(time);
    _delay_ms(200);
  }

}

int main(void) {
  setup();

  while(1) {
    loop();
  }
}
