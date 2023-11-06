// H-bridge Sine Wave Inverter
// Written for the Atmel ATMega328P

volatile const uint16_t period = 4167; // 3840 Hz
volatile const uint16_t duty_ref[32] = {408 , 812 , 1209, 1594, 1964, 2315, 2643, 2946,
                                        3221, 3464, 3674, 3849, 3987, 4086, 4146, 4167,
                                        4167, 4146, 4086, 3987, 3849, 3674, 3464, 3221,
                                        2946, 2643, 2315, 1964, 1594, 1209, 812 , 408 }; //Half sine wave values

volatile uint16_t* duty = (uint16_t*) calloc(32, sizeof(uint16_t));

volatile uint8_t n = 0;

volatile uint8_t mag = 0, mag_last = 0;

void setup() {
  PORTB &= !((1 << PORTB2) | (1 << PORTB1));
  DDRB  |=  ((1 << PORTB2) | (1 << PORTB1));
  for(uint8_t i = 0;i < 32;i++) {
    duty[i] = duty_ref[i];
  }
  cli();
  TCCR1A = 0x00 | ((1 << WGM11) | (1 << COM1A1)); // Fast PWM Mode 14, OC1A connected
  TCCR1B = 0x00 | ((1 << WGM13) | (1 << WGM12));  // Fast PWM Mode 14
  TIMSK1 = 0x00 | (1 << TOIE1);                   // Enable Timer 1 Overflow Interrupt
  ICR1 = period;                                  
  OCR1A = duty[0];
  OCR1B = duty[0];
  TCNT1 = 1;
  TCCR1B |= (1 << CS10);                          // Prescalar = 1
  sei();
  return;
}

void loop() {
/*
  for(uint8_t i = 0;i < 32;i++) {
    mag = i;
    delay(500);
  }
*/
  return;
}

ISR(TIMER1_OVF_vect) {
/*
  if(mag != mag_last & ((n & 0x0F) == 0x00)) {
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
    for(uint8_t i = 0;i < 32;i++) {
      duty[i] = duty_ref[i] >> 5;
      for(uint8_t j = 0;j < mag;j++) {
        duty[i] += duty[i];
      }
    }
    mag_last = mag;
    TCCR1A |= (1 << COM1A1);
  }
*/
  uint8_t index = n & 0x1F;
  if (index == 0x00) TCCR1A ^= ((1 << COM1A1) | (1 << COM1B1)); // Toggle connected port every half cycle
  OCR1A = duty[index];
  OCR1B = duty[index];
  n++;
  return;
}
