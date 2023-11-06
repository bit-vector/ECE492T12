// H-bridge Sine Wave Inverter
// Written for the Atmel ATMega328P

// Purpose
// -------
// Generate signal(s) required to drive an H-brige full sine wave inverter
// Program must be able to modify the amplitude of the output sine wave
// Frequency must be tunable to harmonics of 60 Hz
// Not sure if phase control will be required here

// Implementation
// --------------
// Current program uses 32 digital cycles per half wave
// In order to create a 60 Hz sine wave, the PWM frequency will be at 3840 Hz
//   32 cycles/half wave * 2 half waves/full wave * 60 cycles/second = 3840 Hz
//   The duty cycle of each digital cycle will be proportional to the value of
//   the sine wave at that point in time. The PWM values for a full sine wave will
//   be stored in a constant array and a working array will be used to scale the
//   values and generate the output.
// Timer 1 provides 2 outputs (OC1A/OC1B). OC1A will control the positive half wave
//   and OC1B will control the negative half wave. The OC1x pins are controlled by
//   by the OCR1x registers, which hold the PWM values. Both registers are updated
//   every cycle, but only one pin is active at a time.
// The signal generation is handled entirely in hardware, while the CPU only services
//   interrupts to update the duty cycle and the output pin.
// Magnitude control is not currently functional. It is unclear if the problem is in
//   the software or hardware. The current circuit uses BJTs, which aren't being driven
//   optimally. Testing with n-channel MOSFETs will confirm this.

volatile const uint16_t period = 4167; // 3840 Hz
volatile const uint16_t duty_ref[32] = {408 , 812 , 1209, 1594, 1964, 2315, 2643, 2946,
                                        3221, 3464, 3674, 3849, 3987, 4086, 4146, 4167,
                                        4167, 4146, 4086, 3987, 3849, 3674, 3464, 3221,
                                        2946, 2643, 2315, 1964, 1594, 1209, 812 , 408 }; //Half sine wave values

volatile uint16_t* duty = (uint16_t*) calloc(32, sizeof(uint16_t)); // Working array

volatile uint8_t n = 0;

volatile uint8_t mag = 0, mag_last = 0;

void setup() {
  PORTB &= !((1 << PORTB2) | (1 << PORTB1));      // Set initial output level to LOW
  DDRB  |=  ((1 << PORTB2) | (1 << PORTB1));      // Set both timer pins to OUTPUT
  for(uint8_t i = 0;i < 32;i++) {                 // Copy sine reference values into 
    duty[i] = duty_ref[i];
  }
  cli();                                          // Disable interrupts
  TCCR1A = 0x00 | ((1 << WGM11) | (1 << COM1A1)); // Fast PWM Mode 14, OC1A connected
  TCCR1B = 0x00 | ((1 << WGM13) | (1 << WGM12));  // Fast PWM Mode 14
  TIMSK1 = 0x00 | (1 << TOIE1);                   // Enable Timer 1 Overflow Interrupt
  ICR1 = period;                                  // Set timer to 3840 Hz
  OCR1A = duty[0];                                // Initialize OC1RA
  OCR1B = duty[0];                                // Initialize OCR1B
  TCNT1 = 1;                                      // Reset timer/counter
  TCCR1B |= (1 << CS10);                          // Connect clock, prescalar = 1
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
  uint8_t index = n & 0x1F;                                     // Limit index to 31
  if (index == 0x00) TCCR1A ^= ((1 << COM1A1) | (1 << COM1B1)); // Toggle connected port every half cycle
  OCR1A = duty[index];
  OCR1B = duty[index];
  n++;
  return;
}
