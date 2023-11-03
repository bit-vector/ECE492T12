// H-bridge Sine Wave Inverter
// Written for the Atmel ATMega328P

volatile const uint16_t period = 4167; // 3840 Hz
volatile const uint16_t duty[32] = {408 , 812 , 1209, 1594, 1964, 2315, 2643, 2946,
                                    3221, 3464, 3674, 3849, 3987, 4086, 4146, 4167,
                                    4167, 4146, 4086, 3987, 3849, 3674, 3464, 3221,
                                    2946, 2643, 2315, 1964, 1594, 1209, 812 , 408 };
volatile uint8_t i = 0;
void setup() {
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  noInterrupts();
  TCCR1A = 0x00 | ((1 << WGM11) | (1 << COM1A1)); // Fast PWM Mode 14, OC1A connected
  TCCR1B = 0x00 | ((1 << WGM13) | (1 << WGM12));  // Fast PWM Mode 14
  TIMSK1 = 0x00 | (1 << TOIE1);                   // Enable Timer 1 Overflow Interrupt
  ICR1 = period;                                  // 1920 Hz
  OCR1A = duty[0];
  OCR1B = duty[0];
  TCNT1 = 1;
  TCCR1B |= (1 << CS10);                          // Prescalar = 1
  interrupts();
  return;
}

void loop() {
  return;
}

ISR(TIMER1_OVF_vect) {
  uint8_t index = i & 0x1F;
  if (index == 0x00) TCCR1A ^= ((1 << COM1A1) | (1 << COM1B1)); // Toggle connected port every half cycle
  OCR1A = duty[index];
  OCR1B = duty[index];
  i++;
  return;
}
