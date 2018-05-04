#include <Arduino.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>

volatile unsigned long t; // long
volatile unsigned long u; // long
volatile uint8_t snd; // 0...255

volatile uint8_t pot1; // 0...255
volatile uint8_t pot2; // 0...255
volatile uint8_t pot3; // 0...255

//ADMUX ADC

volatile uint8_t adc1 = _BV(ADLAR) | _BV(MUX0); //PB2-ADC1 pot1
volatile uint8_t adc2 = _BV(ADLAR) | _BV(MUX1); //PB4-ADC2 pot2
volatile uint8_t adc3 = _BV(ADLAR) | _BV(MUX0) | _BV(MUX1); //PB3-ADC3 pot3
volatile uint8_t Buffer[256];             // Circular buffer
volatile uint8_t ReadPtr, WritePtr, LastPtr, New;

void adc_init()
{
  ADCSRA |= _BV(ADIE); //adc interrupt enable
  ADCSRA |= _BV(ADEN); //adc enable
  ADCSRA |= _BV(ADATE); //auto trigger
//  ADCSRA |= _BV(ADPS1)|_BV(ADPS0); //prescale 128
    ADCSRA |= _BV(ADPS2); //prescale 128

  ADMUX  = adc3;
  ADCSRB = 0;
}

void adc_start()
{
  ADCSRA |= _BV(ADSC); //start adc conversion
}

void timer_init()
{
  //no prescale
  clock_prescale_set(clock_div_1);

  //PWM SOUND OUTPUT - FIX
  TCCR0A |= (1 << WGM00) | (1 << WGM01); //Fast pwm

  //TCCR0A |= (1<<WGM00) ; //Phase correct pwm
  //TCCR0A |= (1<<COM0A1); //Clear OC0A/OC0B on Compare Match when up-counting.
  TCCR0A |= (1 << COM0B1); //USE PB1 --> Clear OC0A/OC0B on Compare Match when up-counting.

  TCCR0B |= (1 << CS00); //no prescale

  //TIMER1 SOUND GENERATOR @ 44100hz
  //babygnusb attiny85 clock frequency = 16.5 Mhz

  //TIMER SETUP -- FIX
  TCCR1 |= _BV(CTC1); //clear timer on compare
  TIMSK |= _BV(OCIE1A); //activate compare interruppt
  TCNT1 = 0; //init count

  //TIMER FREQUENCY
  //TCCR1 |= _BV(CS10); // prescale 1
  //TCCR1 |= _BV(CS11); // prescale 2
  TCCR1 |= _BV(CS10) | _BV(CS12); // prescale 16
  //TCCR1 |= _BV(CS11)|_BV(CS12); // prescale 32
  //TCCR1 |= _BV(CS10)|_BV(CS11)|_BV(CS12); // prescale 64
  //TCCR1 |= _BV(CS13); // prescale 128
  //TCCR1 |= _BV(CS10) | _BV(CS13); // prescale 256

  //SAMPLE RATE - FIX
//  OCR1C = 120; // (16500000/16)/8000 = 128
  //OCR1C = 45; // (16500000/16)/11025 = 93
  //OCR1C = 22; // (16500000/16)/22050 = 46
  OCR1C = 70; // (16500000/16)/44100 = 23

  // babygnusb led pin
  DDRB |= (1 << PB1); //pin connected to led

}


ISR(TIMER1_COMPA_vect)
{    
  //sound generator pwm out - FIX
  OCR0B  = Buffer[ReadPtr];
    ReadPtr = (ReadPtr + 1) & 0xFF;

}

 
int main(void)
{
  setup();
  while (1)
  {

  // OCR1C = (pot1>>4);

  } //end while
  return 0;
}



void setup() {
timer_init();// initialize timer & Pwm
adc_init(); //init adc
sei(); //enable global interrupt
adc_start(); //start adc conversion
}

void loop() {

}

ISR(ADC_vect)
{
  static uint8_t firstTime = 1;
  static uint8_t val;

  val = ADCH;

  if (firstTime == 1) {
    firstTime = 0;
  }
  else if (ADMUX  == adc1) {
    pot3 = val;    


  Buffer[LastPtr] = New;
  New = pot3 >> 1;
  Buffer[WritePtr] = (Buffer[WritePtr] + New)>>1;
  LastPtr = WritePtr;
  WritePtr = (WritePtr + 1) & 0xFF;


    ADMUX = adc2;
  }
  else if ( ADMUX == adc2) {
    pot2  = val;

    OCR1C = 20 + (pot2>>2);
    
    ADMUX = adc3;
  }
  else if ( ADMUX == adc3) {
    pot1  = val;
    ADMUX = adc1;
  }



}


