//note compile with spencekonde core (not working with teenyriot core)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <NeoSynth.h>
#include <WS2812.h>
#define LEDCount 8
#define outputPin 0
WS2812 LED(LEDCount);
cRGB value;
int sync;
int count_led;
NeoSynth synth;

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


void adc_init()
{
  ADCSRA |= _BV(ADIE); //adc interrupt enable
  ADCSRA |= _BV(ADEN); //adc enable
  ADCSRA |= _BV(ADATE); //auto trigger
//  ADCSRA |= _BV(ADPS1)|_BV(ADPS0); //prescale 128
  ADCSRA |= _BV(ADPS1);

  ADMUX  = adc3;
  ADCSRB = 0;
}

void adc_start()
{
  ADCSRA |= _BV(ADSC); //start adc conversion
}


 
int main(void)
{
    clock_prescale_set(clock_div_1);


    pinMode(A3, INPUT_PULLUP);
      synth.begin();
    synth.setupVoice(0,NOISE,60,ENVELOPE1,80,64);
  LED.setOutput(0);

  setup();
  while (1)
  {
//160
  if (pot3 > 160) {
           synth.mTrigger(0,(55));
               value.r = 255; value.g = 0; value.b = 0; // RGB Value -> Blue
    LED.set_crgb_at(count_led, value); // Set value at LED found at index 0

    value.r = 0; value.g = 0; value.b = 0; // RGB Value -> Blue
    LED.set_crgb_at(count_led - 1, value); // Set value at LED found at index 0

    count_led += 1;
    if (count_led > 8) count_led = 0;

    LED.sync();
    synth.delay(10);
  }
  // OCR1C = (pot1>>4);

  } //end while
  return 0;
}



void setup() {
//timer_init();// initialize timer & Pwm
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
    ADMUX = adc2;
  }
  else if ( ADMUX == adc2) {
    pot2  = val;    
    ADMUX = adc3;
  }
  else if ( ADMUX == adc3) {
    pot1  = val;
    ADMUX = adc1;
  }



}

