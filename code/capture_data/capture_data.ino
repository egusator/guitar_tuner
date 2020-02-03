#include <TM1638.h>

// define a module on data pin 8, clock pin 9 and strobe pin 10
TM1638 module(8, 9, 10);
unsigned long a=1;

#define BUF_LENGTH 512

//data storage variables
byte dataBuffer[BUF_LENGTH];
int index = 0;//current storage index
byte ready = 0;

void setup(){
  
  Serial.begin(9600);
  
  cli();//disable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enable auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  sei();//enable interrupts
}

ISR(ADC_vect) {//when new ADC value ready

  byte adc = ADCH;//get value from A0
  
  if (ready == 0) {
    dataBuffer[index] = adc;

    index++;
    
    if (index == BUF_LENGTH) {
      index = 0;
      ready = 1;
    }
  }
  
}

int i = 0;

void loop(){


  module.setLED(TM1638_COLOR_RED, 1);
  module.setLED(TM1638_COLOR_RED, 4);

  i = module.getButtons();
  
  module.setDisplayToDecNumber(i,0,false);
  delay(1000);
  
}


//int i,k;
//long sum, sum_old;
//int thresh = 0;
//float freq_per = 0;
//byte pd_state = 0;
//
//  if (ready == 1) {
//
//    sum = 0;
//    sum_old = 0;
//    pd_state = 0;
//    int period = 0;
//    thresh = 0;
//    freq_per = 0;
//
//    for(i=0; i < BUF_LENGTH; i++)
//    {
//      // Autocorrelation
//      sum_old = sum;
//      sum = 0;
//      for(k=0; k < BUF_LENGTH-i; k++) sum += (dataBuffer[k]-128)*(dataBuffer[k+i]-128);
//    /*  // RX8 [h=43] @1Key1 @0Key1
//      Serial.print("C");
//      Serial.write((rawData[i]-128)>>8);
//      Serial.write((rawData[i]-128)&0xff); */
//      
//    /*  // RX8 [h=43] @1Key1 @0Key1
//      Serial.print("C");
//      Serial.write(sum>>8);
//      Serial.write(sum&0xff); */
//      
//      // Peak Detect State Machine
//      if (pd_state == 2 && (sum-sum_old) <= 0) 
//      {
//        period = i;
//        pd_state = 3;
//      }
//      if (pd_state == 1 && (sum > thresh) && (sum-sum_old) > 0) pd_state = 2;
//      if (!i) {
//        thresh = sum * 0.5;
//        pd_state = 1;
//      }
//    }
//    // Frequency identified in kHz
//    freq_per = 38400/period;
//    Serial.print(freq_per);
//    Serial.print(" ");
//    Serial.print(period);
//    Serial.print(" ");
//    Serial.println(thresh);
//
//    
//    if (freq_per > 60 && freq_per < 350 ) { 
//      module.setDisplayToDecNumber(freq_per,0,false);
//    }
//
////    if (freq_per > 1000 && freq_per < 8000) {
////      
////      for (int i = 0; i < BUF_LENGTH; i++) {
////        Serial.print(dataBuffer[i]);
////        Serial.print(", ");
////      }
////      Serial.println(" ");
////    }
//  
//    ready = 0;
//  };
//  //delay(100);//delete this if you want
//}
