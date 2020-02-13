#include <TM1638.h>

// назначим ножки для управления индикатором: данные - pin 8, синхра - pin 9 и строб pin 10
TM1638 module(8, 9, 10);
unsigned long a=1;

#define BUF_LENGTH 512

//Константы частот нот
#define NOTE_e  329.63
#define NOTE_B  246.94
#define NOTE_G  196.00
#define NOTE_D  146.82
#define NOTE_A  110.00
#define NOTE_E  82.41

//Максимальное отклонение частоты которое будем настраивать
#define MAX_DEVIATION 10

//Максимальные и минимальные частоты для каждой ноты
#define NOTE_e_MAX NOTE_e * (100 + MAX_DEVIATION) / 100
#define NOTE_e_MIN NOTE_e * (100 - MAX_DEVIATION) / 100
#define NOTE_B_MAX NOTE_B * (100 + MAX_DEVIATION) / 100
#define NOTE_B_MIN NOTE_B * (100 - MAX_DEVIATION) / 100
#define NOTE_G_MAX NOTE_G * (100 + MAX_DEVIATION) / 100
#define NOTE_G_MIN NOTE_G * (100 - MAX_DEVIATION) / 100
#define NOTE_D_MAX NOTE_D * (100 + MAX_DEVIATION) / 100
#define NOTE_D_MIN NOTE_D * (100 - MAX_DEVIATION) / 100
#define NOTE_A_MAX NOTE_A * (100 + MAX_DEVIATION) / 100
#define NOTE_A_MIN NOTE_A * (100 - MAX_DEVIATION) / 100
#define NOTE_E_MAX NOTE_E * (100 + MAX_DEVIATION) / 100
#define NOTE_E_MIN NOTE_E * (100 - MAX_DEVIATION) / 100

//Маска бита кнопок
#define BUTTON_S1               1
#define BUTTON_S2               2
#define BUTTON_S3               4
#define BUTTON_S4               8
#define BUTTON_S5               16
#define BUTTON_S6               32
#define BUTTON_FREQ             64
#define BUTTON_UART             128

//Номера светодиодов
#define LED_BIT_S1              0
#define LED_BIT_S2              1
#define LED_BIT_S3              2
#define LED_BIT_S4              3
#define LED_BIT_S5              4
#define LED_BIT_S6              5
#define LED_BIT_FREQ            6
#define LED_BIT_UART            7

//Маски светодиодов
#define LED_MASK_S1             1
#define LED_MASK_S2             2
#define LED_MASK_S3             4
#define LED_MASK_S4             8
#define LED_MASK_S5             16
#define LED_MASK_S6             32
#define LED_MASK_FREQ           64
#define LED_MASK_UART           128

//Ножки управления шаговым мотором
#define STEP_MOTOR_ENABLE_PIN   4
#define STEP_MOTOR_PULSE_PIN    3
#define STEP_MOTOR_DIR_PIN      2

//Константы состояния сигналов шагового мотора
#define STEP_MOTOR_ON           1
#define STEP_MOTOR_OFF          0
#define STEP_MOTOR_RIGHT        0
#define STEP_MOTOR_LEFT         1

//Биты состояния частей программы
#define STATE_UART_MESSAGES     1
#define STATE_SHOW_FREQ         2

#define ON                      1
#define OFF                     0

//data storage variables
byte dataBuffer[BUF_LENGTH];
int index = 0;//current storage index
byte ready = 0;

void setup(){

  pinMode(STEP_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(STEP_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(STEP_MOTOR_DIR_PIN, OUTPUT);
  
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

  module.setLEDs(0); //switch off all LEDs
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

float maxFreq;
float minFreq;
char indicatorBuffer[10];

byte state;

byte keys, oldKeys;
word leds = 0;

void loop(){


  menu();
 
  delay(100);
  
}

void menu() {
  boolean edit = false;
  oldKeys = keys;
  keys = module.getButtons();
  
  if (keys & BUTTON_S1) {
    maxFreq = NOTE_e_MAX;
    minFreq = NOTE_e_MIN;
    
    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S1;
    edit = true;
  } else if (keys & BUTTON_S2) {
    maxFreq = NOTE_B_MAX;
    minFreq = NOTE_B_MIN;

    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S2;
    edit = true;
  } else if (keys & BUTTON_S3) {
    maxFreq = NOTE_G_MAX;
    minFreq = NOTE_G_MIN;

    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S3;
    edit = true;
  } else if (keys & BUTTON_S4) {
    maxFreq = NOTE_D_MAX;
    minFreq = NOTE_D_MIN;

    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S4;
    edit = true;
  } else if (keys & BUTTON_S5) {
    maxFreq = NOTE_A_MAX;
    minFreq = NOTE_A_MIN;

    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S5;
    edit = true;
  } else if (keys & BUTTON_S6) {
    maxFreq = NOTE_E_MAX;
    minFreq = NOTE_E_MIN;

    leds &= LED_MASK_FREQ | LED_MASK_UART;
    leds |= LED_MASK_S6;
    edit = true;
  } else if ((keys & BUTTON_FREQ) != 0 && (oldKeys & BUTTON_FREQ) == 0) {

    state ^= STATE_SHOW_FREQ;

    if (state & STATE_SHOW_FREQ) {
      leds |= LED_MASK_FREQ;
    } else {
      leds &= ~LED_MASK_FREQ;
    }
     edit = true;
 } else if ((keys & BUTTON_UART) != 0 && (oldKeys & BUTTON_UART) == 0) {

    state ^= STATE_UART_MESSAGES;

    if (state & STATE_UART_MESSAGES) {
      leds |= LED_MASK_UART;
    } else {
      leds &= ~LED_MASK_UART;
    }
    edit = true;
  }

  if (edit) {
    module.setLEDs(leds);

//    dtostrf(maxFreq, 3, 0, indicatorBuffer);
//    dtostrf(minFreq, 3, 0, indicatorBuffer + 5);
//    indicatorBuffer[3] = ' ';
//    indicatorBuffer[4] = ' ';
//    
//
//    module.setDisplayToString(indicatorBuffer);

  }

}


void turnTuner () {
  
  digitalWrite(STEP_MOTOR_ENABLE_PIN, STEP_MOTOR_ON);

  digitalWrite(STEP_MOTOR_DIR_PIN, STEP_MOTOR_RIGHT);

  for (int i = 0; i < 500; i++) {
    digitalWrite(STEP_MOTOR_PULSE_PIN, STEP_MOTOR_ON);
    delay(1);
    digitalWrite(STEP_MOTOR_PULSE_PIN, STEP_MOTOR_OFF);
    delay(1);
  }

  digitalWrite(STEP_MOTOR_DIR_PIN, STEP_MOTOR_LEFT);

  for (int i = 0; i < 500; i++) {
    digitalWrite(STEP_MOTOR_PULSE_PIN, STEP_MOTOR_ON);
    delay(1);
    digitalWrite(STEP_MOTOR_PULSE_PIN, STEP_MOTOR_OFF);
    delay(1);
  }
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
