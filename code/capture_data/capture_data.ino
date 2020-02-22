#include <TM1638.h>

// назначим ножки для управления индикатором: данные - pin 8, синхра - pin 9 и строб pin 10
TM1638 module(8, 9, 10);
unsigned long a=1;

#define USE_UART

#define BUF_LENGTH              512
#define MIN_MAX_START_DIFF      150
#define FREQ_DIFF_COEFF         3
#define PROTECTION_PAUSE        50

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
#define BUTTON_DIR             128

//Номера светодиодов
#define LED_BIT_S1              0
#define LED_BIT_S2              1
#define LED_BIT_S3              2
#define LED_BIT_S4              3
#define LED_BIT_S5              4
#define LED_BIT_S6              5
#define LED_BIT_FREQ            6
#define LED_BIT_DIR            7

//Маски светодиодов
#define LED_MASK_S1             1
#define LED_MASK_S2             2
#define LED_MASK_S3             4
#define LED_MASK_S4             8
#define LED_MASK_S5             16
#define LED_MASK_S6             32
#define LED_MASK_FREQ           64
#define LED_MASK_DIR           128

//Ножки управления шаговым мотором
#define STEP_MOTOR_ENABLE_PIN   4
#define STEP_MOTOR_PULSE_PIN    3
#define STEP_MOTOR_DIR_PIN      2

//Константы состояния сигналов шагового мотора
#define STEP_MOTOR_ON           1
#define STEP_MOTOR_OFF          0
#define STEP_MOTOR_FREQ_UP      0
#define STEP_MOTOR_FREQ_DW      1

//Биты состояния частей программы
#define STATE_SHOW_FREQ         1

#define ON                      1
#define OFF                     0

#define TICK_INTERVAL           100

//data storage variables
byte dataBuffer[BUF_LENGTH];
int index = 0;//current storage index
byte ready = 0;

byte maxAdc = 0, minAdc = 255;

float maxFreq = 0;
float minFreq = 0;
float note = 0;

char indicatorBuffer[10];

boolean showFreq = true;

byte keys, oldKeys;
word leds = 0;

unsigned long prevMillis = 0, currMillis = 0;

#define FLASH_TIME    10

#define ALL_LEDS_ON   0xff
#define ALL_LEDS_OFF  0

byte flashTimer = 0;

boolean toggle1 = 0;
word steps = 0;
word protectionPause = 0;
boolean turnCompleted = true;
boolean invertDirection = false;

void setup(){

  pinMode(STEP_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(STEP_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(STEP_MOTOR_DIR_PIN, OUTPUT);

  #ifdef USE_UART
    Serial.begin(9600);
  #endif
  
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

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 156;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//enable interrupts

  module.setLEDs(LED_MASK_FREQ); //switch off all LEDs
}

ISR(ADC_vect) {//when new ADC value ready

  byte adc = ADCH;//get value from A0
  
  if (ready == 0) {
    if (maxAdc < adc) {
      maxAdc = adc;
    }

    if (minAdc > adc) {
      minAdc = adc;
    }
    if ((maxAdc - minAdc) > MIN_MAX_START_DIFF) {
      dataBuffer[index] = adc;
  
      index++;
      
      if (index == BUF_LENGTH) {
        index = 0;
        ready = 1;
      }
    }
  }
}

ISR(TIMER1_COMPA_vect) {//timer1 control of step motor
  if (showFreq) {
      digitalWrite(STEP_MOTOR_ENABLE_PIN, STEP_MOTOR_OFF);
  } else {
    if (steps != 0) {
      if (toggle1) {
        digitalWrite(STEP_MOTOR_PULSE_PIN, HIGH);
        toggle1 = 0;
  
        steps--;
      } else {
        digitalWrite(STEP_MOTOR_PULSE_PIN, LOW);
        toggle1 = 1;
      }

      protectionPause = PROTECTION_PAUSE;
    } else {
      digitalWrite(STEP_MOTOR_ENABLE_PIN, STEP_MOTOR_OFF);
      
      if (protectionPause != 0) {
        protectionPause--;
      } else {
        turnCompleted = true;
      }
    }
  }
}

void loop(){
  currMillis = millis();
  
  if (currMillis - prevMillis > TICK_INTERVAL) {
    prevMillis = currMillis;

    if (flashTimer == 0) {
      module.setLEDs(ALL_LEDS_ON);
      module.setDisplayToString("88888888", ALL_LEDS_ON);
      
      flashTimer++;
    } else if (flashTimer < FLASH_TIME) {
      flashTimer++;
    } else if(flashTimer == FLASH_TIME) {
      module.setLEDs(LED_MASK_FREQ);
      module.setDisplayToString("        ", ALL_LEDS_OFF);
      flashTimer++;
    } else {
      menu();
      calcFreq();
    }
  }
}

void menu() {
  boolean edit = false;
  oldKeys = keys;
  keys = module.getButtons();
  
  if (keys & BUTTON_S1) {
    showFreq = false;

    maxFreq = NOTE_e_MAX;
    minFreq = NOTE_e_MIN;
    note = NOTE_e;
    
    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S1;
    edit = true;
  } else if (keys & BUTTON_S2) {
    showFreq = false;

    maxFreq = NOTE_B_MAX;
    minFreq = NOTE_B_MIN;
    note = NOTE_B;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S2;
    edit = true;
  } else if (keys & BUTTON_S3) {
    showFreq = false;

    maxFreq = NOTE_G_MAX;
    minFreq = NOTE_G_MIN;
    note = NOTE_G;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S3;
    edit = true;
  } else if (keys & BUTTON_S4) {
    showFreq = false;

    maxFreq = NOTE_D_MAX;
    minFreq = NOTE_D_MIN;
    note = NOTE_D;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S4;
    edit = true;
  } else if (keys & BUTTON_S5) {
    showFreq = false;

    maxFreq = NOTE_A_MAX;
    minFreq = NOTE_A_MIN;
    note = NOTE_A;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S5;
    edit = true;
  } else if (keys & BUTTON_S6) {
    showFreq = false;

    maxFreq = NOTE_E_MAX;
    minFreq = NOTE_E_MIN;
    note = NOTE_E;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_S6;
    edit = true;
  } else if (keys & BUTTON_FREQ) {
    showFreq = true;

    leds &= LED_MASK_DIR;
    leds |= LED_MASK_FREQ;
     edit = true;
 } else if ((keys & BUTTON_DIR) != 0 && (oldKeys & BUTTON_DIR) == 0) {

    if (invertDirection) {
      invertDirection = false;
      leds &= ~LED_MASK_DIR;
    } else {
      invertDirection = true;
      leds |= LED_MASK_DIR;
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

void turnMotor (word st, byte dir) {
  cli();
  digitalWrite(STEP_MOTOR_ENABLE_PIN, STEP_MOTOR_ON);
  digitalWrite(STEP_MOTOR_DIR_PIN, dir);

  turnCompleted = false;
  steps = st;
  sei();
}

void calcFreq() {
  int i,k;
  long sum, sumOld;
  int thresh = 0;
  float freqPer = 0;
  byte pdState = 0;

  word stps;
  
    if (ready == 1) {
  
      sum = 0;
      sumOld = 0;
      pdState = 0;
      int period = 0;
      thresh = 0;
      freqPer = 0;
  
      for(i=0; i < BUF_LENGTH; i++)
      {
        // Autocorrelation
        sumOld = sum;
        sum = 0;
        for(k=0; k < BUF_LENGTH-i; k++) sum += (dataBuffer[k]-128)*(dataBuffer[k+i]-128);
      /*  // RX8 [h=43] @1Key1 @0Key1
        Serial.print("C");
        Serial.write((rawData[i]-128)>>8);
        Serial.write((rawData[i]-128)&0xff); */
        
      /*  // RX8 [h=43] @1Key1 @0Key1
        Serial.print("C");
        Serial.write(sum>>8);
        Serial.write(sum&0xff); */
        
        // Peak Detect State Machine
        if (pdState == 2 && (sum-sumOld) <= 0) 
        {
          period = i;
          pdState = 3;
        }
        if (pdState == 1 && (sum > thresh) && (sum-sumOld) > 0) pdState = 2;
        if (!i) {
          thresh = sum * 0.5;
          pdState = 1;
        }
      }
      // Frequency identified in kHz
      freqPer = 38400/period;
      
      if (showFreq) {
        module.setDisplayToDecNumber(freqPer,0,false);
      } else {
        if (freqPer > minFreq && freqPer < maxFreq ) { 
          module.setDisplayToDecNumber(freqPer,0,false);
          if (turnCompleted) {
            if (freqPer > note) {
              stps = (freqPer - note) * FREQ_DIFF_COEFF;
              if (invertDirection) {
                turnMotor(stps, STEP_MOTOR_FREQ_UP);
              } else {
                turnMotor(stps, STEP_MOTOR_FREQ_DW);
              }
            } else {
              stps = (note - freqPer) * FREQ_DIFF_COEFF;
              if (invertDirection) {
                turnMotor(stps, STEP_MOTOR_FREQ_DW);
              } else {
                turnMotor(stps, STEP_MOTOR_FREQ_UP);
              }
            }
          }
        }
      }
      
      #ifdef USE_UART
        Serial.print(freqPer);
        Serial.print(" ");
        Serial.print(period);
        Serial.print(" ");
        Serial.print(thresh);
        Serial.print(" ");
        Serial.print(maxAdc);
        Serial.print(" ");
        Serial.print(minAdc);
        Serial.print(" ");
        Serial.print(maxFreq);
        Serial.print(" ");
        Serial.print(minFreq);
        Serial.print(" ");
        Serial.print(note);
        Serial.print(" ");
        Serial.println(stps);
      #endif

  
  //    if (freqPer > 1000 && freqPer < 8000) {
  //      
  //      for (int i = 0; i < BUF_LENGTH; i++) {
  //        Serial.print(dataBuffer[i]);
  //        Serial.print(", ");
  //      }
  //      Serial.println(" ");
  //    }
          
        maxAdc = 0;
        minAdc = 255;
        ready = 0;

    };
}
