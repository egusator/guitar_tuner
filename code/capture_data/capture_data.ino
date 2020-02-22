#include <TM1638.h>

// назначим ножки для управления индикатором: данные - pin 8, синхра - pin 9 и строб pin 10
TM1638 module(8, 9, 10);

#define USE_UART

#define MIN_MAX_START_DIFF      150     //разница между минимумом и максимумом показаний АЦП с которой стартуем измерение частоты
#define FREQ_DIFF_COEFF         3       //коэффициент преобразования ошибки по частоте в шаги мотора
#define PROTECTION_PAUSE        50      //защитная пауза между новыми измерением и последним шагом мотора

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

#define ON                      1
#define OFF                     0

//квант времени основного цикла (в миллисекундах)
#define TICK_INTERVAL           100

//время засветки всех сегментов индикатора при старте (в тиках)
#define FLASH_TIME    10

#define ALL_LEDS_ON   0xff
#define ALL_LEDS_OFF  0

//буфер под запись данных с АЦП
#define BUF_LENGTH              512
byte dataBuffer[BUF_LENGTH];
int index = 0;                //индекс текущей записи АЦП в буфер
boolean ready = false;        //флаг готовности данных с АЦП для вычисления частоты

//переменные для хранения максимального и минимального значения АЦП
byte maxAdc = 0, minAdc = 255;

//переменные для хранения пределов реальной частоты ноты
float maxFreq = 0;
float minFreq = 0;
//частота ноты которую сейчас настраиваем
float note = 0;

//буфер под формирования сообщений на индикаторе
char indicatorBuffer[10];

//флаг только показа частоты, без вращения мотора
boolean showFreq = true;

//переменные для меню (отслеживание нажатия клавиш и состояние светодиодов)
byte keys, oldKeys;
word leds = 0;

//счетчики для высчитывания кванта времени основного меню
unsigned long prevMillis = 0, currMillis = 0;

//таймер для засветки индикатора на старте
byte flashTimer = 0;

//переменные для прерывания таймера 1 обслуживающего шаговый мотор
boolean toggle = 0;                 //для формирования меандра
word steps = 0;                     //количество шагов которые осталось произвести
word protectionPause = 0;           //счетчик для защитной паузы между вращением мотора и следующим измерением
boolean turnCompleted = true;       //флаг окончания вращения мотора
boolean invertDirection = false;    //флаг необходимости инверсии направления вращения (если колки на разных сторонах головки грифа)

void setup(){

  pinMode(STEP_MOTOR_ENABLE_PIN, OUTPUT); //ноги управлением шаговым мотором работают как выходы
  pinMode(STEP_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(STEP_MOTOR_DIR_PIN, OUTPUT);

  #ifdef USE_UART
    Serial.begin(115200); //инициализируем последовательный порт на скорость 115200
  #endif
  
  cli();//отключаем прерывания
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //инициализация регистров АЦП
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //установка опорного напряжения АЦП
  ADMUX |= (1 << ADLAR); //выравниваем показания АЦП влево чтобы читать только старшие 8 бит из ADCH
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);  //частота тактирования АЦП идет через предделитель на 32 - 16 МГц / 32 = 500 КГц
  ADCSRA |= (1 << ADATE);                 //включаем автоперезапуск
  ADCSRA |= (1 << ADIE);                  //включаем прерывание АЦП
  ADCSRA |= (1 << ADEN);                  //включаем АЦП
  ADCSRA |= (1 << ADSC);                  //запускаем первое преобразование

  //Установить прерывание таймера ~100 Гц
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;     //устанавливаем 0 в счетчик
  //настраиваем компаратор на константу для 100 Гц частоты
  OCR1A = 155;// = (16*10^6) / (100*1024) - 1 (must be <65536)
  // включаем CTC режим
  TCCR1B |= (1 << WGM12);
  //установить биты CS10 и CS12 для предделителя на 1024 
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  //включить прерывание компаратора таймера 1
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//включаем все прерывания

  module.setLEDs(LED_MASK_FREQ); //выключить все светодиоды
}

ISR(ADC_vect) {//прерывание по готовности одного измерения

  byte adc = ADCH;//берем показания с канала A0
  
  if (ready == false) {
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
        ready = true;
      }
    }
  }
}

ISR(TIMER1_COMPA_vect) {//timer1 управление шаговым двигателем
  if (showFreq) {
      digitalWrite(STEP_MOTOR_ENABLE_PIN, STEP_MOTOR_OFF);
  } else {
    if (steps != 0) {
      if (toggle) {
        digitalWrite(STEP_MOTOR_PULSE_PIN, HIGH);
        toggle = 0;
  
        steps--;
      } else {
        digitalWrite(STEP_MOTOR_PULSE_PIN, LOW);
        toggle = 1;
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
  
    if (ready) {
  
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
        ready = false;
    };
}
