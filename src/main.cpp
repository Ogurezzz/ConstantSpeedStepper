#include <GyverButton.h>
#include <PWM.h>
//Pins layout
//Buttons
#define BTN_Stop 2          //Кнопка стоп. Хотя она будет активироваться прерыванием
#define BTN_Set 7           //Кнопка выбор
#define BTN_Left 8          //Стрелка влево
#define BTN_Right 5         //Стрелка вправо
GButton butLeft(BTN_Left);
GButton butRight(BTN_Right);
GButton butStop(BTN_Stop);
GButton butSet(BTN_Set);
//LEDs, LCD, Buzzer
const int LED1Pin = 15;     //Зеленый диод
const int LED2Pin = 16;     //Диод +
const int LED3Pin = 14;     //Диод -
//Stepper Driver
const int stpEnaPin = 12;   //Enable
const int stpSTPPin = 9;    //Step pin
const int stpM0Pin = 11;    //M0
const int stpM1Pin = 10;    //M1
const int stpM2Pin = 6;     //M2
const float speedList[] = {0.1,3,6,30,60,100,200,300,600};

//Параметры мотора.
const uint8_t spr = 200;              //Количество шагов на один оборот ротора.
const uint16_t maxRPM = 600;           //Максимальная скорость, которую может выдать мотор

//Виртуальные машины
//Машина вращения. Самая главная и важная
//int macStepControl = 0;     //Фаза работы машины управления шаговиками
boolean motor_flag = 0;         //Включение или выключение машины
//boolean motor;
//boolean startStop;
const int8_t PWM = 127;        //значение ШИМ для шагов. в нашем случае 50% - 127        
//int16_t current_F = 0;      //Текущая частота смены шага
//int16_t target_F = 0;       //Частота к которой стремимся.
//int16_t selected_F = 0;     //Выбранная частота.
float current_RPM = 0;
float target_RPM = 0;
float selected_RPM = 0;

//int accel;                  

//Машина расчета скорости и шага. Результат работы - корректное значение StepDelay
//int macStepCalc = 0;        //Фаза работы машины расчета скорости
//float rpm;                  //Скорость в оборотах в минуту
//int diffRPM = 0;            //Разница между заданной и рассчитанной скоростями
//int dRPM = 24;              //Компенсация шага в микросекундах, для стабильной скорости.
int microstepDiv;           //Выбор дробления микрошага (1,2,4,8,16,32)

//Машина выбора скорости. Результат - переменная rpm корорая передается в машину расчета
//int macSpeedSelect = 0;     //Фаза работы машины выбора скорости
//float tempRPM;              //Выбранная скорость для передачи в машину расчета
int rpmIncrement;           //Инкремент при выборе скорости стрелками
int rpmDecrement;           //Декремент при выборе скорости стрелками
int speed_Num=5;            //Текущий порядковый номер стандартной скорости
//unsigned long roundTime;    //Время одного оборота в мкс

//Машина компенсации вращения. Результат - рассчитанная погрешность dRPM
//int macCorrection = 0;      //Фаза работы машины коррекции.
//unsigned long prevFlash;    //Время, кода прилетел последний импульс с датчика вращения (датчик Холла)
unsigned long thisFlash;
//float realRPM;              //Рассчитанная реальная скорость вращения
//boolean correction_flag;    //Флаг о необходимости расчета коррекции. Возможно избавимся
//boolean skip_sens;          //На высоких оборотах пропускать шаг при корректировке.

//Выбор программ для вращения
//0 - Режим вкл/выкл на заданной скорости без учета времени. Любая скорость выставляется стрелками.
//1 - Аналогично режиму 0, но переключение между стандартными скоростями: 0.1 3 6 30 60 100 200 300 600 оборотов.
//2 - Режим Гель 10 секунд. В течение 10 секунт маслает на максимальных оборотах.
//3 - Режим Гель 10 минут. Крутит на минимальной скорости. 1 оборот за 10 минут.
//4 - Режим ручной настройки компенсации шага.
uint8_t modeNum = 1;
uint8_t modeTemp = 1;     

//Машина таймера программы. Выключает вращение двигателя через заданный промежуток времени.
uint32_t end_time;
boolean timer_started;
boolean program_started;

//=============================================================================================
//Процедуры

void sens() {
    float RPM= (float)60/((float)(micros()-thisFlash)/1000000);  //расчет
    thisFlash=micros();  //запомнить время последнего оборота
    Serial.println(RPM);    
}
void selectMode(int n){
  int maxMode = 4;
  if (n < 0) {
      n=maxMode;
    } else if (n > maxMode){
        n=0;
    }
  switch (n) {
    case 0:
      Serial.println("Start/Stop mode");
      modeTemp = 0;
      break;
    case 1:
      Serial.println("Standart Speeds");
      modeTemp = 1;
      break;
    case 2:
      Serial.println("10 Sec Gel");
      modeTemp = 2;
      break;
    case 3:
      Serial.println("10 Min Gel");
      modeTemp = 3;
      break;
    case 4:
      Serial.println("Speed Calibration");
      modeTemp = 4;
      break;
    }
  }

float spdSelect(boolean up){       //Установка следующей/предыдущей стандартной скорости
  int max_speed_Num = 8;
  if (up){
    ++speed_Num;
    if (speed_Num>max_speed_Num) speed_Num = max_speed_Num;
    } else {
    --speed_Num;
    if (speed_Num<0) speed_Num = 0;
    }
    return speedList[speed_Num];
  }
void spdSet(float lrpm){
  Serial.print(" RPM set: ");Serial.println(lrpm);
  selected_RPM = lrpm;
  }
int mcStpChoose(float lrpm){     //Выбор режима деления микрошага. Для низких скоростей - больше.
  int mcStp = 1;
  if (lrpm > 0.01) {
    if (lrpm <= 10){
      mcStp = 32;
      } else if (lrpm <= 20){
        mcStp = 16;
      } else if (lrpm <= 40){
        mcStp = 8;
      } else if (lrpm <= 80){
        mcStp = 4;
      } else if (lrpm <= 160){
        mcStp = 2;
      } else {
        mcStp = 1;
    }
  switch (mcStp){               //Выставляем соответствующие уровни на управляющих пинах. Значения приведены для DRV8825.
    case 1:
      digitalWrite(stpM0Pin,0);
      digitalWrite(stpM1Pin,0);
      digitalWrite(stpM2Pin,0);
      break;
    case 2:
      digitalWrite(stpM0Pin,1);
      digitalWrite(stpM1Pin,0);
      digitalWrite(stpM2Pin,0);
      break;
    case 4:
      digitalWrite(stpM0Pin,0);
      digitalWrite(stpM1Pin,1);
      digitalWrite(stpM2Pin,0);
      break;
    case 8:
      digitalWrite(stpM0Pin,1);
      digitalWrite(stpM1Pin,1);
      digitalWrite(stpM2Pin,0);
      break;
    case 16:
      digitalWrite(stpM0Pin,0);
      digitalWrite(stpM1Pin,0);
      digitalWrite(stpM2Pin,1);
      break;
    case 32:
      digitalWrite(stpM0Pin,1);
      digitalWrite(stpM1Pin,1);
      digitalWrite(stpM2Pin,1);
      break;
      }
    }
  return mcStp;
  }
int getFreq(float RPM){
  if (RPM > maxRPM){
      RPM = maxRPM;
    } else if (RPM <= 0.1 && RPM!=0) {
      RPM = 0.1;
    }
  microstepDiv = mcStpChoose(RPM); //Определение режима микростепа.
  int freq = (RPM*spr*microstepDiv)/60;    //Расчет частоты шага
  return freq;
  }
void stop() {
  motor_flag = 0;
  Serial.println("Stopped");
  selectMode(modeNum);
  }
void setup() {                  //Стандартная процедура инициализации
  //LEDs, LCD, Buzzer
  pinMode(LED1Pin, OUTPUT); //Green LED
  pinMode(LED2Pin, OUTPUT); //Red LED 1
  pinMode(LED3Pin,OUTPUT);  //Red LED 2
  //Устанавливаем Выводы для управления драйвером мотора.
  pinMode(stpEnaPin,OUTPUT);  //Enable Pin
  //pinMode(stpDIRPin,OUTPUT);  //Direction Pin
  pinMode(stpSTPPin,OUTPUT);  //Step Pin
  pinMode(stpM0Pin,OUTPUT);   //Microstep M0
  pinMode(stpM1Pin,OUTPUT);   //Microstep M1
  pinMode(stpM2Pin,OUTPUT);   //Microstep M2
  //Настраиваем прерывания
attachInterrupt(0,stop,RISING);    //Кнопка аварийной остановки.
  attachInterrupt(1,sens,FALLING); //подключить прерывание на 3 пине. Датчик Холла
  //Настраиваем скорость отработки зажатых кнопок
  butLeft.setStepTimeout(100);
  butRight.setStepTimeout(100);
  InitTimersSafe(); 
  Serial.begin(115200);
  ADCSRA = 0b11100010;
  ADMUX = 0b01100100;
  digitalWrite(stpEnaPin,HIGH);   //stop motor
  selected_RPM = speedList[speed_Num];
  Serial.println("Welcome to Viscosimeter by OGurev");
  }

void loop() {                   //Основной цикл работы МК
  butStop.tick();
  butSet.tick();
  butLeft.tick();
  butRight.tick();
  if (butSet.isHolded()) {      //Выход в меню выбора режима. При этом останавливаем мотор. Ибо нефиг :)
    motor_flag = 0;
    Serial.println("Mode Selection");
    modeNum = 99;
    }
  switch (modeNum) {            //Отработака виртуальных машин в зависимости от текущего режима. Общение с пользователем, нажатия клавиш.    
    case 0: //Стандартный режим работы (Вкл/выкл и выбор любой доступной скорости)
      if (butSet.isClick()) motor_flag = !motor_flag;
      if (selected_RPM < 1){               //Изменение инкремента
        rpmIncrement = 0.1;
        } else if (selected_RPM < 60.0) {
          rpmIncrement = 1;
          } else {
            rpmIncrement = 10;
            }
      if (selected_RPM <= 1){              //Декремента
        rpmDecrement = 0.1;
        } else if (selected_RPM <= 60.0) {
          rpmDecrement = 1;
          } else {
            rpmDecrement = 10;
            }
      if (butLeft.isClick() || butLeft.isStep()) spdSet(selected_RPM-rpmDecrement);
      if (butRight.isClick() || butRight.isStep()) spdSet(selected_RPM + rpmIncrement);
      break;
    case 1: //Режим с переключением между стандартными скоростями
      if (butSet.isClick()) motor_flag = !motor_flag;
      if (butLeft.isClick() || butLeft.isStep()) spdSet(spdSelect(0));
      if (butRight.isClick() || butRight.isStep()) spdSet(spdSelect(1));
      break;
    case 2: //Режим геля на 10 секунд.
      if (butSet.isClick()) timer_started = !timer_started;
      if (timer_started) {
          if (!program_started) {
            if (current_RPM != maxRPM) {
              spdSet(maxRPM);
            }
            Serial.println("10 sec Gel started");
            motor_flag = 1;
            program_started = 1;
            }
          if (millis()>end_time) {
            timer_started=0;
            Serial.println("10 sec Gel done");
            }
        } else {
          end_time = millis() + 10000;
          motor_flag = 0;
          program_started = 0;
        }
      break;
    case 3: //Режим геля на 10 минут
      if (butSet.isClick()) timer_started = !timer_started;
      if (timer_started) {
          if (!program_started) {
            if (current_RPM != 0.1) {
              spdSet(0.1);
            }
            Serial.println("10 min Gel started");
            motor_flag = 1;
            program_started = 1;
            }
          if (millis()>end_time) {
            timer_started=0;
            Serial.println("10 min Gel done");
            }
        } else {
          end_time = millis() + 600000;
          motor_flag = 0;
          program_started = 0;
        }
      break;
    case 4: //Режим ручной калибровки. Скорее всего откажемся от него в пользу чего-то более интересного.
      if (butSet.isClick()) motor_flag = !motor_flag;
      if (butLeft.isClick() || butLeft.isStep()) {
          selected_RPM--;
          Serial.println(selected_RPM);
        }
      if (butRight.isClick() || butRight.isStep()){
          selected_RPM++;
          Serial.println(selected_RPM);
      } 
      break;
    case 99://Mode selection самый последний режим. Собственно, режим выбора режима. :)
      if (butSet.isClick()) {
        Serial.print("Saved.");Serial.println();
        modeNum = modeTemp;
        }
      if (butLeft.isClick() || butLeft.isStep())  selectMode(modeTemp - 1);
      if (butRight.isClick() || butRight.isStep()) selectMode(modeTemp + 1);
      break;
    default://По идее такого не должно случиться. Но если что - в любой непонятной ситуации выключай мотор! :)
      if (butSet.isClick()) motor_flag = !motor_flag;
      break;
    }
  
//Плавное изменение скорости.
  if (motor_flag == 0) {    //Если мотор нужно отключить, то ставим частоту = 0
      target_RPM = 0;
    } else {
      target_RPM = selected_RPM;
    }
  if (current_RPM != target_RPM){ //Измеряем разницу частот и меняем текущую скорость.
    int mux = (target_RPM - current_RPM);
    if (abs(mux)<=1) {
      current_RPM=target_RPM;
      mux = 0;
    }
    if (mux>0){
      current_RPM++;
    }else if (mux <0){
      current_RPM--;
    }
    //current_F = getFreq(current_RPM);
    SetPinFrequencySafe(stpSTPPin, getFreq(current_RPM));
    if (target_RPM ==0 && mux == 0) {
        digitalWrite(stpEnaPin,HIGH);
      } else {
        digitalWrite(stpEnaPin,LOW);
      }
      delay(10);
    }
  pwmWrite(stpSTPPin, PWM); //Скважность 50%.
}