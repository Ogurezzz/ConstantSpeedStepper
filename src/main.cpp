//#include <GyverButton.h>
#include <PWM.h>
#include <U8glib.h>
#include <GyverEncoder.h>
//Pins layout
//Buttons
#define ENC_SW 7          //Кнопка энкодера
#define ENC_CLK 2         //Энкодер контакт CLK
#define ENC_DT 6          //Энкодер, второй вывод
Encoder enc(ENC_CLK,ENC_DT,ENC_SW,true);

//LCD, Buzzer
#define LCD_CS_RS 4
U8GLIB_ST7920_128X64_1X u8g(LCD_CS_RS);
#define REDRAW_TIMER 250  //Задержка в мс для перерисовки экрана.
#define FONT_MENU_NAME u8g_font_profont11
#define BEEPER_PIN 8
uint32_t next_print;

//Параметры мотора и его драйвера
#define DRV_ENABLE_PIN 18
#define DRV_STEP_PIN 9
#define DRV_M0_PIN 16
#define DRV_M1_PIN 15
#define DRV_M2_PIN 14
#define DRV_STEPS_PER_ROUND 200
#define DRV_MAX_SPEED 1000

const float speedList[] = {0.1,3,6,30,60,100,200,300,600,1000};

//Виртуальные машины
//Машина вращения. Самая главная и важная
boolean motor_flag = 0;     //Включение или выключение машины
#define PWM 127             //значение ШИМ для шагов. в нашем случае 50% - 127        
float current_RPM = 0;      //Текущая выбранная скорость
float target_RPM = 0;       //
float selected_RPM = 0;
float measured_RPM = 0;
int mux;

//Машина расчета скорости и шага. Результат работы - корректное значение StepDelay
int microstepDiv;           //Выбор дробления микрошага (1,2,4,8,16,32)

//Машина выбора скорости. Результат - переменная rpm корорая передается в машину расчета
int rpmIncrement;           //Инкремент при выборе скорости стрелками
int rpmDecrement;           //Декремент при выборе скорости стрелками
int speed_Num=5;            //Текущий порядковый номер стандартной скорости

uint32_t thisFlash;         //время micros() последнего сигнала с датчика Холла.
//Машина вывода на экран
 int macPrint = 0;
 String StrA = "";

//Выбор программ для вращения
//0 - Режим вкл/выкл на заданной скорости без учета времени. Любая скорость выставляется стрелками.
//1 - Аналогично режиму 0, но переключение между стандартными скоростями: 0.1 3 6 30 60 100 200 300 600 оборотов.
//2 - Режим Гель 10 секунд. В течение 10 секунт маслает на максимальных оборотах.
//3 - Режим Гель 10 минут. Крутит на минимальной скорости. 1 оборот за 10 минут.
//4 - Режим ручной настройки компенсации шага.
uint8_t modeNum = 1;
uint8_t modeTemp = 1; 
const String modeNames[] = {
    "Start/Stop",
    "Standart",
    "10 Sec Gel",
    "10 Min Gel",
    "Speed Cal"
    };   

//Машина таймера программы. Выключает вращение двигателя через заданный промежуток времени.
uint32_t end_time;
boolean timer_started;
boolean program_started;

//=============================================================================================
//Процедуры
void sens() {
    measured_RPM= (float)60/((float)(micros()-thisFlash)/1000000);  //расчет
    thisFlash=micros();  //запомнить время последнего оборота
    Serial.println(measured_RPM,4);
    //printData();    
}
void selectMode(int n){
  int maxMode = 4;
  if (n < 0) {
      n=maxMode;
    } else if (n > maxMode){
        n=0;
    }
    Serial.println(modeNames[n]);
    modeTemp = n;

/*   switch (n) {
    case 0:
      Serial.println(modeNames[n]);
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
    } */
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
     if (lrpm <= 300){
      mcStp = 32;
      } else if (lrpm <= 600){
        mcStp = 16;
      } else if (lrpm <= 1200){
        mcStp = 8;
      } else if (lrpm <= 2400){
        mcStp = 4;
      } else if (lrpm <= 4800){
        mcStp = 2;
      } else {
        mcStp = 1;
    } 
  switch (mcStp){               //Выставляем соответствующие уровни на управляющих пинах. Значения приведены для DRV8825.
    case 1:
      digitalWrite(DRV_M0_PIN,0);
      digitalWrite(DRV_M1_PIN,0);
      digitalWrite(DRV_M2_PIN,0);
      break;
    case 2:
      digitalWrite(DRV_M0_PIN,1);
      digitalWrite(DRV_M1_PIN,0);
      digitalWrite(DRV_M2_PIN,0);
      break;
    case 4:
      digitalWrite(DRV_M0_PIN,0);
      digitalWrite(DRV_M1_PIN,1);
      digitalWrite(DRV_M2_PIN,0);
      break;
    case 8:
      digitalWrite(DRV_M0_PIN,1);
      digitalWrite(DRV_M1_PIN,1);
      digitalWrite(DRV_M2_PIN,0);
      break;
    case 16:
      digitalWrite(DRV_M0_PIN,0);
      digitalWrite(DRV_M1_PIN,0);
      digitalWrite(DRV_M2_PIN,1);
      break;
    case 32:
      digitalWrite(DRV_M0_PIN,1);
      digitalWrite(DRV_M1_PIN,1);
      digitalWrite(DRV_M2_PIN,1);
      break;
      }
    }
  return mcStp;
  }
int getFreq(float RPM){
  if (RPM > DRV_MAX_SPEED){
      RPM = DRV_MAX_SPEED;
    } else if (RPM <= 0.1 && RPM!=0) {
      RPM = 0.1;
    }
  microstepDiv = mcStpChoose(RPM); //Определение режима микростепа.
  int freq = (RPM*DRV_STEPS_PER_ROUND*microstepDiv)/60;    //Расчет частоты шага
  return freq;
  }
void isr() {
    enc.tick();
  }
void printData(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(FONT_MENU_NAME);
  u8g.setPrintPos(0, 10); 
  u8g.print(String("=TRIAS VISCOSIMETER="));
  u8g.setPrintPos(0, 21);
  u8g.print(String(" Current : ") + (motor_flag==1? String(measured_RPM,4) : String("Stopped")));
  u8g.setPrintPos(0, 32);
  u8g.print(String(" Selected :") + String(selected_RPM,1) + (mux==0? String("") : (mux>0 ? String("+") : String("-"))));
  u8g.setPrintPos(0, 43);
  u8g.print((modeNum == 99? String(">") : String(" ")) + String("Mode: ") + modeNames[modeTemp]);
  //u8g.setPrintPos(0, 54);
  //u8g.print(String("Set: ") + String(target_RPM,1));
  //u8g.setPrintPos(0, 64);
  //u8g.print(String("viscosimeter V0.2"));
}
void setup() {                  //Стандартная процедура инициализации
  //LEDs, LCD, Buzzer
  pinMode(BEEPER_PIN, OUTPUT); //Beeper pin
  //Устанавливаем Выводы для управления драйвером мотора.
  pinMode(DRV_ENABLE_PIN,OUTPUT);  //Enable Pin
  pinMode(DRV_STEP_PIN,OUTPUT);  //Step Pin
  pinMode(DRV_M0_PIN,OUTPUT);   //Microstep M0
  pinMode(DRV_M1_PIN,OUTPUT);   //Microstep M1
  pinMode(DRV_M2_PIN,OUTPUT);   //Microstep M2
  //Настраиваем прерывания
  attachInterrupt(0, isr, CHANGE); //Подключить прерывание на 2 пине. Энкодер
  attachInterrupt(1,sens,FALLING); //подключить прерывание на 3 пине. Датчик Холла
  InitTimersSafe(); 
  Serial.begin(115200);
  ADCSRA = 0b11100010;
  ADMUX = 0b01100100;
  digitalWrite(DRV_ENABLE_PIN,HIGH);   //stop motor
  selected_RPM = speedList[speed_Num];
  Serial.println("Welcome to Viscosimeter by OGurev");
  }

void loop() {                   //Основной цикл работы МК
  enc.tick();
  if (enc.isHolded()) {      //Выход в меню выбора режима. При этом останавливаем мотор. Ибо нефиг :)
    motor_flag = 0;
    Serial.println("Mode Selection");
    modeNum = 99;
    }
  switch (modeNum) {            //Отработака виртуальных машин в зависимости от текущего режима. Общение с пользователем, нажатия клавиш.    
    case 0: //Стандартный режим работы (Вкл/выкл и выбор любой доступной скорости)
      if (enc.isClick()) motor_flag = !motor_flag;
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
      if (enc.isLeft()) spdSet(selected_RPM-rpmDecrement);
      if (enc.isRight()) spdSet(selected_RPM + rpmIncrement);
      break;
    case 1: //Режим с переключением между стандартными скоростями
      if (enc.isClick()) motor_flag = !motor_flag;
      if (enc.isLeft()) spdSet(spdSelect(0));
      if (enc.isRight()) spdSet(spdSelect(1));
      break;
    case 2: //Режим геля на 10 секунд.
      if (enc.isClick()) timer_started = !timer_started;
      if (timer_started) {
          if (!program_started) {
            if (current_RPM != DRV_MAX_SPEED) {
              spdSet(DRV_MAX_SPEED);
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
      if (enc.isClick()) timer_started = !timer_started;
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
      if (enc.isClick()) motor_flag = !motor_flag;
      if (enc.isLeft()) {
          selected_RPM--;
          Serial.println(selected_RPM);
        }
      if (enc.isRight()){
          selected_RPM++;
          Serial.println(selected_RPM);
      } 
      break;
    case 99://Mode selection самый последний режим. Собственно, режим выбора режима. :)
      if (enc.isClick()) {
        Serial.print("Saved.");Serial.println();
        modeNum = modeTemp;
        }
      if (enc.isLeft())  selectMode(modeTemp - 1);
      if (enc.isRight()) selectMode(modeTemp + 1);
      break;
    default://По идее такого не должно случиться. Но если что - в любой непонятной ситуации выключай мотор! :)
      if (enc.isClick()) motor_flag = !motor_flag;
      break;
    }
  
//Плавное изменение скорости.
  if (motor_flag == 0) {    //Если мотор нужно отключить, то ставим частоту = 0
      target_RPM = 0;
      digitalWrite(DRV_ENABLE_PIN,HIGH);
      measured_RPM = 0;
      current_RPM = 0;

    } else {
      target_RPM = selected_RPM;
    }
  if (current_RPM != target_RPM){ //Измеряем разницу частот и меняем текущую скорость.
    mux = (target_RPM - current_RPM);
    if (abs(mux)<=1) {
      current_RPM=target_RPM;
      mux = 0;
    }
    if (mux>0){
      current_RPM++;
    }else if (mux <0){
      current_RPM--;
    }
    
    //Установка частоты импульсов на выводе 9 + коррекция скважности.
    SetPinFrequencySafe(DRV_STEP_PIN, getFreq(current_RPM));
    if (target_RPM ==0 && mux == 0) {
        digitalWrite(DRV_ENABLE_PIN,HIGH);
      } else {
        digitalWrite(DRV_ENABLE_PIN,LOW);
      }
      delay(10);
    }
  pwmWrite(DRV_STEP_PIN, PWM); //Скважность 50%.

  if (millis()>next_print) {
    next_print = millis()+REDRAW_TIMER;
    u8g.firstPage();  
    do {
      printData();
    } while( u8g.nextPage() );
  }


}