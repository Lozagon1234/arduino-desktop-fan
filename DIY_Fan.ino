/*
    Controlador de un ventilador de PC por PWM para su uso como desktop fan.
    Incluye funciones para controlar un servo que modifica la direccion del aire.
    Tiene opcion de usar un sensor de temperatura y humedad (DHT11, DHT22 o similar).
    Recibe ordenes por IR para una mayor comodidad al poder usarlo a distancia.
    ----------------------------------
    Esquema de conexiones (para Arduino Nano)
    -D2: Servo
    -D3: Mosfet control
    -D4: IR data pin
    -D5: Fan RPM tach
    -D9: Fan PWM signal
    ----------------------------------
    gb.
    24/02/2023
*/
//Incluimos las librerias necesarias
#define DECODE_NEC              //Definimos el protocolo de comunicacion IR (debe hacerse antes)
#include <IRremote.hpp>         //Librería IR
#include <Adafruit_SSD1306.h>   //OLED
#include <Adafruit_GFX.h>
#include <ClosedCube_HDC1080.h> //HDC1080
//#include <Adafruit_CCS811.h>    //CCS811

//Definimos variables
#define irPin 4             //Pin IR receiver
#define fan_control_pin 9   //Pin FAN
#define pinMosfet 3         //Pin mosfet
#define tach 5              //RPM pin
unsigned int rpm;           //Variable donde se guardan las RPM calculadas
byte servoPin;              //Variable donde se guarda el pin del servo motor
float servoPos;             //Posicion del servo
byte isMovement;            //Flag para marcar movimiento pendiente
#define servoSpeed  5       //Velocidad del movimiento(menor es mayor)
float posSmooth,posPrev;
float temp,hum,CO2,TVOC,tempA;
#define tempHum_updateInterval 60000 //Tiempo de actualizacion en ms (cada min)
#define screenRefreshInterval 15000

  //CONFIGURACION SERVO
  #define servo_Pin 2         //Pin del servo motor
  #define servo_MED_POS 150   //Definimos la posicion horizontal de las aletas (en grados, de 0 a 180). En esta posicion se enciende el sistema.
  #define servo_MAX_POS 160   //Posicion max
  #define servo_MIN_POS 120   //Posicion min
  #define servo_CLOSED_POS 100//Posicion cerrado
  #define servo_CHANGE_POS 10 //Delta posicion en grados
  #define swingSpeed 200      //Velocidad de barrido de las fins
  #define posStep 2           //Cambio posicion de barrido

byte fanSpeed; //En %
  //CONFIGURACION VENTILADOR
  #define fan_CHANGE_SPEED 25 //Porcentaje de cambio en la velocidad del ventilador por pulsacion en el mando
  #define fan_START_SPEED  50 //Velocidad inicial del ventilador en %
  #define TEMP_CUTOFF 25      //Temperatura para la que el ventilador ya no es necesario y se puede apagar (MODO AUTO)
  #define TEMP_MAX_SPEED 31   //Temperatura para la que el ventilador se pone al 100% (MODO AUTO)

unsigned long tiempo,time,tiempo2,tiempo3,tiempo4,tiempo5,tiempo6,tiempo7,tiempo8,currentMillis; //Contadores de tiempo
bool powerState = 0;                               //Marcador de encendido/apagado
bool isSwing = 0;                                  //Marcador del movimiento de fins
bool initMode;                                     //Marcador del estado de los fins al inicio (subir o bajar)
bool isScreen = 1;                                 //Marcador para encender o apagar pantalla informativa
bool isAuto = 0;                                   //Marcador para el control automático del ventilador
bool timerMode = 0;                                //Marcador para el modo con temporizador
bool timeChange = 0;                               //Marcador para permitir cambio del tiempo restante
unsigned long setTime = 0;                         //Valor del temporizador
byte horas,minutos;
#define timerStepInterval 30                       //Cambio por pulsacion en el temporizador (minutos)
#define screen_saver_timer 5                       //Tiempo para el apagado automatico de la pantalla (minutos)
float forceUpdateAdd;

//Definimos la frecuencia de la señal PWM

const word PWM_FREQ_HZ = 25000;                   //Ajustar este valor para ajustar la frecuencia
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

//OLED
#define ancho 128
#define alto 64
#define OLED_RESET 0
Adafruit_SSD1306 oled(ancho, alto, &Wire, OLED_RESET);
//Tamaño de letra
#define tam 1
//Fuente de letra
#include <Fonts/FreeSans9pt7b.h>

//CCS
//Adafruit_CCS811 ccs;
//HDC1080
ClosedCube_HDC1080 hdc1080;

//Añadimos los iconos de temperatura y humedad
//Humedad(TAM 20,25)
const unsigned char PROGMEM humLogo[] = {
  0x00,0x40,0x00,
  0x00,0x40,0x00,
  0x00,0xA0,0x00,
  0x00,0xA0,0x00,
  0x01,0x10,0x00,
  0x01,0x10,0x00,
  0x02,0x08,0x00,
  0x02,0x08,0x00,
  0x04,0x04,0x00,
  0x04,0x04,0x00,
  0x08,0x02,0x00,
  0x08,0x02,0x00,
  0x10,0x01,0x00,
  0x10,0x01,0x00,
  0x20,0x00,0x80,
  0x20,0x02,0x80,
  0x20,0x02,0x80,
  0x20,0x04,0x80,
  0x10,0x09,0x00,
  0x10,0x31,0x00,
  0x08,0x02,0x00,
  0x06,0x0C,0x00,
  0x01,0xF0,0x00,
  0x00,0x00,0x00,
  0x00,0x00,0x00
};
//Temperatura(TAM 20,25)
const unsigned char PROGMEM tempLogo[] = {
  0x00,0x00,0x00,
  0x00,0x60,0x00,
  0x00,0x90,0x00,
  0x01,0x08,0x00,
  0x01,0x0F,0x00,
  0x01,0x0F,0x00,
  0x01,0x08,0x00,
  0x01,0x0F,0x00,
  0x01,0x0F,0x00,
  0x01,0x08,0x00,
  0x01,0x08,0x00,
  0x01,0x08,0x00,
  0x01,0x08,0x00,
  0x02,0x04,0x00,
  0x02,0x04,0x00,
  0x04,0x02,0x00,
  0x04,0x02,0x00,
  0x08,0x01,0x00,
  0x08,0x05,0x00,
  0x08,0x05,0x00,
  0x08,0x09,0x00,
  0x04,0x12,0x00,
  0x04,0x62,0x00,
  0x02,0x04,0x00,
  0x01,0xF8,0x00
};
//Power logo
const unsigned char PROGMEM powerLogo[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x07,0xFF,0xF9,0xFF,0xFE,0x7F,0xFF,0x9F,0xFF,0xE0,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

//Añadimos en primer lugar las funciones necesarias

//Funcion para PWM freq
void setPWMfreq()
{
  // Clear Timer1 control and count registers
  TCCR1A = 0; // undo the configuration done by...
  TCCR1B = 0; // ...the Arduino core library
  TCNT1  = 0; // reset timer

  // Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)
  
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
} //setPWMfreq

void setPWMDuty(byte duty) 
{
  if(duty == 0)
  {
    analogWrite(pinMosfet,0);
  }
  else
  {
    analogWrite(pinMosfet,255);
    OCR1A = (word) (duty*TCNT1_TOP)/100;
  }
  //Serial.println(duty);
} //setPwmDuty

int getRPM() 
{
 time = pulseIn(tach, HIGH);
 if(time == 0)
 { rpm = 0; }
 else
 { rpm = (1000000 * 60) / (time * 4); }
 return rpm;
} //getRPM

void moveServo(float pos)
{
  isMovement = 1; //Marcamos que se ha solicitado movimiento
  //Suavizamos el movimiento
  pos = pos * 100;
  posSmooth = (0.20*pos)+(0.80*posPrev);
  posPrev = posSmooth;
  //Mostramos en terminal valores (Debugging)
  //Serial.print("Pos: ");
  //Serial.println(posSmooth/100);
  //Serial.println(pos/100);
  //Comprobamos si ha terminado el movimiento
  if(abs(pos-posSmooth) <= 3)
  {
    isMovement = 0;
    //Serial.println("Terminado");
  }
  //Movemos el motor
  softServoWrite(posSmooth/100, 0);
} //Funcion SERVO sin uso de timers

void softServoAttach(byte pin)
{
  servoPin = pin;
  pinMode(pin, OUTPUT);
}

//Writes given angle to servo for given delay in milliseconds
void softServoWrite(int angle, long servoDelay)
{
  int pulsewidth = map(angle, 0, 180, 544, 2200); // width in microseconds
  do {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin, LOW);
    delay(20); // wait for 20 milliseconds
    servoDelay -= 20;
    //Serial.println(pulsewidth);
  } 
  while(servoDelay >=0);
}

//Movimiento de las fins
float moveSwing(float initPos)
{
  float finalPos;
  if(initMode == 0) //Sumar
  {
    finalPos = initPos + posStep;
  }
  else
  {
    finalPos = initPos - posStep;
  }
  if(finalPos >= servo_MAX_POS) initMode = 1;
  else if(finalPos <= servo_MIN_POS) initMode = 0;
  return finalPos;
}

//Convertir ms en HH:MM
void convertToHours(unsigned long timeRemainingInMs)
{
  horas = ((timeRemainingInMs/1000L)/60L)/60L;
}

void convertToMin(unsigned long timeRemainingInMs)
{
  minutos = ((timeRemainingInMs/1000L)/60L) - horas*60L;
}

//Codigo
void setup()
{
  //Serial.begin(9600);       //Iniciamos el serial (opcional)
  setPWMfreq();             //Definimos la frecuencuencia del PWM
  //ccs.begin();              //Iniciamos el CCS811 (opcional)
  IrReceiver.begin(irPin);  //Iniciamos el receptor IR
  hdc1080.begin(0x40);      //Iniciamos el HDC1080
  //hdc1080.setResolution(HDC1080_RESOLUTION_8BIT, HDC1080_RESOLUTION_8BIT);
  softServoAttach(servo_Pin);
  //moveServo(servo_CLOSED_POS); //Con esto hace ruidos raros al inicio xd
  digitalWrite(tach, HIGH);
  //Definimos los pines
  pinMode(fan_control_pin, OUTPUT);
  pinMode(pinMosfet, OUTPUT);
  setPWMDuty(0);
  //Iniciamos OLED
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextSize(tam);         //Tamano de letra
  oled.setFont(&FreeSans9pt7b);  //Fuente 
  oled.setTextColor(SSD1306_WHITE);
  oled.clearDisplay();
  oled.display();
  //Forzamos primera medida de temp y hum
  tiempo7 = tempHum_updateInterval;
  //Forzamos que se encienda la pantalla
  forceUpdateAdd = 15000;
} //setup

void loop()
{
  //Tomamos el tiempo actual
  currentMillis = millis();
  //Funcion para el control por mando IR
  if(IrReceiver.decode())
  {
    //IrReceiver.printIRResultShort(&Serial);
    //Serial.println();
    if(IrReceiver.decodedIRData.flags == 0x01); //Do nothing, debouncing repeated signal within 100ms
    else if(IrReceiver.decodedIRData.protocol == UNKNOWN);
    else //Check functions
    {
      if(isScreen == 1)
      {
        forceUpdateAdd = screenRefreshInterval; //Forzamos la actualizacion de la pantalla
        tiempo8 = currentMillis; //Screen saver starts counting from now
      }
      //IrReceiver.printIRResultShort(&Serial);
      //Serial.println();
      if(IrReceiver.decodedIRData.command == 0x1C) //Encendido - apagado
      {
        powerState = !powerState;
        //start();
        switch(powerState)
        {
          case 0: fanSpeed = 0;
                  setPWMDuty(fanSpeed); //Apagado
                  isSwing = 0;
                  isAuto = 0;
                  timerMode = 0;
                  servoPos = servo_CLOSED_POS;
                  moveServo(servoPos);
                  break;
          case 1: fanSpeed = fan_START_SPEED;
                  setPWMDuty(fanSpeed); //Encendido
                  servoPos = servo_MED_POS;
                  //Serial.println(servoPos);
                  moveServo(servoPos);
                  break;
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x18) //Subir fins
      {
        if(powerState == 1) //Solo funciona si esta ON
        {
          if((timerMode == 1)&&(timeChange == 1)) //Si modo temporizador
          {
            setTime = setTime + (timerStepInterval * 60L * 1000L); //Sumamos 30 min al temporizador
            tiempo6 = currentMillis;
          }
          else if(isSwing == 0) //Si no hay swing
          {
            servoPos = servoPos + servo_CHANGE_POS;
            if(servoPos >= servo_MAX_POS)
            {
              servoPos = servo_MAX_POS;
              moveServo(servoPos);
            }
            else
            {
              moveServo(servoPos);
            }
          }
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x52) //Bajar fins
      {
        if(powerState == 1) //Solo funciona si esta ON
        {
          if((timerMode == 1)&&(timeChange == 1)) //Si modo temporizador
          {
            setTime = setTime - (timerStepInterval * 60L * 1000L); //Restamos 30 min al temporizador
            //Nos aseguramos de que no baje de 1 hora
            if(setTime <= 3600*1000L)
            {
              setTime = 3600*1000L;
            }
            tiempo6 = currentMillis;
          }
          else if(isSwing == 0) //Si no hay swing
          {
            servoPos = servoPos - servo_CHANGE_POS;
            if(servoPos <= servo_MIN_POS)
            {
              servoPos = servo_MIN_POS;
              moveServo(servoPos);
            }
            else
            {
              moveServo(servoPos);
            }
          }
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x5A) //Subir velocidad ventilador
      {
        //Primero comprobamos que podemos modificar la velocidad del ventilador
        if(isAuto); //No debemos alterar el modo auto. No hacer nada
        else
        {
          fanSpeed = fanSpeed + fan_CHANGE_SPEED;
          if(powerState == 0)
          {
            powerState = 1;
            fanSpeed = fan_START_SPEED;
            servoPos = servo_MED_POS;
            moveServo(servoPos);
          }
          if(fanSpeed >= 100)
          {
            fanSpeed = 100;
            setPWMDuty(fanSpeed);
          }
          else
          {
            setPWMDuty(fanSpeed);
          }
          //Serial.print("FAN SPEED: ");
          //Serial.println(fanSpeed);
        }
        
      }
      else if(IrReceiver.decodedIRData.command == 0x8) //Bajar velocidad ventilador
      {
        //Primero comprobamos que podemos modificar la velocidad del ventilador
        if(isAuto); //No debemos alterar el modo auto. No hacer nada
        else
        {
          fanSpeed = fanSpeed - fan_CHANGE_SPEED;
          if(fanSpeed <= 0)
          {
            fanSpeed = 0;
            setPWMDuty(fanSpeed); //Apagamos
            powerState = 0; //Marcamos que esta apagado
            isSwing = 0;    //Paramos el swing (si lo hay)
            servoPos = servo_CLOSED_POS;
            moveServo(servoPos); //Cerramos las fins
          }
          else
          {
            setPWMDuty(fanSpeed);
          }
          //Serial.print("FAN SPEED: ");
          //Serial.println(fanSpeed);
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x16) //FinSwing
      {
        if(powerState == 1) //Solo funciona si esta ON
        {
          isSwing = !isSwing;
          //Decidimos si empezamos subiendo o bajando en funcion de la posicion inicial
          if(abs(servo_MIN_POS - servoPos) <= abs(servo_MAX_POS - servoPos))
          {
            initMode = 0; //Empezamos sumando
          }
          else
          {
            initMode = 1; //Restamos en otro caso
          }
        }
      }
      else if(IrReceiver.decodedIRData.command == 0xD) //Screen ON-OFF
      {
        isScreen = !isScreen;
        if(isScreen == 0)
        {
          oled.clearDisplay();
          oled.display();
        }
        else
        {
          tiempo8 = currentMillis; //Screen saver starts counting from now
          forceUpdateAdd = screenRefreshInterval; //Forzamos la actualizacion de la pantalla
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x45) //Modo auto
      {
        isAuto = !isAuto;
        timerMode = 0; //Desactivamos el otro programa si esta activado
        if(powerState == 0) //Si esta apagado encendemos el ventilador
        {
          powerState = 1;
          servoPos = servo_MED_POS;
          moveServo(servoPos);
        }
        if(isAuto == 0)
        {
          fanSpeed = fan_START_SPEED;
          setPWMDuty(fanSpeed);
        }
        else
        {
          tiempo4 = -60000; //Forzamos que entre al bucle
        }
      }
      else if(IrReceiver.decodedIRData.command == 0x46) //Modo timer
      {
        timerMode = !timerMode;
        isAuto = 0; //Desactivamos el otro programa si esta activado
        if(powerState == 0) //Si esta apagado encendemos el ventilador
        {
          powerState = 1;
          servoPos = servo_MED_POS;
          moveServo(servoPos);
        }
        if(timerMode == 1)
        {
          setTime = 3600*1000L; //Partimos de 1 hora (en ms)
          timeChange = 1; //Permitimos el cambio de valores por un tiempo
          tiempo5 = currentMillis;
          tiempo6 = currentMillis;
        }
        else
        {
          fanSpeed = fan_START_SPEED;
          setPWMDuty(fanSpeed);
        }
      }
    }
    IrReceiver.resume(); //Enable receiving of the next value
  }

  //Medida temperatura y humedad (Medimos tambien CO2 y VOCs) (Por separado para poder controlar la frecuencia de actualizacion)
  if((unsigned long)(currentMillis - tiempo7) >= tempHum_updateInterval)
  {
    tiempo7 = currentMillis;
    temp = hdc1080.readTemperature();
    hum = hdc1080.readHumidity();
    /*if(ccs.available())
    {
      CO2 = ccs.geteCO2();
      TVOC = ccs.getTVOC();
    }*/
  }

  //Funcion para el movimiento de servo sin bloqueo
  if((isMovement == 1)&&((unsigned long)(currentMillis - tiempo) >= servoSpeed))
  {
    tiempo = currentMillis;
    moveServo(servoPos);
  }

  //Funcion del swingFin
  if((isSwing == 1)&&((unsigned long)(currentMillis - tiempo2) >= swingSpeed))
  {
    tiempo2 = currentMillis;
    servoPos = moveSwing(servoPos);
    moveServo(servoPos);
  }

  //Modo auto
  if((isAuto == 1)&&((unsigned long)(currentMillis - tiempo4) >= 60000)) //Refresh RPM cada 1 min
  {
    tiempo4 = currentMillis;
    tempA = temp;
    //Curva de temperatura-velocidad
    if(tempA < TEMP_CUTOFF)
    {
      isAuto = 0;                   //Apagamos el modo auto
      powerState = 0;               //Marcamos como apagado
      isSwing = 0;                  //Paramos el swing
      fanSpeed = 0;
      setPWMDuty(fanSpeed);         //Apagamos el ventilador
      servoPos = servo_CLOSED_POS;
      moveServo(servoPos);          //Cerramos las fins
    }
    else 
    {
      if(tempA > TEMP_MAX_SPEED) tempA = TEMP_MAX_SPEED;
      fanSpeed = map(tempA*10, TEMP_CUTOFF*10, TEMP_MAX_SPEED*10, 20, 100); //Multiplicamos para pasar los decimales a enteros, map no usa los decimales y simplemente trunca el numero, por eso.
      setPWMDuty(fanSpeed);
    }
  }

  //Temporizador
  if((timerMode == 1)&&((unsigned long)(currentMillis - tiempo5) >= 60000)) //Cada minuto restamos 1 minuto
  {
    tiempo5 = currentMillis;
    setTime = setTime - 60000; //Ya que setTime trabaja en ms
    if(setTime <= 0)
    {
      fanSpeed = 0;
      setPWMDuty(fanSpeed); //Apagado
      isSwing = 0;
      isAuto = 0;
      timerMode = 0;
      servoPos = servo_CLOSED_POS;
      moveServo(servoPos);
      powerState = 0;
    }
  }

  //Timer set timeout
  if((timeChange == 1)&&((unsigned long)(currentMillis - tiempo6) >= 5000)) //Tras 5 segundos no nos dejara cambiar el tiempo y podemos mover las fins
  {
    timeChange = 0;
  }

  //Control de la informacion en pantalla
  if((isScreen == 1)&&((unsigned long)(currentMillis - tiempo3 + forceUpdateAdd) >= screenRefreshInterval)) //1/15 Hz de refresco (no necesitamos mas)
  {
    tiempo3 = currentMillis;
    forceUpdateAdd = 0; //Already forced, set it to zero
    //Serial.println(hdc1080.readTemperature());
    oled.clearDisplay();
    //Humedad
    oled.setCursor(84,20); //Movemos el cursor
    oled.print(hum,1);
    oled.drawBitmap(64,4,humLogo,20,25,WHITE);
    //Temperatura
    oled.setCursor(18,20);
    oled.print(temp,1);
    oled.drawBitmap(-2,2,tempLogo,20,25,WHITE);
    //TEST
    //oled.setCursor(20,40);
    //oled.print(currentMillis);
    //Power logo
    if((isAuto == 0)&&(powerState == 1)&&(timerMode == 0))
    {
      oled.drawBitmap(25,42,powerLogo,78,32,WHITE);
      if(fanSpeed == 0)
      {
        oled.fillRect(25,32,120,32,BLACK);
      }
      else if(fanSpeed == 25)
      {
        oled.fillRect(48,32,52,32,BLACK);
      }
      else if(fanSpeed == 50)
      {
        oled.fillRect(63,32,52,32,BLACK);
      }
      else if(fanSpeed == 75)
      {
        oled.fillRect(83,32,20,32,BLACK);
      }
    }
    else if((isAuto == 1)&&(powerState == 1)) //Marcador modo auto (control por temperatura)
    {
      oled.setCursor(5,59);
      oled.print(F("A"));
      oled.drawRect(2,45,18,17,WHITE);
      oled.setCursor(35,59);
      oled.print(fanSpeed);
      oled.setCursor(64,59);
      oled.print(F("%"));
    }
    else if((timerMode == 1)&&(powerState == 1)) //Modo timer (temporizador)
    {
      //Llamamos a la funcion para convetir setTime en HH:MM
      convertToHours(setTime);
      convertToMin(setTime);
      oled.setCursor(34,59); //Mostramos el tiempo restante en pantalla
      oled.print(horas);
      oled.setCursor(45,59);
      oled.print(F("h:"));
      oled.setCursor(59,59);
      oled.print(minutos);
      oled.setCursor(80,59);
      oled.print(F("m"));
      //Marcador
      oled.setCursor(5,59);
      oled.print(F("T"));
      oled.drawRect(2,45,18,17,WHITE);
    }
    /*else if(powerState == 0) //Mostramos info de CO2 y TVOCs
    {
      //CO2
      oled.setCursor(18,50);
      oled.print(F("CO2: "));
      oled.print(CO2);
      //TVOC
      oled.setCursor(84,50);
      oled.print(F("TVOC: "));
      oled.print(TVOC);
    }*/

    //Swing logo
    if(isSwing == 1)
    {
      oled.setCursor(110,59);
      oled.print(F("S"));
      oled.drawRect(107,45,18,17,WHITE);
    }
    oled.display();
  }

  //Screen saver (Prevents burn-in with OLED screens)
  if((isScreen == 1)&&((unsigned long)(currentMillis - tiempo8) >= (screen_saver_timer*60L*1000L)))
  {
    tiempo8 = currentMillis;
    isScreen = 0; //Set the screen variable to OFF
    oled.clearDisplay();
    oled.display();
  }
} //loop