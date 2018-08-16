#include <Arduino.h>
#include <DueTimer.h>
#include "pwm_lib.h"
#include "tc_lib.h"
#include <PID_v1.h>

void funcion_t1();

#define PWM_PERIODO_US 10000 //10khz
#define DEBUG_MODE true
// Diametro de la rueda en cm. cm = 2.54 * pulgadas
#define DIAMETRO_CM 24.13 // 9.5 pulgadas
#define PI 3.1415926535
// cantidad de periodos que se tienen por vuelta del motor
#define PULSOS_POR_VUELTAS 42
// perimetro en cm de la rueda
double perimetro_rueda = PI * DIAMETRO_CM;
// distancia recorrida por cada periodo de señal del motor
double distancia_periodo = perimetro_rueda / PULSOS_POR_VUELTAS;
double us_to_s = 1000000;
double cm_to_m = 1;
double metros_en_us = distancia_periodo * cm_to_m * us_to_s;

#define MAX_VEL_CM_S 1000

// Sentido de giro del motor
#define z_f_pin_motor_front 9
// Pulsos de velocidad de salida
//#define signal_pin_motor_front 10
// Enable control
#define el_pin_motor_front 12


// Definiciones motor front(Delantero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
#define CAPTURE_TIME_WINDOW 40000000 // usecs
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_motor_front;

int salida_pwm_motor_front = 0;
int entrada_analogica_motor_front = 0;

capture_tc8_declaration(); // TC0 and channel 1 pin A7
auto& capture_motor_front = capture_tc8;

uint32_t status_motor_front, duty_motor_front, period_motor_front, period_motor_front_us, pulses_motor_front;

double set_point_PID_front = 0;
double output_PID_front = 0;
double input_PID_front = 0;
double PID_Kp_front = 1;
double PID_Ki_front = 0;
double PID_Kd_front = 0;

PID PID_motor_front(&input_PID_front, &output_PID_front, &set_point_PID_front, PID_Kp_front, PID_Ki_front, PID_Kd_front, DIRECT);

// Sentido de giro del motor
#define z_f_pin_motor_back 7
// Pulsos de velocidad de salida
//#define signal_pin_motor_back 5
// Enable control
#define el_pin_motor_back 4

// Definiciones motor back(Tracero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> pwm_motor_back;

int salida_pwm_motor_back = 0;
int entrada_analogica_motor_back = 0;

capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto& capture_motor_back = capture_tc6;

uint32_t status_motor_back, duty_motor_back, period_motor_back, period_motor_back_us, pulses_motor_back;

double set_point_PID_back = 0;
double output_PID_back = 0;
double input_PID_back = 0;
double PID_Kp_back = 1;
double PID_Ki_back = 0;
double PID_Kd_back = 0;

PID PID_motor_back(&input_PID_back, &output_PID_back, &set_point_PID_back, PID_Kp_back, PID_Ki_back, PID_Kd_back, DIRECT);

byte cantidad_sensores;
byte cantidad_actuadores;

// BUFFERS
unsigned int envio[20];
// Buffer temporal de lectura. No usar
byte inData[20];
// Buffer final de lectura.Datos crudos.
byte inData1[20];
// Buffer final de datos de actuadores. Datos en formato 15 bit.
float actuador[20]; 
// Buffer final de escritura
float sensor[20];
// Mapeo
float Ksensor = 1.0;
// Mapeo
float Kactuador = 1.0; 
byte IDslave = 1;
int contador = 0;
int encabezado = 0, i = 0, j = 0, k = 0;

// Banderas de eventos-tareas
bool evento_tx = 0, evento_control = 0, evento_rx = 0, evento_tarea_1 = 0;

int pin_RST_RS485 = 10;
// Modo-funcion
byte modo = 0;
byte estado_debug = 0;



//*************************************************************************************
//********** SERVICIO DE INTERRUPCION  ISR(SERIAL_RX)   *********************
//*************************************************************************************
void serialEvent3() {
  // Lee un byte
  inData[i] = Serial3.read();
  // Separacion de datos recibidos
  if (encabezado < 3) {
    // Detección de cabecera
    if (inData[i] == 255) {
      encabezado = encabezado + 1;
    } else {
      // No detecta encabezado
      if((encabezado == 2) && (inData[i] == IDslave )) {
        encabezado = encabezado + 1;
      } else {        
        i = 0;
        encabezado = 0;
      }
    }
  } else {//SI detecta cabecera encabezado=2,i=1 
    i = i + 1; 
    if (i <= 3) {
      if (i == 1)
        modo = inData[0];
      if (i == 2)
        cantidad_actuadores = inData[1];
      if (i == 3)
        cantidad_sensores = inData[2];
    } else {
      if (i >= (4 + 2 * cantidad_actuadores)) {
        if (inData[3 + 2 * cantidad_actuadores] == 0) {
          for (k = 0; k < (2 * cantidad_actuadores + 3); k++)
            inData1[k] = inData[k];
          // Indico nuevo dato
          evento_rx = 1;   
        }
        // Reinicio de deteccion de encabezado de trama
        i = 0;
        encabezado = 0;
      }
    }
  }
}//--END serialEvent()-------------- -------------------------


//*************************************************************************************
//********* XXX  INITIALIZATION AND SETUP DE LOS PINOS Y CONTROLADOR XXX  *************
//*************************************************************************************
void setup() {
  // Initialization of capture objects
  capture_motor_front.config(CAPTURE_TIME_WINDOW);
  capture_motor_back.config(CAPTURE_TIME_WINDOW);
  // Put your setup code here, to run once:
  // Seteado a 10Khz para probar
  pwm_motor_front.start(PWM_PERIODO_US, 1000);
  pwm_motor_back.start(PWM_PERIODO_US, 1000);

  pinMode(z_f_pin_motor_front, OUTPUT);
  //pinMode(signal_pin_motor_front, INPUT);
  pinMode(el_pin_motor_front, OUTPUT);

  pinMode(z_f_pin_motor_back, OUTPUT);
  //pinMode(signal_pin_motor_back, INPUT);
  pinMode(el_pin_motor_back, OUTPUT);

  pinMode(pin_RST_RS485, OUTPUT);
  // desactiva el modo transmision en el conversor RS485
  digitalWrite(pin_RST_RS485, LOW);

  pwm_motor_front.set_duty(0);
  pwm_motor_back.set_duty(0);
  #if (DEBUG_MODE) 
    Serial.begin(115200);
  #endif
  //Serial1.begin(1312500);
  Serial3.begin(256000);
  //Serial3.begin(115200);
  pinMode(30, OUTPUT);
  // Ventana de tiempo 10 mS
  Timer0.start(10000);
  // Interrupcion timer 1
  Timer0.attachInterrupt(funcion_t1);
  //turn the PID on
  PID_motor_front.SetMode(AUTOMATIC);
  //turn the PID on
  PID_motor_back.SetMode(AUTOMATIC);
  PID_motor_front.SetSampleTime(10);// in ms
  PID_motor_back.SetSampleTime(10);// in ms
}//----------------   FIM DO SETUP Y PARAMETROS -----------------------------

//*************************************************************************************
//***************************** XXX  MAIN  XXX  ***************************************
//*************************************************************************************
void loop() {
  // Cada 1ms ejecuta este codigo - Lazo de control
  if (evento_control == 1) {
    
    if (modo == 1) {
      // Ejemplo: espejo en dato[0]
      sensor[0] = actuador[0]; 

      set_point_PID_front = actuador[1]; // cm/s


      if ((PID_Kp_front != actuador[2]) || (PID_Kp_front != actuador[3]) || (PID_Kp_front != actuador[4]) ){
        PID_Kp_front = actuador[2];
        PID_Ki_front = actuador[3];
        PID_Kd_front = actuador[4];
        PID_motor_front.SetTunings(PID_Kp_front, PID_Ki_front, PID_Kd_front);
      }

      set_point_PID_back = actuador[5]; // cm/s


      if ((PID_Kp_back != actuador[6]) || (PID_Kp_back != actuador[7]) || (PID_Kp_back != actuador[8]) ){
        PID_Kp_back = actuador[6];
        PID_Ki_back = actuador[7];
        PID_Kd_back = actuador[8];
        PID_motor_back.SetTunings(PID_Kp_back, PID_Ki_back, PID_Kd_back);
      }

      status_motor_front = capture_motor_front.get_duty_period_and_pulses(duty_motor_front, period_motor_front, pulses_motor_front);
      period_motor_front_us = period_motor_front/(42);
      input_PID_front = metros_en_us / period_motor_front_us;
      sensor[1] = input_PID_front;

      status_motor_back = capture_motor_back.get_duty_period_and_pulses(duty_motor_back, period_motor_back, pulses_motor_back);
      period_motor_back_us = period_motor_back/(42);
      input_PID_back = metros_en_us / period_motor_back_us;
      sensor[2] = input_PID_back;
      
      // PID calculation and command
      PID_motor_front.Compute();
      salida_pwm_motor_front = map(output_PID_front, 0, MAX_VEL_CM_S, 0, PWM_PERIODO_US);
      pwm_motor_front.set_duty(salida_pwm_motor_front);
      
      PID_motor_back.Compute();
      salida_pwm_motor_back = map(output_PID_back, 0, MAX_VEL_CM_S, 0, PWM_PERIODO_US);
      pwm_motor_back.set_duty(salida_pwm_motor_back);


      #if (DEBUG_MODE) 
        Serial.println("El valor de los encoder en cm por s es :");
        Serial.print("el motor A : ");
        Serial.println(sensor[1]);
        Serial.print("el motor B : ");
        Serial.println(sensor[2]);
        Serial.print("Metros en ms motor : ");
        Serial.println(metros_en_us);
        Serial.print("PID output motor front in cm/s : ");
        Serial.println(output_PID_front);
        Serial.print("PID output motor back in cm/s : ");
        Serial.println(output_PID_back);
      #endif
      capture_motor_front.config(CAPTURE_TIME_WINDOW);
      capture_motor_back.config(CAPTURE_TIME_WINDOW);
      
    }
    // MODO = 0 --> STOP
    if (modo == 0) {
      // Incluir Aqui codigo de parada
      if(sensor[0] > 30000.0) {
        sensor[0] = 0.0;
       }
    }
    evento_control = 0;
  }
  // Cada xxx ms ejecuta este codigo
  if (evento_tarea_1 == 1) {      
    evento_tarea_1  = 0;
  }
  //*************************************************************************************
  //*************** EVENTO DETECCION DE TRAMA   *************************************
  //*************************************************************************************
  if (evento_rx == 1) {
    // Recibe modo
    modo = inData1[0];  
    // Recibe cantidad de actuadores
    cantidad_actuadores = inData1[1];
    // Recibe cantidad de sensores
    cantidad_sensores = inData1[2];
    for (k = 0; k < cantidad_actuadores; k++) {
      // Arma dato de 15 bits de magnitud
      actuador[k] = ((float)256.00 * (0x7F & inData1[2 * k + 4]) + inData1[2 * k + 3]); 
      // Signo en bit 16
      if ((inData1[2 * k + 4] & 0x80) > 0) {
        actuador[k] = Kactuador * actuador[k] * (-1.0);
      }
    }
    // Reset de bandera RX trama
    evento_rx = 0;
    // Activa bandera de TX
    evento_tx = 1;
  }

  //*************************************************************************************
  //*************** XXX  TRANSMIION DE DATOS  XXX  *************************************
  //*************************************************************************************
  if (evento_tx == 1) {
    // Activa modo transmisor en el conversor RS485
    digitalWrite(pin_RST_RS485, HIGH);
    if(estado_debug == 1) {
      digitalWrite(30, HIGH);
      estado_debug = 0;
    } else {
      digitalWrite(30, LOW);
      estado_debug = 1;
    }
    // DEBUG
    //sensor[0] = sensor[0] + 1.0;
    // Envia cabecera FF FF
    Serial3.write(0xFF);    
    // Envia cabecera FF FF
    Serial3.write(0xFF);    
    // Envia modo
    Serial3.write(IDslave);
    // Envia modo   
    Serial3.write(modo);   
  
    for (k = 0; k < cantidad_sensores; k++) {
      if (sensor[k] < 0) {
        envio[k] =  ((unsigned int)(-sensor[k] * Ksensor)) & 0x7FFF;
        envio[k] = envio[k] | 0x8000;
      } else {
        envio[k] =  ((unsigned int)(sensor[k] * Ksensor)) & 0x7FFF;
      }
      Serial3.write(highByte(envio[k]));
      Serial3.write(lowByte(envio[k]));
    }
    // Fin trama
    Serial3.write((byte)0);       
    evento_tx = 0;
    // desactiva el modo transmision en el conversor RS485
    //delayMicroseconds(350);
    while (Serial3.availableForWrite() != 127 ){
        //Serial.print(" los datos disponibles son : ");
        //Serial.println(Serial3.availableForWrite());
    }
    delayMicroseconds(170);
    digitalWrite(pin_RST_RS485, LOW);
  }
}//----------------   FIM MAIN ---------------------------------

//*************************************************************************************
//****************** XXX  INTERRUPCION TIMER DE 10 ms  XXX  *****************************
//*************************************************************************************
void funcion_t1() {
  evento_control = 1;
  contador++;
  if (contador >= 100) {
    // Tarea cada 100 ms
    evento_tarea_1 = 1; 
    contador = 0;
  }
}
