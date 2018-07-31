#include <Arduino.h>
#include <DueTimer.h>
#include "pwm_lib.h"
#include "tc_lib.h"

void funcion_t1();

#define PWM_PERIODO_US 10000 //10khz

// Diametro de la rueda en cm. cm = 2.54 * pulgadas
#define diametro_cm 25.4 // 10 pulgadas
#define pi 3.1415926535
// cantidad de periodos que se tienen por vuelta del motor
#define pulsos_por_vuelta 21
// perimetro en cm de la rueda
float perimetro_rueda = pi * diametro_cm;
// distancia recorrida por cada periodo de señal del motor
float distancia_periodo = perimetro_rueda / pulsos_por_vuelta;
#define us_to_s 1/1000000
#define cm_to_m 1/100
#define metros_en_us distancia_periodo * cm_to_m / us_to_s


// Definiciones motor D(Delantero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
#define CAPTURE_TIME_WINDOW 40000000 // usecs
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_motor_D;

int salida_pwm_motor_D = 0;
int entrada_analogica_motor_D = 0;

capture_tc8_declaration(); // TC0 and channel 1 pin A7
auto& capture_motor_D = capture_tc8;

uint32_t status_motor_D, duty_motor_D, period_motor_D, period_motor_D_us, pulses_motor_D;

// Sentido de giro del motor
#define z_f_pin_motor_D 9
// Pulsos de velocidad de salida
//#define signal_pin_motor_D 10
// Enable control
#define el_pin_motor_D 12

// Definiciones motor T(Tracero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> pwm_motor_T;

int salida_pwm_motor_T = 0;
int entrada_analogica_motor_T = 0;

capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto& capture_motor_T = capture_tc6;

uint32_t status_motor_T, duty_motor_T, period_motor_T, period_motor_T_us, pulses_motor_T;

// Sentido de giro del motor
#define z_f_pin_motor_T 7
// Pulsos de velocidad de salida
//#define signal_pin_motor_T 5
// Enable control
#define el_pin_motor_T 4

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
  capture_motor_D.config(CAPTURE_TIME_WINDOW);
  capture_motor_T.config(CAPTURE_TIME_WINDOW);
  // Put your setup code here, to run once:
  // Seteado a 10Khz para probar
  pwm_motor_D.start(PWM_PERIODO_US, 1000);
  pwm_motor_T.start(PWM_PERIODO_US, 1000);

  pinMode(z_f_pin_motor_D, OUTPUT);
  //pinMode(signal_pin_motor_D, INPUT);
  pinMode(el_pin_motor_D, OUTPUT);

  pinMode(z_f_pin_motor_T, OUTPUT);
  //pinMode(signal_pin_motor_T, INPUT);
  pinMode(el_pin_motor_T, OUTPUT);

  pinMode(pin_RST_RS485, OUTPUT);
  // desactiva el modo transmision en el conversor RS485
  digitalWrite(pin_RST_RS485, LOW);

  pwm_motor_D.set_duty(0);
  pwm_motor_T.set_duty(0);

  //Serial1.begin(1312500);
  Serial3.begin(256000);
  //Serial3.begin(115200);
  pinMode(30, OUTPUT);
  // Ventana de tiempo 1mS
  Timer0.start(10000);
  // Interrupcion timer 1
  Timer0.attachInterrupt(funcion_t1);
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

      salida_pwm_motor_D = map(actuador[1], 0, 100, 0, PWM_PERIODO_US);
      pwm_motor_D.set_duty(salida_pwm_motor_D);
      salida_pwm_motor_T = map(actuador[2], 0, 100, 0, PWM_PERIODO_US);
      pwm_motor_T.set_duty(salida_pwm_motor_T);
    
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
      status_motor_D = capture_motor_D.get_duty_period_and_pulses(duty_motor_D, period_motor_D, pulses_motor_D);
      period_motor_D_us = period_motor_D/(42);
      sensor[1] = metros_en_us / period_motor_D_us;
      capture_motor_D.config(CAPTURE_TIME_WINDOW);
      status_motor_T = capture_motor_T.get_duty_period_and_pulses(duty_motor_T, period_motor_T, pulses_motor_T);
      period_motor_T_us = period_motor_T/(42);
      sensor[2] = metros_en_us / period_motor_T_us;
      capture_motor_T.config(CAPTURE_TIME_WINDOW);
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
//****************** XXX  INTERRUPCION TIMER DE 1ms  XXX  *****************************
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