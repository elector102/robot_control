#include <Arduino.h>

//#include <avr/wdt.h>
#include <DueTimer.h>

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

void funcion_t1();

//#define PWM_PERIODO_US 100000 //1khz
#define PWM_PERIODO_US 10000 //10khz


// definiciones motor D(Delantero)
// defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// this object uses PWM channel 0
#define CAPTURE_TIME_WINDOW 40000000 // usecs
pwm<pwm_pin::PWML7_PC24> pwm_motor_D;

int salida_pwm_motor_D = 0;
int entrada_analogica_motor_D = 0;

capture_tc1_declaration(); // TC0 and channel 1 pin A7
auto& capture_motor_D = capture_tc1;

uint32_t status_motor_D, duty_motor_D, period_motor_D, pulses_motor_D;

// sentido de giro del motor
#define z_f_pin_motor_D 9
// pulsos de velocidad de salida
#define signal_pin_motor_D 10
// enable control
#define el_pin_motor_D 11

// definiciones motor T(Tracero)
// defining pwm object using pin 7, pin PC23 mapped to pin 7 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWML6_PC23> pwm_motor_T;

int salida_pwm_motor_T = 0;
int entrada_analogica_motor_T = 0;

capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto& capture_motor_T = capture_tc6;

uint32_t status_motor_T, duty_motor_T, period_motor_T, pulses_motor_T;

// sentido de giro del motor
#define z_f_pin_motor_T 9
// pulsos de velocidad de salida
#define signal_pin_motor_T 10
// enable control
#define el_pin_motor_T 11

byte cantidad_sensores;
byte cantidad_actuadores;

//BUFFERS
unsigned int envio[20];
byte inData[20]; //buffer temporal de lectura. No usar
byte inData1[20]; //buffer final de lectura.Datos crudos.
float actuador[20]; //buffer final de datos de actuadores. Datos en formato 15 bit. 
float sensor[20]; //buffer final de escritura
float Ksensor = 1.0; //mapeo
float Kactuador = 1.0; //mapeo
byte IDslave = 1;
int contador = 0;
int encabezado = 0, i = 0, j = 0, k = 0;

bool evento_tx = 0, evento_control = 0, evento_rx = 0, evento_tarea_1 = 0; //banderas de eventos-tareas

byte modo = 0;//modo-funcion
byte estado_debug = 0;





//*************************************************************************************
//********** SERVICIO DE INTERRUPCION  ISR(SERIAL_RX)   *********************
//*************************************************************************************
void serialEvent() {
  inData[i] = Serial.read();  //LEE UN BYTE
  //******************** || SEPARANDO LOS DADOS RECIBIDOS || *************************
  if (encabezado < 3)
  {
    if (inData[i] == 255)//detección de cabecera
    {
      encabezado = encabezado + 1;
    } else {//No detecta encabezado

      if((encabezado==2)&&(inData[i]==IDslave)){
        encabezado = encabezado + 1;
      }else{        
        i = 0;
        encabezado = 0;
      }
    }
  } else {//SI detecta cabecera encabezado=2,i=1 
    i = i + 1; 
    if (i <= 3) {
      if (i == 1) {
        modo = inData[0];
      }
      if (i == 2) {
        cantidad_actuadores = inData[1];
      }
      if (i == 3) {
        cantidad_sensores = inData[2];
      }
    } else {
      if (i >= (4 + 2 * cantidad_actuadores))
      {
        if (inData[3 + 2 * cantidad_actuadores] == 0)
        {
          for (k = 0; k < (2 * cantidad_actuadores + 3); k++)
          {
            inData1[k] = inData[k];
          }
          evento_rx = 1; //indico nuevo dato  
     
        }
        i = 0; //reinicio de deteccion de encabezado de trama
        encabezado = 0;
      }
    }
  }
}//--END serialEvent()-------------- -------------------------


//*************************************************************************************
//********* XXX  INITIALIZATION AND SETUP DE LOS PINOS Y CONTROLADOR XXX  *************
//*************************************************************************************
void setup() {
  // initialization of capture objects
  capture_motor_D.config(CAPTURE_TIME_WINDOW);
  capture_motor_T.config(CAPTURE_TIME_WINDOW);
  // put your setup code here, to run once:
  //seteado a 10Khz para probar
  pwm_motor_D.start(PWM_PERIODO_US,1000);
  pwm_motor_T.start(PWM_PERIODO_US,1000);

  pinMode(z_f_pin_motor_D, OUTPUT);
  pinMode(signal_pin_motor_D, INPUT);
  pinMode(el_pin_motor_D, OUTPUT);

  pinMode(z_f_pin_motor_T, OUTPUT);
  pinMode(signal_pin_motor_T, INPUT);
  pinMode(el_pin_motor_T, OUTPUT);

  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
  pwm_motor_D.set_duty(0);
  pwm_motor_T.set_duty(0);

  
  Serial.begin(256000);
  pinMode(7, OUTPUT);
  Timer0.start(1000);            //ventana de tiempo 1mS
  Timer0.attachInterrupt(funcion_t1);   //interrupcion timer 1
  

}//----------------   FIM DO SETUP Y PARAMETROS -----------------------------

//*************************************************************************************
//***************************** XXX  MAIN  XXX  ***************************************
//*************************************************************************************
void loop() {

  if (evento_control == 1)       //CADA 1ms EJECUTA ESTE CODE - LAZO DE CONTROL
  {
    
    if (modo == 1) // MODO = 1 START
    {
      sensor[0] = actuador[0]; //ejemplo: espejo en dato[0]

      salida_pwm_motor_D = map(actuador[1], 0, 100, 0, PWM_PERIODO_US);
      pwm_motor_D.set_duty(salida_pwm_motor_D);
      salida_pwm_motor_T = map(actuador[2], 0, 100, 0, PWM_PERIODO_US);
      pwm_motor_T.set_duty(salida_pwm_motor_T);
    
    }
    if (modo == 0) { //STOP
      //Incluir Aqui codigo de parada
     
  
      if(sensor[0]>30000.0) {
        sensor[0]=0.0;
       }
   }
   evento_control=0;
    
 }
  if (evento_tarea_1 == 1) {      //TAREA CADA xxx ms
      status_motor_D = capture_motor_D.get_duty_period_and_pulses(duty_motor_D,period_motor_D,pulses_motor_D);
      sensor[1] = pulses_motor_D;
      capture_motor_D.config(CAPTURE_TIME_WINDOW);
      status_motor_T = capture_motor_T.get_duty_period_and_pulses(duty_motor_T, period_motor_T, pulses_motor_T);
      sensor[2] = pulses_motor_T;
      capture_motor_T.config(CAPTURE_TIME_WINDOW);
      evento_tarea_1 = 0;
  }
  //*************************************************************************************
  //*************** EVENTO DETECCION DE TRAMA   *************************************
  //*************************************************************************************
  if (evento_rx == 1) {
    modo = inData1[0];                // RECIBE MODO
    cantidad_actuadores = inData1[1]; // RECIBE CANTIDAD DE AcTUADORES
    cantidad_sensores = inData1[2];   // RECIBE CANTIDAD DE SENSORES
    for (k = 0; k < cantidad_actuadores; k++) {
      actuador[k] = ((float)256.00 * (0x7F & inData1[2 * k + 4]) + inData1[2 * k + 3]); //arma dato de 15 bits de magnitud
      if ((inData1[2 * k + 4] & 0x80) > 0) { // signo en bit 16
        actuador[k] = Kactuador * actuador[k] * (-1.0);
      }
    }
    evento_rx = 0; // RESET DE BANDERA RX TRAMA
    evento_tx = 1; // ACTIVA BANDERA DE TX
  }

  //*************************************************************************************
  //*************** XXX  TRANSMIION DE DATOS  XXX  *************************************
  //*************************************************************************************
  if (evento_tx == 1)
  {
    if(estado_debug==1){
      digitalWrite(7, HIGH);
      estado_debug=0;
    }else{
      digitalWrite(7, LOW);
      estado_debug=1;
    }
    sensor[0] = sensor[0]+1.0;//DEBUG
    Serial.write(0xFF);    // envia cabecera FF FF
    Serial.write(0xFF);    // envia cabecera FF FF
    Serial.write(IDslave);   //envia modo
    Serial.write(modo);   //envia modo
  
    for (k = 0; k < cantidad_sensores; k++) {
      
      if (sensor[k] < 0) {
        envio[k] =  ((unsigned int)(-sensor[k] * Ksensor)) & 0x7FFF;
        envio[k] = envio[k] | 0x8000;
      }else{
         envio[k] =  ((unsigned int)(sensor[k] * Ksensor)) & 0x7FFF;
      }
      Serial.write(highByte(envio[k]));
      Serial.write(lowByte(envio[k]));
      
    }
    Serial.write((byte)0);       //fin trama
    evento_tx = 0;
  }
}//----------------   FIM MAIN ---------------------------------


//*************************************************************************************
//****************** XXX  INTERRUPCION TIMER DE 1ms  XXX  *****************************
//*************************************************************************************
void funcion_t1()
{
  evento_control = 1;
  digitalWrite(42, HIGH);

  contador++;
  if (contador >= 100) {

    evento_tarea_1 = 1; //tarea cada 100 ms
    contador = 0;


  }
}


