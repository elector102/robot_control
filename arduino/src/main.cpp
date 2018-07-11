#include <Arduino.h>

//#include <avr/wdt.h>
#include <DueTimer.h>

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

void funcion_t1();
//señal de control de velocidad
#define vr_pin 8
// sentido de giro del motor
#define z_f_pin 9
// pulsos de velocidad de salida
#define signal_pin 10
// enable control
#define el_pin 11
//#define PWM_PERIODO_US 100000 //1khz
#define PWM_PERIODO_US 10000 //10khz
// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWML7_PC24> pwm_pin6;

int salida_pwm =0;
int entrada_analogica =0;

#define CAPTURE_TIME_WINDOW 40000000 // usecs

capture_tc1_declaration(); // TC0 and channel 1
auto& capture_pinA7=capture_tc1;

uint32_t status,duty,period,pulses;





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
byte IDslave=1;
int contador = 0;
int encabezado = 0, i = 0, j = 0, k = 0;

bool evento_tx = 0, evento_control = 0, evento_rx = 0, evento_tarea_1 = 0; //banderas de eventos-tareas

byte modo = 0;//modo-funcion
byte estado_debug=0;





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
  capture_pinA7.config(CAPTURE_TIME_WINDOW);
  // put your setup code here, to run once:
  //seteado a 1Khz para probar
  pwm_pin6.start(PWM_PERIODO_US,1000);
  pinMode(vr_pin, OUTPUT);
  pinMode(z_f_pin, OUTPUT);
  pinMode(signal_pin, INPUT);
  pinMode(el_pin, OUTPUT);

  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
  pwm_pin6.set_duty(0);

  
  Serial.begin(256000);
  pinMode(7, OUTPUT);
  Timer0.start(1000);            //ventana de tiempo 1mS
  Timer0.attachInterrupt(funcion_t1);   //interrupcion timer 1
  

}//----------------   FIM DO SETUP Y PARAMETROS -----------------------------

//*************************************************************************************
//***************************** XXX  MAIN  XXX  ***************************************
//*************************************************************************************
void loop() {
  digitalWrite(42, LOW);
  //digitalWrite(42, HIGH);
  if (evento_control == 1)       //CADA 1ms EJECUTA ESTE CODE - LAZO DE CONTROL
  {
    
    if (modo == 1) // MODO = 1 START
    {
      //digitalWrite(42, HIGH);
      sensor[0] = actuador[0]; //ejemplo: espejo en dato[0]

      salida_pwm = map(actuador[1], 0, 100, 0, PWM_PERIODO_US);
      pwm_pin6.set_duty(salida_pwm);
    
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
      status = capture_pinA7.get_duty_period_and_pulses(duty,period,pulses);
      sensor[1] = pulses;
      capture_pinA7.config(CAPTURE_TIME_WINDOW);
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


