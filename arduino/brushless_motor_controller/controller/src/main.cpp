#include <Arduino.h>
#include <DueTimer.h>
#include "pwm_lib.h"
#include "tc_lib.h"
#include <PID_v1.h>

void funcion_t1();
#define LEFT 1
#define RIGHT 0
#define BOARD_SIDE LEFT

#define FRONT 0
#define BACK 1

#if (BOARD_SIDE)
  #define forward_direction HIGH
  #define backward_direction LOW
  byte IDslave = 1;
#else
  #define forward_direction LOW
  #define backward_direction HIGH
  byte IDslave = 2;
#endif

#define STOP_STATE (float)0.0
#define FORWARD_ROTATION_STATE (float)1.0
#define BACKWARD_ROTATION_STATE (float)-1.0
#define ROTATION_STATE (float)2.0

int current_motor_direction[2] = { STOP_STATE, STOP_STATE};
int output_motor_direction[2] = { STOP_STATE, STOP_STATE};

int last_motor_input_update_rate[2] = {0, 0};
int last_motor_input[2] = {0, 0};

#define CORRECT_ROTATION 0
#define BRAKE_ROTATION 1
#define CONFIRM_ROTATION_CHANGE 2
int state_of_rotation[2] = { STOP_STATE, STOP_STATE};

#define EVENT_CONTROL_DEBUG_PIN 33

#define PWM_PERIODO_US 10000 //10khz
#define PWM_PERIODO_US_MIN 0
#define DEBUG_MODE false
#define USE_PID_MODE true
// Diametro de la rueda en cm. cm = 2.54 * pulgadas
#define DIAMETRO_CM 24.13 // 9.5 pulgadas
#define PI 3.1415926535
// cantidad de periodos que se tienen por vuelta del motor
#define PULSOS_POR_VUELTAS 45.0
// perimetro en cm de la rueda
float perimetro_rueda = PI * DIAMETRO_CM;
// distancia recorrida por cada periodo de señal del motor
float distancia_periodo = perimetro_rueda / PULSOS_POR_VUELTAS; // 1.8cm
float us_to_s = 1000000;
float cm_to_m = 1;
float cm_por_periodo = distancia_periodo;

#define MAX_VEL_CM_S 800.0

// Sentido de giro del motor
int z_f_pin_motor[2] ={9, 4};
// Pulsos de velocidad de salida
//#define signal_pin_motor_front 10
// Enable control
int el_pin_motor[2] = {12, 7};

double input_PID_inter[2];
// Definiciones motor front(Delantero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
#define CAPTURE_TIME_WINDOW 400000 // usecs
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_motor_front;

double salida_pwm_motor_cm[2] = {0, 0};
double salida_pwm_motor[2] = {0, 0};
int entrada_analogica_motor[2] = {0, 0};

capture_tc8_declaration(); // TC0 and channel 1 pin A7
capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto& capture_motor_front = capture_tc8;
auto& capture_motor_back = capture_tc6;

uint32_t status_motor[2], duty_motor[2], period_motor[2], pulses_motor[2];
double period_motor_us[2];

double signal_set_point[2] = {0, 0};
double set_point_PID_cm[2] = {0, 0};
double set_point_PID[2] = {0, 0};
double output_PID[2] = {0, 0};
double output_PID_cm[2] = {0, 0};
double input_PID_cm[2] = {0, 0};
double input_PID[2] = {0, 0};
double PID_Kp[2] = {1, 1};
double PID_Ki[2] = {0, 0};
double PID_Kd[2] = {0, 0};

#if (USE_PID_MODE)
  PID PID_motor[2] = {PID(&input_PID[FRONT], &output_PID[FRONT], &set_point_PID[FRONT], PID_Kp[FRONT], PID_Ki[FRONT], PID_Kd[FRONT], DIRECT), PID(&input_PID[BACK], &output_PID[BACK], &set_point_PID[BACK], PID_Kp[BACK], PID_Ki[BACK], PID_Kd[BACK], DIRECT)};
  
#endif
// Sentido de giro del motor
// Pulsos de velocidad de salida
//#define signal_pin_motor_back 5
// Enable control

// Definiciones motor back(Tracero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> pwm_motor_back;


byte cantidad_sensores;
byte cantidad_actuadores;

// BUFFERS
unsigned int envio[50];
// Buffer temporal de lectura. No usar
byte inData[50];
// Buffer final de lectura.Datos crudos.
byte inData1[50];
// Buffer final de datos de actuadores. Datos en formato 15 bit.
float actuador[50]; 
// Buffer final de escritura
float sensor[50];
// Mapeo
float Ksensor = 1000.0;
// Mapeo
float Kactuador = 0.001; 

int contador = 0;
int encabezado = 0, i = 0, j = 0, k = 0;

// Banderas de eventos-tareas
bool evento_tx = 0, evento_control = 0, evento_rx = 0, evento_tarea_1 = 0;

int pin_RST_RS485 = 10;
// Modo-funcion
byte modo = 0;
byte estado_debug = 0;

int comunication_control_count = 0;



double doubleMap(double x, double in_min, double in_max, double out_min, double out_max) {
  double temp = (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
  return temp;
}
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



bool estimate_rotation_direction(uint motor){
  uint motor_state = 0;
  if (fabs(input_PID[motor]) < 0.001) {
    motor_state = STOP_STATE;
  } else {
    motor_state = ROTATION_STATE;
  }  
  switch(state_of_rotation[motor]) {
    case CORRECT_ROTATION:
      if (output_motor_direction[motor] != current_motor_direction[motor]) {
        state_of_rotation[motor] = BRAKE_ROTATION;
      }
      break;
    case BRAKE_ROTATION:
      if ( motor_state == STOP_STATE) {
        current_motor_direction[motor] = STOP_STATE;
        state_of_rotation[motor] = CONFIRM_ROTATION_CHANGE;
      }
      if (last_motor_input[motor] < (fabs(input_PID[motor]) - 0.025 * MAX_VEL_CM_S) ) {
        state_of_rotation[motor] = CONFIRM_ROTATION_CHANGE;
      } 
      break;
    case CONFIRM_ROTATION_CHANGE:
      if (motor_state == ROTATION_STATE) {
        current_motor_direction[motor] = output_motor_direction[motor];
        state_of_rotation[motor] = CORRECT_ROTATION;  
      }
      break;
    default:
      state_of_rotation[motor] = CORRECT_ROTATION;
      break;
  }
  if (last_motor_input_update_rate[motor] >= 10) {
    last_motor_input[motor] = fabs(input_PID[motor]);
    last_motor_input_update_rate[motor] = 0;
  } else {
      last_motor_input_update_rate[motor]++;
  }
}

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

  pinMode(z_f_pin_motor[FRONT], OUTPUT);
  //pinMode(signal_pin_motor_front, INPUT);
  pinMode(el_pin_motor[FRONT], OUTPUT);

  pinMode(z_f_pin_motor[BACK], OUTPUT);
  //pinMode(signal_pin_motor_back, INPUT);
  pinMode(el_pin_motor[BACK], OUTPUT);
  digitalWrite(z_f_pin_motor[FRONT], LOW);
  digitalWrite(z_f_pin_motor[BACK], LOW);

  pinMode(pin_RST_RS485, OUTPUT);
  // desactiva el modo transmision en el conversor RS485
  digitalWrite(pin_RST_RS485, LOW);

  pinMode(EVENT_CONTROL_DEBUG_PIN, OUTPUT);
  digitalWrite(EVENT_CONTROL_DEBUG_PIN, LOW);

  pwm_motor_front.set_duty(PWM_PERIODO_US_MIN);
  pwm_motor_back.set_duty(PWM_PERIODO_US_MIN);
  #if (DEBUG_MODE) 
    SerialUSB.begin(115200);
  #endif
  //Serial1.begin(1312500);
  Serial3.begin(256000);
  //Serial3.begin(115200);
  pinMode(30, OUTPUT);
  // Ventana de tiempo 10 mS
  Timer0.start(10000);
  // Interrupcion timer 1
  Timer0.attachInterrupt(funcion_t1);
  #if (USE_PID_MODE)
    //turn the PID on
    PID_motor[FRONT].SetMode(AUTOMATIC);
    //turn the PID on
    PID_motor[BACK].SetMode(AUTOMATIC);
    PID_motor[FRONT].SetSampleTime(10);// in ms
    PID_motor[BACK].SetSampleTime(10);// in ms
    PID_motor[FRONT].SetOutputLimits(-1.0, 1.0);
    PID_motor[BACK].SetOutputLimits(-1.0, 1.0);
  #endif
  digitalWrite(el_pin_motor[FRONT], HIGH);
  digitalWrite(el_pin_motor[BACK], HIGH);
  delay(100);
}//----------------   FIM DO SETUP Y PARAMETROS -----------------------------


//*************************************************************************************
//***************************** XXX  MAIN  XXX  ***************************************
//*************************************************************************************
void loop() {
  // Cada 1ms ejecuta este codigo - Lazo de control
  if (evento_control == 1) {
    if (comunication_control_count >= 10) {
      modo = 0;
    }
    if (modo == 1) {
      digitalWrite(el_pin_motor[FRONT], HIGH);
      digitalWrite(el_pin_motor[BACK], HIGH);
      // Ejemplo: espejo en dato[0]
      digitalWrite(EVENT_CONTROL_DEBUG_PIN, HIGH);
      sensor[0] = actuador[0]; 
      signal_set_point[FRONT] = (double)actuador[1] / (double)Kactuador;
      
      if (set_point_PID_cm[FRONT] != signal_set_point[FRONT]) {
        set_point_PID_cm[FRONT] = signal_set_point[FRONT]; // cm/s
        //
        set_point_PID_cm[FRONT] = set_point_PID_cm[FRONT] > MAX_VEL_CM_S ? MAX_VEL_CM_S : set_point_PID_cm[FRONT];
        set_point_PID_cm[FRONT] = set_point_PID_cm[FRONT] < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : set_point_PID_cm[FRONT];
        
        set_point_PID[FRONT] = doubleMap(set_point_PID_cm[FRONT], -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);
      }
      #if (DEBUG_MODE) 
        SerialUSB.print("el valor del actuador frontal con signo es : ");
        SerialUSB.println(actuador[1],8);
      #endif


      if ((PID_Kp[FRONT] != actuador[2]) || (PID_Kp[FRONT] != actuador[3]) || (PID_Kp[FRONT] != actuador[4]) ){
        PID_Kp[FRONT] = actuador[2];
        PID_Ki[FRONT] = actuador[3];
        PID_Kd[FRONT] = actuador[4];
        #if (USE_PID_MODE)
          PID_motor[FRONT].SetTunings(PID_Kp[FRONT], PID_Ki[FRONT], PID_Kd[FRONT]);
        #endif
      }
      signal_set_point[BACK] = actuador[5] / Kactuador;
      if (set_point_PID_cm[BACK] != signal_set_point[BACK]) {
        set_point_PID_cm[BACK] = signal_set_point[BACK]; // cm/s
        //
        set_point_PID_cm[BACK] = set_point_PID_cm[BACK] > MAX_VEL_CM_S ? MAX_VEL_CM_S : set_point_PID_cm[BACK];
        set_point_PID_cm[BACK] = set_point_PID_cm[BACK] < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : set_point_PID_cm[BACK];

        set_point_PID[BACK] = doubleMap(set_point_PID_cm[BACK], -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);
      }
      #if (DEBUG_MODE) 
        SerialUSB.print("el valor del actuador trasero con signo es : ");
        SerialUSB.println(actuador[5],8);
      #endif


      if ((PID_Kp[BACK] != actuador[6]) || (PID_Kp[BACK] != actuador[7]) || (PID_Kp[BACK] != actuador[8]) ){
        PID_Kp[BACK] = actuador[6];
        PID_Ki[BACK] = actuador[7];
        PID_Kd[BACK] = actuador[8];
        #if (USE_PID_MODE)
          PID_motor[BACK].SetTunings(PID_Kp[BACK], PID_Ki[BACK], PID_Kd[BACK]);
        #endif
      }
      estimate_rotation_direction(FRONT);
      status_motor[FRONT] = capture_motor_front.get_duty_period_and_pulses(duty_motor[FRONT], period_motor[FRONT], pulses_motor[FRONT]);
      period_motor_us[FRONT] = (double)period_motor[FRONT] / (double)capture_motor_front.ticks_per_usec();
      //capture_motor_front.config(CAPTURE_TIME_WINDOW);
      input_PID_inter[FRONT] = ((double)distancia_periodo * (double)us_to_s)/ period_motor_us[FRONT];
      //input_PID_front = isinf(input_PID_front_inter) ? 0 : input_PID_front_inter;
      if isinf(input_PID_inter[FRONT]) {
        input_PID_cm[FRONT] = 0.0;
        #if (DEBUG_MODE)
          SerialUSB.print("Input PID front es inf, el valor es : ");
          SerialUSB.println(input_PID_inter[FRONT], 8);
        #endif
      } else {
        if (current_motor_direction[FRONT] == STOP_STATE) {
          input_PID_cm[FRONT] = (double)input_PID_inter[FRONT] * output_motor_direction[FRONT];
        } else {
          input_PID_cm[FRONT] = (double)input_PID_inter[FRONT] * (double)current_motor_direction[FRONT];
        }
        #if (DEBUG_MODE) 
          SerialUSB.print("Input PID front, el valor es : ");
          SerialUSB.println(input_PID_inter[FRONT], 8);
          SerialUSB.print("Input PID front, el valor en cm es : ");
          SerialUSB.println(input_PID_cm[FRONT], 8);
        #endif
      }
      input_PID_cm[FRONT] = input_PID_cm[FRONT] > MAX_VEL_CM_S ? MAX_VEL_CM_S : input_PID_cm[FRONT];
      input_PID_cm[FRONT] = input_PID_cm[FRONT] < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : input_PID_cm[FRONT];

      input_PID[FRONT] = doubleMap(input_PID_cm[FRONT], -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);
      sensor[1] = input_PID[FRONT];


      estimate_rotation_direction(BACK);
      status_motor[BACK] = capture_motor_back.get_duty_period_and_pulses(duty_motor[BACK], period_motor[BACK], pulses_motor[BACK]);
      period_motor_us[BACK] = (double)period_motor[BACK] / (double)capture_motor_back.ticks_per_usec();
      //capture_motor_back.config(CAPTURE_TIME_WINDOW);
      input_PID_inter[BACK] = ((double)distancia_periodo * (double)us_to_s)/ period_motor_us[BACK];
      //input_PID_back = isinf(input_PID_back_inter) ? 0 : input_PID_back_inter;
      if isinf(input_PID_inter[BACK]) {
        input_PID_cm[BACK] = 0.0;
        #if (DEBUG_MODE)
          SerialUSB.print("Input PID back es inf, el valor es : ");
          SerialUSB.println(input_PID_inter[BACK], 8);
        #endif
      } else {
        if (current_motor_direction[BACK] == STOP_STATE) {
          input_PID_cm[BACK] = (double)input_PID_inter[BACK] * output_motor_direction[BACK];
        } else {
          input_PID_cm[BACK] = (double)input_PID_inter[BACK] * (double)current_motor_direction[BACK];
        }
        #if (DEBUG_MODE) 
          SerialUSB.print("Input PID back, el valor es : ");
          SerialUSB.println(input_PID_inter[BACK], 8);
          SerialUSB.print("Input PID back, el valor en cm es : ");
          SerialUSB.println(input_PID_cm[BACK], 8);
        #endif
      }

      input_PID_cm[BACK] = input_PID_cm[BACK] > MAX_VEL_CM_S ? MAX_VEL_CM_S : input_PID_cm[BACK];
      input_PID_cm[BACK] = input_PID_cm[BACK] < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : input_PID_cm[BACK];

      input_PID[BACK] = doubleMap(input_PID_cm[BACK], -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);
      sensor[2] = input_PID[BACK];
      
      #if (USE_PID_MODE) 
        // PID calculation and command
        PID_motor[FRONT].Compute();

        salida_pwm_motor[FRONT] = output_PID[FRONT] ;
        salida_pwm_motor_cm[FRONT] = MAX_VEL_CM_S * salida_pwm_motor[FRONT];
        
        PID_motor[BACK].Compute();
        

        salida_pwm_motor[BACK] = output_PID[BACK] ;
        salida_pwm_motor_cm[BACK] = MAX_VEL_CM_S * salida_pwm_motor[BACK];

      #else 
        salida_pwm_motor[FRONT] = set_point_PID[FRONT];

        salida_pwm_motor[BACK] = set_point_PID[BACK];


      #endif
      
      salida_pwm_motor[FRONT] = salida_pwm_motor[FRONT] > 1.0 ? 1 : salida_pwm_motor[FRONT];
      salida_pwm_motor[FRONT] = salida_pwm_motor[FRONT] < -1.0 ? -1 : salida_pwm_motor[FRONT];
      sensor[3] = output_PID[FRONT];
      if (salida_pwm_motor[FRONT] > 0.001) {
        digitalWrite(z_f_pin_motor[FRONT], forward_direction);
        output_motor_direction[FRONT] = FORWARD_ROTATION_STATE; 
      } else if (salida_pwm_motor[FRONT] < -0.001){ 
        digitalWrite(z_f_pin_motor[FRONT], backward_direction);
        output_motor_direction[FRONT] = BACKWARD_ROTATION_STATE;
      } else {
        output_motor_direction[FRONT] = STOP_STATE;
      }
      salida_pwm_motor[FRONT] = doubleMap(fabs(salida_pwm_motor[FRONT]), 0.0, 1.0, PWM_PERIODO_US_MIN, PWM_PERIODO_US);
      pwm_motor_front.set_duty(salida_pwm_motor[FRONT]);

      salida_pwm_motor[BACK] = salida_pwm_motor[BACK] > 1.0 ? 1 : salida_pwm_motor[BACK];
      salida_pwm_motor[BACK] = salida_pwm_motor[BACK] < -1.0 ? -1 : salida_pwm_motor[BACK];
      sensor[4] = output_PID[BACK];
      if (salida_pwm_motor[BACK] > 0.001) {
        digitalWrite(z_f_pin_motor[BACK], forward_direction);
        output_motor_direction[BACK] = FORWARD_ROTATION_STATE; 
      } else if (salida_pwm_motor[BACK] < -0.001){ 
        digitalWrite(z_f_pin_motor[BACK], backward_direction);
        output_motor_direction[BACK] = BACKWARD_ROTATION_STATE;
      } else {
        output_motor_direction[BACK] = STOP_STATE;
      }
      salida_pwm_motor[BACK] = doubleMap(fabs(salida_pwm_motor[BACK]), 0.0, 1.0, PWM_PERIODO_US_MIN, PWM_PERIODO_US);
      pwm_motor_back.set_duty(salida_pwm_motor[BACK]);

      sensor[5] = set_point_PID[FRONT];
      sensor[6] = set_point_PID[BACK];

      #if (DEBUG_MODE) 
        SerialUSB.print("Sensado motor front en cm/s es : ");
        SerialUSB.println(sensor[1],8);
        SerialUSB.print("Sensado motor back en cm/s es: ");
        SerialUSB.println(sensor[2],8);
        //SerialUSB.print("Metros en ms motor : ");
        //SerialUSB.println(cm_por_periodo, 7);
        //SerialUSB.print("distancia_periodo motor : ");
        //SerialUSB.println(distancia_periodo, 3);
        //SerialUSB.print("perimetro_rueda motor : ");
        //SerialUSB.println(perimetro_rueda, 3);
        //SerialUSB.print("us_to_s motor : ");
        //SerialUSB.println(us_to_s, 7);
        SerialUSB.print("Periodo motor front : ");
        SerialUSB.println(period_motor_us[FRONT]);
        SerialUSB.print("Periodo motor back : ");
        SerialUSB.println(period_motor_us[BACK]);
        #if (USE_PID_MODE)
          SerialUSB.print("PID output motor front in cm/s : ");
          SerialUSB.println(output_PID[FRONT],8);
          SerialUSB.print("PID output motor back in cm/s : ");
          SerialUSB.println(output_PID[BACK], 8);
        #else
          SerialUSB.print("PWM out motor front in cm/s : ");
          SerialUSB.println(salida_pwm_motor[FRONT]);
          SerialUSB.print("PWM out motor back in cm/s : ");
          SerialUSB.println(salida_pwm_motor[BACK]);
        #endif
      #endif
      
      
      digitalWrite(EVENT_CONTROL_DEBUG_PIN, LOW);
    }
    // MODO = 0 --> STOP
    if (modo == 0) {
      // Incluir Aqui codigo de parada
      pwm_motor_front.set_duty(PWM_PERIODO_US_MIN);
      pwm_motor_back.set_duty(PWM_PERIODO_US_MIN);
      digitalWrite(el_pin_motor[FRONT], LOW);
      digitalWrite(el_pin_motor[BACK], LOW);
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
        actuador[k] = actuador[k] * (-1.0);
      }
      actuador[k] = Kactuador * actuador[k];
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
    comunication_control_count = 0;
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
    Serial3.flush();
    digitalWrite(pin_RST_RS485, LOW);
  }
}//----------------   FIM MAIN ---------------------------------

//*************************************************************************************
//****************** XXX  INTERRUPCION TIMER DE 10 ms  XXX  *****************************
//*************************************************************************************
void funcion_t1() {
  evento_control = 1;
  contador++;
  comunication_control_count++;
  if (contador >= 100) {
    // Tarea cada 100 ms
    evento_tarea_1 = 1; 
    contador = 0;
  }
}
