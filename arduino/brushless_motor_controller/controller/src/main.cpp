#include <Arduino.h>
#include <DueTimer.h>
#include "pwm_lib.h"
#include "tc_lib.h"
#include <PID_v1.h>
#include <pinout.h>
#include <MCP3208.h>
#include <SPI.h>


void funcion_t1();
#define RIGHT 0
#define LEFT 1
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

#define STOP_STATE 0
#define FORWARD_ROTATION_STATE 1
#define BACKWARD_ROTATION_STATE -1
#define ROTATION_STATE 2

int current_motor_direction[2] = { STOP_STATE, STOP_STATE};
int output_motor_direction[2] = { STOP_STATE, STOP_STATE};

int last_motor_input_update_rate[2] = {0, 0};
int last_motor_input[2] = {0, 0};

#define CORRECT_ROTATION 0
#define BRAKE_ROTATION 1
#define CONFIRM_ROTATION_CHANGE 2
int state_of_rotation[2] = { STOP_STATE, STOP_STATE};

#define PWM_PERIODO_US 10000 //10khz
#define PWM_PERIODO_US_MIN 0
#define DEBUG_MODE false
#define USE_PID_MODE true
#define USE_CURRENT_SENSOR false

#if USE_CURRENT_SENSOR
  MCP3208 adc(PIN_SPI_CS2);
#endif

bool first_time = true;
// Diametro de la rueda en cm. cm = 2.54 * pulgadas
#define DIAMETRO_CM 24.13 // 9.5 pulgadas
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


#define CAPTURE_TIME_WINDOW 400000 // usecs

double last_set_point_PID[2] = {0, 0};
double set_point_PID[2] = {0, 0};
double output_PID[2] = {0, 0};
double input_PID[2] = {0, 0};

#if (USE_PID_MODE)
  PID PID_motor[2] = {
        PID(&input_PID[FRONT], &output_PID[FRONT], &set_point_PID[FRONT], 1, 0, 0, DIRECT),
        PID(&input_PID[BACK], &output_PID[BACK], &set_point_PID[BACK], 1, 0, 0, DIRECT)
      };
  
#endif

byte cantidad_sensores;
byte cantidad_actuadores;

// BUFFERS
unsigned int envio[100];
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
int motor_status[2] = {STOP_STATE, STOP_STATE};

void stopMotor(int motor) {
  if (motor_status != STOP_STATE) {
    digitalWrite(el_pin_motor[motor], LOW);
    //delay(1);
    digitalWrite(PIN_BRAKE[motor], HIGH);
//    if (motor == FRONT) {
//      pwm_motor_brake_front.set_duty(0);
//    } else {
//      pwm_motor_brake_back.set_duty(0);
//    }
  }
}

void runMotor(int motor) {
//  if (motor == FRONT) {
//    pwm_motor_brake_front.set_duty(PWM_PERIODO_US);
//  } else {
//    pwm_motor_brake_back.set_duty(PWM_PERIODO_US);
//  }
  digitalWrite(PIN_BRAKE[motor], LOW);
  //delay(1);
  digitalWrite(el_pin_motor[motor], HIGH);
}


bool estimateRotationDirection(const uint motor, const int output_motor_direction){
  int motor_state = 0;
  if (fabs(input_PID[motor]) < 0.01) {
    motor_state = STOP_STATE;
  } else {
    motor_state = ROTATION_STATE;
  }  
  switch(state_of_rotation[motor]) {
    case CORRECT_ROTATION:
      runMotor(motor);
      if (output_motor_direction != current_motor_direction[motor]) {
        state_of_rotation[motor] = BRAKE_ROTATION;
      }
      break;
    case BRAKE_ROTATION:
      stopMotor(motor);
      if ( motor_state == STOP_STATE) {
        current_motor_direction[motor] = STOP_STATE;
        state_of_rotation[motor] = CONFIRM_ROTATION_CHANGE;
      }
      //if (last_motor_input[motor] < (fabs(input_PID[motor]) - 0.025 * MAX_VEL_CM_S) ) {
      //  state_of_rotation[motor] = CONFIRM_ROTATION_CHANGE;
      //} 
      break;
    case CONFIRM_ROTATION_CHANGE:
      runMotor(motor);
      if (motor_state == ROTATION_STATE) {
        current_motor_direction[motor] = output_motor_direction;
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
#if USE_CURRENT_SENSOR
  bool estimateRotationDirectionWithCurrentSensor(const uint motor, const int output_motor_direction) {

  }
#endif
void initializeMotor(int motor) {
  pinMode(PIN_POWER_ENABLE[motor], OUTPUT);
  pinMode(PIN_BRAKE[motor], OUTPUT);
  // brake motor
  digitalWrite(PIN_BRAKE[motor], LOW);
  // turno off brushless driver
  digitalWrite(PIN_POWER_ENABLE[motor], HIGH);
  if (motor == FRONT) {
    // Initialization of capture objects
    capture_motor_front.config(CAPTURE_TIME_WINDOW);
    pwm_motor_front.start(PWM_PERIODO_US, 1000);
    pwm_motor_front.set_duty(PWM_PERIODO_US_MIN);

    // brake
    pwm_motor_brake_front.start(PWM_PERIODO_US, PWM_PERIODO_US);
    pwm_motor_brake_front.set_duty(PWM_PERIODO_US);
  } else {  
    // Initialization of capture objects
    capture_motor_back.config(CAPTURE_TIME_WINDOW);
    pwm_motor_back.start(PWM_PERIODO_US, 1000);
    pwm_motor_back.set_duty(PWM_PERIODO_US_MIN);
    // brake
    pwm_motor_brake_back.start(PWM_PERIODO_US, PWM_PERIODO_US);
    pwm_motor_brake_back.set_duty(PWM_PERIODO_US);
  }
  pinMode(z_f_pin_motor[motor], OUTPUT);
  pinMode(el_pin_motor[motor], OUTPUT);
  stopMotor(motor);

  digitalWrite(z_f_pin_motor[motor], LOW);


  #if (USE_PID_MODE)
    //turn the PID on
    PID_motor[motor].SetMode(AUTOMATIC);
    PID_motor[motor].SetSampleTime(10);// in ms
    PID_motor[motor].SetOutputLimits(-1.0, 1.0);
  #endif  
}

void initializeBrushlessDriver(int motor) {
  digitalWrite(PIN_POWER_ENABLE[motor], HIGH);
  stopMotor(motor);
  digitalWrite(el_pin_motor[motor], HIGH);

  // Init brushless driver
  digitalWrite(PIN_POWER_ENABLE[motor], LOW);
  delay(1);
  stopMotor(motor);
  

}
//*************************************************************************************
//********* XXX  INITIALIZATION AND SETUP DE LOS PINOS Y CONTROLADOR XXX  *************
//*************************************************************************************
void setup() {
  #if USE_CURRENT_SENSOR
    adc.begin();
  #endif
  initializeMotor(FRONT);
  initializeMotor(BACK);

  pinMode(pin_RST_RS485, OUTPUT);
  // desactiva el modo transmision en el conversor RS485
  digitalWrite(pin_RST_RS485, LOW);

  pinMode(EVENT_CONTROL_DEBUG_PIN, OUTPUT);
  digitalWrite(EVENT_CONTROL_DEBUG_PIN, LOW);

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
}//----------------   FIM DO SETUP Y PARAMETROS -----------------------------

void updateSetPoint(const int motor, const float set_point_value) {
    double signal_set_point;
    signal_set_point = (double)set_point_value / (double)Kactuador;
      
    if (last_set_point_PID[motor] != signal_set_point) {
      
      signal_set_point = signal_set_point > MAX_VEL_CM_S ? MAX_VEL_CM_S : signal_set_point;
      signal_set_point = signal_set_point < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : signal_set_point;

      set_point_PID[motor] = doubleMap(signal_set_point, -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);

      last_set_point_PID[motor] = signal_set_point; // cm/s
    }
    #if (DEBUG_MODE) 
      SerialUSB.print("el valor del actuador frontal con signo es : ");
      SerialUSB.println(set_point_value,8);
    #endif
}

double updateMotorSpeed(const int motor) {
  double input_PID_inter, input_PID;
  uint32_t status_motor, duty_motor, period_motor, pulses_motor;
  double period_motor_us = 0;

  if (motor == FRONT) {
    status_motor = capture_motor_front.get_duty_period_and_pulses(duty_motor, period_motor, pulses_motor);
    period_motor_us = period_motor / capture_motor_front.ticks_per_usec();
  } else if (motor == BACK) {
    status_motor = capture_motor_back.get_duty_period_and_pulses(duty_motor, period_motor, pulses_motor);
    period_motor_us = period_motor / capture_motor_back.ticks_per_usec();
  }

  input_PID_inter = (distancia_periodo * us_to_s)/ period_motor_us;

  if isinf(input_PID_inter) {
    input_PID_inter = 0.0;
  } else {
    if (current_motor_direction[motor] == STOP_STATE) {
      input_PID_inter = input_PID_inter * output_motor_direction[motor];
    } else {
      input_PID_inter = input_PID_inter * current_motor_direction[motor];
    }
  }
  input_PID_inter = input_PID_inter > MAX_VEL_CM_S ? MAX_VEL_CM_S : input_PID_inter;
  input_PID_inter = input_PID_inter < -MAX_VEL_CM_S ? -MAX_VEL_CM_S : input_PID_inter;

  input_PID = doubleMap(input_PID_inter, -MAX_VEL_CM_S, MAX_VEL_CM_S, -1.0, 1.0);

  #if (DEBUG_MODE)
    SerialUSB.print("Sensado motor back en cm/s es: ");
    SerialUSB.println(input_PID,8);
  #endif

  return input_PID;
}

int setMotorDirection(const int motor, const double velocity) {
  int output_motor_direction;
  if (velocity > 0.01) {
    digitalWrite(z_f_pin_motor[motor], forward_direction);
    output_motor_direction = FORWARD_ROTATION_STATE; 
  } else if (velocity < -0.01){ 
    digitalWrite(z_f_pin_motor[motor], backward_direction);
    output_motor_direction = BACKWARD_ROTATION_STATE;
  } else {
    output_motor_direction = STOP_STATE;
  }
  return output_motor_direction;
}

double setMotorSpeedAndDirection(const int motor) {
  double salida_motor = 0;
  double salida_pwm_motor;
  #if (USE_PID_MODE) 
    // PID calculation and command
    PID_motor[motor].Compute();
    
    salida_motor = output_PID[motor] ;

  #else 
    salida_motor = set_point_PID[motor];
  #endif

  output_motor_direction[motor] = setMotorDirection(motor, salida_motor);

  salida_pwm_motor = doubleMap(fabs(salida_motor), 0.0, 1.0, PWM_PERIODO_US_MIN, PWM_PERIODO_US);
  if (motor == FRONT)
    pwm_motor_front.set_duty(salida_pwm_motor); 
  else
    pwm_motor_back.set_duty(salida_pwm_motor);

  #if (DEBUG_MODE)
    SerialUSB.print("PWM out motor front in cm/s : ");
    SerialUSB.println(salida_pwm_motor);
  #endif

  return salida_motor;
} 
#if USE_CURRENT_SENSOR
  void readCurrentSensor (int motor, int *current_sensor) {
    int sensor_group = 2*(motor + 1 ) + motor;
    for (int i = 0; i < 3; i++) {
      current_sensor[i] = adc.analogRead(sensor_group + i);
    }
    #if DEBUG_MODE
      SerialUSB.print("current sensor value of motor ");
      SerialUSB.println(motor);
      SerialUSB.println(current_sensor[0]);
      SerialUSB.println(current_sensor[1]);
      SerialUSB.println(current_sensor[2]);
    #endif
  }
#endif

//*************************************************************************************
//***************************** XXX  MAIN  XXX  ***************************************
//*************************************************************************************
void loop() {
  // Cada 10 ms ejecuta este codigo - Lazo de control
  if (evento_control == 1) {
    if (comunication_control_count >= 10) {
      modo = 0;
    }
    
    updateSetPoint(FRONT, actuador[1]);
    updateSetPoint(BACK, actuador[5]);
    sensor[5] = set_point_PID[FRONT];
    sensor[6] = set_point_PID[BACK];

    estimateRotationDirection(FRONT, output_motor_direction[FRONT]);
    input_PID[FRONT] = updateMotorSpeed(FRONT);
    sensor[1] = input_PID[FRONT];

    estimateRotationDirection(BACK, output_motor_direction[BACK]);
    input_PID[BACK] = updateMotorSpeed(BACK);
    sensor[2] = input_PID[BACK];

    #if USE_CURRENT_SENSOR
      int current_sensor[3];
      readCurrentSensor(FRONT, current_sensor);
      sensor[7] = current_sensor[0];
      sensor[8] = current_sensor[1];
      sensor[9] = current_sensor[2];

      readCurrentSensor(BACK, current_sensor);
      sensor[10] = current_sensor[0];
      sensor[11] = current_sensor[1];
      sensor[12] = current_sensor[2];
    #endif
    // MODO = 0 --> STOP
    if (modo == 0) {
      // Incluir Aqui codigo de parada
      pwm_motor_front.set_duty(PWM_PERIODO_US_MIN);
      pwm_motor_back.set_duty(PWM_PERIODO_US_MIN);
      stopMotor(FRONT);
      stopMotor(BACK);
      PID_motor[FRONT].SetTunings(actuador[2], actuador[3], actuador[4]);
      PID_motor[BACK].SetTunings(actuador[6], actuador[7], actuador[8]);

    } else if (modo == 1) {

      if (first_time == true) {
        initializeBrushlessDriver(FRONT);
        initializeBrushlessDriver(BACK);
        first_time = false;
      }
      
      digitalWrite(EVENT_CONTROL_DEBUG_PIN, HIGH);
      // Ejemplo: espejo en dato[0]
      sensor[0] = actuador[0]; 

      sensor[3] = setMotorSpeedAndDirection(FRONT);
      sensor[4] = setMotorSpeedAndDirection(BACK);

      digitalWrite(EVENT_CONTROL_DEBUG_PIN, LOW);
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
    // Tarea cada 1000 ms
    evento_tarea_1 = 1; 
    contador = 0;
  }
}
