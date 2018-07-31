#include "pwm_lib.h"

//#include <DueTimer.h>
#include "tc_lib.h"
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,10,10,0,P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                            //P_ON_E (Proportional on Error) is the default behavior
using namespace arduino_due::pwm_lib;



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

// this object uses PWM channel 0
pwm<pwm_pin::PWML6_PC23> pwm_pin7;

int salida_pwm =0;
int entrada_analogica =0;

float periodo_ms =0;

#define CAPTURE_TIME_WINDOW 80000000 // usecs

capture_tc1_declaration(); // TC0 and channel 1
auto& capture_pinA7=capture_tc1;


capture_tc6_declaration(); // TC0 and channel 2
auto& capture_pin5=capture_tc6;


uint32_t status,duty,period,pulses,ra;


void setup() {
  // initialization of capture objects
  capture_pin5.config(CAPTURE_TIME_WINDOW);
  // put your setup code here, to run once:
  //seteado a 1Khz para probar
  pwm_pin7.start(PWM_PERIODO_US,1000);
  pinMode(vr_pin, OUTPUT);
  pinMode(z_f_pin, OUTPUT);
  pinMode(signal_pin, INPUT);
  pinMode(el_pin, OUTPUT);
  //initialize the variables we're linked to
  Setpoint = map(analogRead(A0), 0, 1024, 0, PWM_PERIODO_US);
  Input = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-PWM_PERIODO_US,PWM_PERIODO_US);
  // sentido de giro
  digitalWrite(z_f_pin, HIGH);
  // habilitar motor
  digitalWrite(el_pin, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  //pwm_pin35.set_duty(0.5);


  Setpoint = map(analogRead(A0), 0, 1024, 0, PWM_PERIODO_US);
  SerialUSB.println("---------------------------------------------");
  SerialUSB.println(Setpoint);
  //salida_pwm = map(entrada_analogica, 0, 1023, 0, PWM_PERIODO_US);
  //SerialUSB.println(salida_pwm);
  status = capture_pin5.get_duty_period_ra_and_pulses(duty,period,pulses,ra);
  periodo_ms = period/42;
  SerialUSB.print("los pulsos son : ");
  SerialUSB.print(pulses);
  SerialUSB.print("\t");
  SerialUSB.print("los duty son : ");
  SerialUSB.print(duty);
  SerialUSB.print("\t");
  SerialUSB.print("los ms son : ");
  SerialUSB.println(periodo_ms);
  Input = map(periodo_ms,0 ,30000, 0, PWM_PERIODO_US );
  myPID.Compute();
  SerialUSB.print("los tiempo por vuelta es : ");
  SerialUSB.print(periodo_ms/42);
  SerialUSB.print("\t");
  SerialUSB.print("los RPM son : ");
  SerialUSB.println((1/((periodo_ms/21)/1000)));
  //capture_pin5.config(CAPTURE_TIME_WINDOW);
  SerialUSB.print("los ticks por us son : ");
  SerialUSB.println(capture_pin5.ticks_per_usec());
  SerialUSB.print("la salida es : ");
  SerialUSB.println(Output);
  if (Output < 0){
    digitalWrite(z_f_pin, LOW);
  } else {
    digitalWrite(z_f_pin, HIGH);
  }
  SerialUSB.print("la salida con setpoint es : ");
  SerialUSB.println(Setpoint - abs(Output));
  SerialUSB.print("la SetPoint es : ");
  SerialUSB.println(myPID.GetSetPoint());
  salida_pwm = map(Setpoint - abs(Output), 0, PWM_PERIODO_US*2, 0, PWM_PERIODO_US);
  SerialUSB.print("la salida pwm es : ");
  SerialUSB.println(salida_pwm);
  pwm_pin7.set_duty(salida_pwm);
  //delay(100);
/*  for (int i = 0; i < 100; i++){
    
    salida_pwm = map(i, 0, 99, 0, PWM_PERIODO_US);
    pwm_pin35.set_duty(salida_pwm);
    delay(1000);
  }
*/
}
