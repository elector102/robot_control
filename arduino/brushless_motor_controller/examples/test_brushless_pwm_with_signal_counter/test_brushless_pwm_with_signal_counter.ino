#include "pwm_lib.h"

//#include <DueTimer.h>
#include "tc_lib.h"

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
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;

int salida_pwm =0;
int entrada_analogica =0;

#define CAPTURE_TIME_WINDOW 40000000 // usecs

capture_tc0_declaration(); // TC0 and channel 0
auto& capture_pin2=capture_tc0;

uint32_t status,duty,period,pulses;


void setup() {
  // initialization of capture objects
  capture_pin2.config(CAPTURE_TIME_WINDOW);
  // put your setup code here, to run once:
  //seteado a 1Khz para probar
  pwm_pin35.start(PWM_PERIODO_US,1000);
  pinMode(vr_pin, OUTPUT);
  pinMode(z_f_pin, OUTPUT);
  pinMode(signal_pin, INPUT);
  pinMode(el_pin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //pwm_pin35.set_duty(0.5);
  // habilitar motor
  digitalWrite(el_pin, HIGH);
  // sentido de giro
  digitalWrite(z_f_pin, HIGH);
  entrada_analogica = analogRead(A0);
  SerialUSB.println("---------------------------------------------");
  SerialUSB.println(entrada_analogica);
  salida_pwm = map(entrada_analogica, 0, 1023, 0, PWM_PERIODO_US);
  SerialUSB.println(salida_pwm);
  status = capture_pin2.get_duty_period_and_pulses(duty,period,pulses);
  SerialUSB.print("los pulsos son : ");
  SerialUSB.println(pulses);
  pwm_pin35.set_duty(salida_pwm);
  delay(100);
/*  for (int i = 0; i < 100; i++){
    
    salida_pwm = map(i, 0, 99, 0, PWM_PERIODO_US);
    pwm_pin35.set_duty(salida_pwm);
    delay(1000);
  }
*/
}
