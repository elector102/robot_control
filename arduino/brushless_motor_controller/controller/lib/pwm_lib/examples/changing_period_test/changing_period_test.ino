/**
 ** pwm_lib library
 ** Copyright (C) 2015, 2016
 **
 **   Antonio C. Domínguez Brito <adominguez@iusiani.ulpgc.es>
 **     División de Robótica y Oceanografía Computacional <www.roc.siani.es>
 **     and Departamento de Informática y Sistemas <www.dis.ulpgc.es>
 **     Universidad de Las Palmas de Gran  Canaria (ULPGC) <www.ulpgc.es>
 **  
 ** This file is part of the pwm_lib library.
 ** The pwm_lib library is free software: you can redistribute it and/or modify
 ** it under  the  terms of  the GNU  General  Public  License  as  published  by
 ** the  Free Software Foundation, either  version  3  of  the  License,  or  any
 ** later version.
 ** 
 ** The  pwm_lib library is distributed in the hope that  it  will  be  useful,
 ** but   WITHOUT   ANY WARRANTY;   without   even   the  implied   warranty   of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR  PURPOSE.  See  the  GNU  General
 ** Public License for more details.
 ** 
 ** You should have received a copy  (COPYING file) of  the  GNU  General  Public
 ** License along with the pwm_lib library.
 ** If not, see: <http://www.gnu.org/licenses/>.
 **/
/*
 * File: changing_period_test.ino 
 * Description: This is a basic example illustrating the use of li-
 * brary pwm_lib. It generates a PWM signal with dynamicaclly changes
 * its period.
 * Date: September 28th, 2016
 * Author: Antonio C. Dominguez-Brito <adominguez@iusiani.ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

#define PERIODS 16 
uint32_t periods[PERIODS]= // hundredths of usecs (1e-8 secs.)
{
  10,      // 0.1 usecs. 
  100,     // 1 usec.
  1000,    // 10 usecs. 
  10000,   // 100 usecs. 
  100000,  // 1000 usecs.
  1000000, // 10000 usecs.
  10000000,// 100000 usecs.
  50000000,// 500000 usecs.
  50000000,// 500000 usecs.
  10000000,// 100000 usecs.
  1000000, // 10000 usecs.
  100000,  // 1000 usecs.
  10000,   // 100 usecs. 
  1000,    // 10 usecs. 
  100,     // 1 usec.
  10,      // 0.1 usecs. 
};
//{
//  100,  // 1 usecs. 
//  200,  // 2 usec.
//  400,  // 4 usecs. 
//  800,  // 8 usecs. 
//  1600, // 16 usecs.
//  3200, // 32 usecs.
//  6400, // 64 usecs.
//  12800,// 128 usecs.
//  25600,// 256 usecs.
//  12800,// 128 usecs.
//  6400, // 64 usecs.
//  3200, // 32 usecs.
//  1600, // 16 usecs. 
//  800,  // 8 usecs. 
//  400,  // 4 usec.
//  200,  // 2 usecs. 
//};
uint32_t period=0;

#define CAPTURE_TIME_WINDOW 1000000 // usecs
#define DUTY_KEEPING_TIME 1500 // msecs

// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;

// To measure PWM signals generated by the previous pwm objects, we will use
// capture objects of tc_lib library as "oscilloscopes" probes. So we will 
// use capture object capture_tc0 for measueing the PWM signal on pin 35.
// IMPORTANT: Take into account that for TC0 (TC0 and channel 0) the TIOA0 is
// PB25, which is pin 2 for Arduino DUE, so capture_tc0's capture pin is pin
// 2. For the correspondence between all TIOA inputs for the 
// different TC module channels, you should consult uC Atmel ATSAM3X8E 
// datasheet in section "36. Timer Counter (TC)"), and the Arduino pin mapping 
// for the DUE.
// All in all, to meausure pwm outputs in this example you should connect the 
// PWM output of pin 35 to object capture_tc0's pin 2.

capture_tc0_declaration(); // TC0 and channel 0
auto& capture_pin2=capture_tc0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // starting PWM signals
  pwm_pin35.start(periods[period],(periods[period]>>1));
  Serial.println("===============================================================");
  Serial.print("period "); Serial.print(period); Serial.print(": ");
  Serial.print(static_cast<double>(periods[period])/100); Serial.print(" usecs. (duty=");
  Serial.print(static_cast<double>(periods[period]>>1)/100); Serial.println(")");
  Serial.println("===============================================================");
  period=(period+1); //&0x0F;

  // initialization of capture objects
  capture_pin2.config(CAPTURE_TIME_WINDOW);
}

void loop() {
  // put your main code here,to run repeatedly:
  uint32_t status,captured_duty,captured_period;

  capture_pin2.restart();

  delay(DUTY_KEEPING_TIME);

  status=capture_pin2.get_duty_and_period(captured_duty,captured_period);
  Serial.print("[PIN 35 -> PIN 2] captured duty: "); 
  Serial.print(
    static_cast<double>(captured_duty)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.print(" usecs. captured period: ");
  Serial.print(
    static_cast<double>(captured_period)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.println(" usecs.");
  Serial.println("===============================================================");

  if(!pwm_pin35.set_period_and_duty(periods[period],(periods[period]>>1)))
  {
    Serial.print("[ERROR] set_period_and_duty(");
    Serial.print(periods[period]);
    Serial.print(",");
    Serial.print((periods[period]>>1));
    Serial.println(") failed!");
  }
  Serial.println("===============================================================");
  Serial.print("period "); Serial.print(period); Serial.print(": ");
  Serial.print(static_cast<double>(periods[period])/100); Serial.print(" usecs. (duty=");
  Serial.print(static_cast<double>(periods[period]>>1)/100); Serial.println(")");
  Serial.println("===============================================================");
  period=(period+1)&0x0F;
}

