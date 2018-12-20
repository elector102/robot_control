#include <configuration.h>

#define EVENT_CONTROL_DEBUG_PIN 33

// Sentido de giro del motor
const int z_f_pin_motor[2] = {9, 4};
// Pulsos de velocidad de salida
//#define signal_pin_motor_front 10
// Enable control
const int el_pin_motor[2] = {12, 7};

// Definiciones motor front(Delantero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_motor_front;

capture_tc8_declaration(); // TC0 and channel 1 pin A7
capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto &capture_motor_front = capture_tc8;
auto &capture_motor_back = capture_tc6;

// Definiciones motor back(Tracero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> pwm_motor_back;

// motor brake
// analog pin 10
//arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML3_PB19> pwm_motor_brake_front;
// analog pin 9
//arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML2_PB18> pwm_motor_brake_back;

const int pin_RST_RS485 = 10;

const int PIN_SPI_CS2 = 52;

const int PIN_POWER_ENABLE[2] = {A9, 53};

const int PIN_BRAKE[2] = {A10, 51};

const int time_measure = 41;

/// Defines

#define RIGHT 0
#define LEFT 1

#define FRONT 0
#define BACK 1

#if (BOARD_SIDE)
#define forward_direction HIGH
#define backward_direction LOW
const byte IDslave = 1;
#else
#define forward_direction LOW
#define backward_direction HIGH
const byte IDslave = 2;
#endif

#define STOP_STATE 0.0
#define FORWARD_ROTATION_STATE 1.0
#define BACKWARD_ROTATION_STATE -1.0
#define ROTATION_STATE 2.0

#define CORRECT_ROTATION 0
#define BRAKE_ROTATION 1
#define CONFIRM_ROTATION_CHANGE 2

#define CAPTURE_TIME_WINDOW 400000 // usecs
#define ZERO_THRESHOLD 0.000001