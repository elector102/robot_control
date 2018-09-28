#define EVENT_CONTROL_DEBUG_PIN 33

// Sentido de giro del motor
int z_f_pin_motor[2] ={9, 4};
// Pulsos de velocidad de salida
//#define signal_pin_motor_front 10
// Enable control
int el_pin_motor[2] = {12, 7};

// Definiciones motor front(Delantero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_motor_front;

capture_tc8_declaration(); // TC0 and channel 1 pin A7
capture_tc6_declaration(); // TC0 and channel 1 pin pin digital 5
auto& capture_motor_front = capture_tc8;
auto& capture_motor_back = capture_tc6;

// Definiciones motor back(Tracero)
// Defining pwm object using pin 6, pin PC24 mapped to pin 6 on the DUE
// This object uses PWM channel 0
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> pwm_motor_back;

int pin_RST_RS485 = 10;