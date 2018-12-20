#define BOARD_SIDE RIGHT

#define PWM_PERIODO_US 10000 //10khz
#define PWM_PERIODO_US_MIN 0
#define DEBUG_MODE false
#define USE_PID_MODE true
#define USE_CURRENT_SENSOR true
// Diametro de la rueda en cm. cm = 2.54 * pulgadas
#define DIAMETRO_CM 24.13 // 9.5 pulgadas
// cantidad de periodos que se tienen por vuelta del motor
#define PULSOS_POR_VUELTAS 45.0

#define MAX_VEL_CM_S 1000.0

#define USE_BRAKE true

#define PID_LIMIT 0.8

#define MAP_LIMIT 1.0