#define IR_RECEIVE_PIN 9

// Moving forwards
#define CALIBRATION_TIME 2
#define CALIBRATION_AT_PWM_100 75
#define CALIBRATION_AT_PWM_255 188

// Turning on the spot
#define TURN_LR_PWM 125
// The gradient determined by the turning on the spot linear regression
// PWM = 125
// f(x) == 248 x - 40
// f(x) + 40 == 248 x
// f(x) + 40 / 248 == x
// f(x) = a x -b
#define TURN_SPOT_F_OF_X_A 0.248
#define TURN_SPOT_F_OF_X_B 40

// Driving forward and back trim
#define FORWARD_STEERING_TRIM 0
#define BACKWARD_STEERING_TRIM 0
// IR Control turn amount in degrees
#define IR_TURN_MAGNITUDE_DEGREES 20

#define MILLISECONDS 1000

// IR control commands on NEC protocol in HEX

#define IR_FORWARD 0x46
#define IR_BACKWARD 0x15
#define IR_LEFT 0x44
#define IR_RIGHT 0x43
#define IR_OK 0x40
#define IR_1 0x16
#define IR_2 0x19
#define IR_3 0xD
#define IR_4 0xC
#define IR_5 0x18
#define IR_6 0x5E
#define IR_7 0x8
#define IR_8 0x1C
#define IR_9 0x5A
#define IR_0 0x52
#define IR_ASTERISK 0x42
#define IR_HASH 0x4A

#define IR_DRIVE_DISTANCE_INCREMENT 10

// RGB LED
#define PIN_RBGLED 4
#define NUM_LEDS 1
