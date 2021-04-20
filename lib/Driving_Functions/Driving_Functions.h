

#define AIN1 8  // direction low = forward
#define AIN2 5  // speed PWM
#define BIN1 7  // direction high = forward
#define BIN2 6  // speed PWM

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

#define UART_FORWARD 'F'
#define UART_BACKWARD 'B'
#define UART_LEFT 'L'
#define UART_JOG_LEFT 'l'
#define UART_RIGHT 'R'
#define UART_JOG_RIGHT 'r'
#define UART_180 'V'
#define UART_1 '1'
#define UART_2 '2'
#define UART_3 '3'
#define UART_4 '4'
#define UART_5 '5'
#define UART_6 '6'
#define UART_7 '7'
#define UART_8 '8'
#define UART_9 '9'
#define UART_0 '0'
#define UART_ASTERISK '*'
#define UART_HASH '#'
#define UART_TURN_MAGNITUDE_DEGREES 20
#define UART_JOG_MAGNITUDE_DEGREES 10

#define UART_DRIVE_DISTANCE_INCREMENT 10

#define ULTRASONIC_TRIGGER_PIN 13
#define ULTRASONIC_ECHO_PIN 12

void setupIrReceiver();
void setupDrivingPins();
void calculateCalibration();
void UARTControlInterpreter();
void IrControlInterpreter();
int calculateWheelPWM();
void setSpeed();
int calculateRotationTime(int);
void stopCar();
void forward(int, float);
void backward(int, float);
void turnLeft(int);
void turnRight(int);
void calF();
void calB();
void calL(float);
void calR();
int calculateMaximumSpeed(int);
void UARTHash();
void UARTAsterisk();
int ultrasonicPingDistance();
void setupUltrasonic();
void avoid();