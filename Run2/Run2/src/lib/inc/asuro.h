#ifndef ASURO_H
#define ASURO_H


// Include the necessary programs.
#include <interrupt.h>
#include <stdlib.h>


// Define the names of the constants.
#define FALSE  0
#define TRUE   1

#define OFF    0
#define ON     1

#define GREEN  1
#define RED    2
#define YELLOW 3

#define FORWARD   (1 << PB5)                      // Motor forward.
#define REVERSE   (1 << PB4)                      // Motor reverse.
#define BRAKE     0x00                            // Motor brake.
#define FREE      (1 << PB4) | (1 << PB5)         // Motor free running.

#define GREEN_LED_ON  PORTB |=  GREEN_LED         // Status LED.
#define GREEN_LED_OFF PORTB &= ~GREEN_LED         // Status LED.
#define RED_LED_ON    PORTD |=  RED_LED           // Status LED.
#define RED_LED_OFF   PORTD &= ~RED_LED           // Status LED.

#define IRTX            (1 << PB3)                // Port for IR LED.
#define GREEN_LED       (1 << PB0)                // Port for green LED (Status LED).
#define RED_LED         (1 << PD2)                // Port for red LED (Status LED).

#define PWM             (1 << PB1) | (1 << PB2)   // Ports for PWM control of motors.
#define LEFT_DIRECTION  (1 << PD4) | (1 << PD5)   // Ports for left motor.
#define RIGHT_DIRECTION (1 << PB4) | (1 << PB5)   // Ports for right motor.

#define SWITCHES     (1 << PD3)                   // Port for collision detection.
#define SWITCHES_ON  PORTD |=  SWITCHES           // Collision detection on.
#define SWITCHES_OFF PORTD &= ~SWITCHES           // Collision detection off.

#define BATTERY     (1 << MUX0) | (1 << MUX2)     // ADC port select for battery voltage.
#define COLLISION_SWITCH (1 << MUX2)              // ADC port select for collision detection.
#define LINE_LEFT   (1 << MUX0) | (1 << MUX1)     // ADC port select for left line-follow sensor.
#define LINE_RIGHT  (1 << MUX1)                   // ADC port select for right line-follow sensor.
#define LINE_LED    (1 << PD6)                    // LED of line-follow sensor.

#define ODOMETRY_LED     (1 << PD7)               // LEDs of line-follow sensors.
#define ODOMETRY_LED_ON  PORTD |= ODOMETRY_LED    // LEDs of line-follow sensors ON.
#define ODOMETRY_LED_OFF PORTD &= ~ODOMETRY_LED   // LEDs of line-follow sensors OFF.

#define ODOMETRY_LEFT   (1 << MUX0)               // ADC port select for odometry of the left wheel.
#define ODOMETRY_RIGHT  0                         // ADC port select for odometry of the right wheel.

#define START_TMR0         (1 << CS02)            // Start counter 0, pre-scalar = 256.
#define STOP_TMR0          0x00                   // Stop counter 0.
#define COUNTER_START_TMR0 0xE0                   // Count from here to overflow = interrupt.
#define  WHITE_SURFACE 400
#define  DARK_SURFACE  200


// Global variables.
volatile unsigned char counter36kHz;              // For Sleep function and system time.
volatile unsigned long timeBase;                  // Variable for system time.
volatile unsigned long systemTime;                // System time in ms.
volatile          int  lineFollowLeft;            // Value of left line-follow sensor.
volatile          int  lineFollowRight;           // Value of left right follow sensor.
volatile          int  lineFollowDelta;           // Difference in values of left and right line-follow sensor.  (delta = left - right).
volatile          int  odometryLeft;              // Value of left odometry sensor.
volatile          int  odometryRight;             // Value of right odometry sensor.
volatile          int  encoderLeft;               // Counter for the number of colour changes of the reflecting sticker on the left wheel.  
volatile          int  encoderRight;              // Counter for the number of colour changes of the reflecting sticker on the right wheel.  
volatile unsigned char switchesActive;            // Collision or emergency stop.

// Declaration of functions.
unsigned long SystemTime(void);                                                 // System time in ms.
void Init(void);                                                                // Init ports etc.
void MotorSpeed(int leftSpeed, int rightSpeed);                                 // Control of motors.
void StatusLED(unsigned char colour);                                           // Control of status LED.
void LineLED(unsigned char status);                                             // Control of line-follow LED.
void BackLEDs(unsigned char left, unsigned char right);                         // Control of rear LEDs.
int  Battery(void);                                                             // Battery voltage.
void SerWrite(unsigned char *data, unsigned char length);                       // Send ASCII via the IR communication.
void SerRead(unsigned char *data, unsigned char length, unsigned int timeout);  // Receive ASCII via the IR communication.
void PrintInt(int value);                                                       // Send Integer  via the IR communication.
unsigned char Switches (void);                                                  // Value of the switches.
void SwitchDetection(int);                                                      // Start/stop switch-detection via external interrupt.
void Sleep(unsigned char time);                                                 // Pause in 1/36 ms.
void Msleep(int time);                                                           // Pause in ms.
void SensorsViaInterrupt(int status);                                           // On/off reading odometry and line-follow sensors via Interrupt.
void SetEncoders(int left, int right);                                          // Set value of encoders.
void Drive(int distance, int speed);                                            // Drive with odometry.
void Turn(int degrees, int speed);                                              // Turn with odometry.
void EmergencyStop(void);                                                       // Wait for reset of the emergency stop.


#endif                                                                          // ASURO_H
