// ---------------------------------------------------------------------
//
// Asuro library of Richard Lagendijk, version 1.0 / 16-09-2017
//
//
// Alternative library for the Asuro Robot with the following
// amendments:
// - Reading the analogue signals per timer interrupt, the data is
//   continuously available via variables, including difference in the
//   line-follow sensors.
// - Encoder data is continuously available via variables.
// - Control of motors with only 1 variable -255 to +255,
//   forward, backward and braking is determined automatically.
// - Interrupt (external) for handling switch detection.
// - Emergency stop routine.
//
// ---------------------------------------------------------------------


// Include the necessary programs.
#include "inc/asuro.h"
#include "string.h"


// ---------------------------------------------------------------------
// Interrupt routine via timer 2 (36 kHz for IR communication).
// ---------------------------------------------------------------------
ISR(TIMER2_OVF_vect) {
   
   TCNT2 += 0x25;
   counter36kHz++;
   if (!counter36kHz) timeBase++;
}


// ---------------------------------------------------------------------
// Interrupt routine via external signal (switch detection).
// ---------------------------------------------------------------------
ISR(INT1_vect) {
   
   switchesActive = 1;

   // Turn off switch detection interrupt.
   SwitchDetection(OFF);  
   
}


// ---------------------------------------------------------------------
// Interrupt routine via timer 0 (reading odometry and line-follow
// sensors).
//
// Per interrupt 1 signal is read. At the next interrupt the next
// signal etc.
//
// ---------------------------------------------------------------------
ISR(TIMER0_OVF_vect) {
   
   static unsigned char counter = 0;                   // Counter for reading the sensors.
   static unsigned char leftSwitchPoint  = FALSE;      // Switch point for odometry sensor (light / dark) left wheel.
   static unsigned char rightSwitchPoint = FALSE;      // Switch point for odometry sensor (light / dark) right wheel.

   // Reading the odometry sensor of the left wheel.
   if (counter == 0) {
      
      ADMUX = (1 << REFS0) | ODOMETRY_LEFT;       // Set the ADC on the signal of the odometry sensor of the left wheel.
      ADCSRA |= (1 << ADSC);                      // Start the ADC conversion.
      while (!(ADCSRA & (1 << ADIF)));            // Wait until the ADC conversion is ready.
      ADCSRA |= (1 << ADIF);                      // Reset ADCIF
      odometryLeft = ADCL + (ADCH << 8);          // Update the variable.

      // If there is a change from dark to light, then encoder + 1.
      if ((odometryLeft < 600) && (leftSwitchPoint == TRUE)) {
         encoderLeft++;
         leftSwitchPoint = FALSE;
      }
      
      // If there is a change from light to dark, then encoder + 1.
      if ((odometryLeft > 700) && (leftSwitchPoint == FALSE)) {
         encoderLeft++;
         leftSwitchPoint = TRUE; 
      }
      
   }
     
   //  Reading the odometry sensor of the right wheel.
   if (counter == 1) {
      
      ADMUX = (1 << REFS0) | ODOMETRY_RIGHT;      // Set the ADC on the signal of the odometry sensor of the right wheel.
      ADCSRA |= (1 << ADSC);                      // Start the ADC conversion.
      while (!(ADCSRA & (1 << ADIF)));            // Wait until the ADC conversion is ready.
      ADCSRA |= (1 << ADIF);                      // Reset ADCIF
      odometryRight = ADCL + (ADCH << 8);         // Update the variable.

      // If there is a change from dark to light, then encoder + 1.
      if ((odometryRight < 650) && (rightSwitchPoint == TRUE)) {
         encoderRight++;
         rightSwitchPoint = FALSE;
      }
      
      // If there is a change from light to dark, then encoder + 1.
      if ((odometryRight > 750) && (rightSwitchPoint == FALSE)) {
         encoderRight++;
         rightSwitchPoint = TRUE; 
      }
      
   }
       
   //  Reading the odometry sensor of the left line-follow sensor.
   if (counter == 2) {
      
      ADMUX = (1 << REFS0) | LINE_LEFT;           // Set the ADC on the signal of the left line-follow sensor.
      ADCSRA |= (1 << ADSC);                      // Start the ADC conversion.
      while (!(ADCSRA & (1 << ADIF)));            // Wait until the ADC conversion is ready.
      ADCSRA |= (1 << ADIF);                      // Reset ADCIF
      lineFollowLeft = ADCL + (ADCH << 8);        // Update the variable.
      
      // Determine the difference of the two line-follow sensors.
      lineFollowDelta = lineFollowLeft - lineFollowRight;
      
   }    
   
   //  Reading the odometry sensor of the right line-follow sensor.
   if (counter == 3) {
      
      ADMUX = (1 << REFS0) | LINE_RIGHT;          // Set the ADC on the signal of the right line-follow sensor.
      ADCSRA |= (1 << ADSC);                      // Start the ADC conversion.
      while (!(ADCSRA & (1 << ADIF)));            // Wait until the ADC conversion is ready.
      ADCSRA |= (1 << ADIF);                      // Reset ADCIF
      lineFollowRight = (((ADCL + (ADCH << 8)) * 160L) / 100L);  // Update the variable, and correct the right photo transistor.
 
      // Determine the difference of the two line-follow sensors.
      lineFollowDelta = lineFollowLeft - lineFollowRight;
   }    
      
   // Next sensor at the next interrupt.
   TCNT0 = COUNTER_START_TMR0;                    // Set counter start for the next interrupt.
   counter++;                                     // Next signal at next interrupt.
   if (counter > 3) counter = 0;                  // All signals read then start over.
     
  
}

 
// ---------------------------------------------------------------------
// Routine for the initialisation of the processor: input, output, timers etc.
// ---------------------------------------------------------------------
void Init(void) {

   // Timer 2 at 36kHz for IR communication.
   TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM20) | (1 << COM21) | (1 << CS20);
   OCR2  = 0x91;                                  // Duty cycle for 36kHz.
   TIMSK |= (1 << TOIE2);                         // 36kHz counter for sleep function.
   
   // Configuration for RS232 communication. 
   UCSRA = 0x00;
   UCSRB = 0x00;   
   UCSRC = 0x86;                                  // No Parity | 1 Stop Bit | 8 Data Bit
   UBRRL = 0xCF;                                  // 2400bps @ 8.00MHz
   
   // I/O Ports.
   DDRB = IRTX | LEFT_DIRECTION | PWM | GREEN_LED; 
   DDRD = RIGHT_DIRECTION | LINE_LED | ODOMETRY_LED | RED_LED;
   
   // Timer 1 for PWM (8-Bit PWM) via OC1A & OC1B
   TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
   TCCR1B = (1 << CS11);                          // TMR1 runs on MCU clock/8
   
   // Timer 0 for reading odometry and line-follow sensors.
   TCCR0 |= START_TMR0;                           // Pre-scalar.
   TIMSK |= (1 << TOIE0);                         // Interrupt timer 0 on
   TCNT0  = COUNTER_START_TMR0;                   // 0xC0 = 2048 interrupts per second.
   
   // A/D Conversion
   ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // clk/64 

   // General configuration.
   BackLEDs(OFF, OFF);
   ODOMETRY_LED_ON;
   LineLED(ON);
   StatusLED(GREEN);
   MotorSpeed(0, 0);
   sei();
}


// ---------------------------------------------------------------------
// Routine for system time in ms.
//
// Call value  : ---
//
// Return value :  : Time in ms from switching on the Asuro robot.
// ---------------------------------------------------------------------
unsigned long SystemTime(void) {
   
   return ((timeBase * 256) + counter36kHz) / 36;
}


// ---------------------------------------------------------------------
// Routine for motor speed
//
// Call value  : -255 t/m +255: Negative is reverse
//                              Positive is forward
//                              Zero is brake.
//
// Return value : ---
// ---------------------------------------------------------------------
void MotorSpeed(int leftSpeed, int rightSpeed) {
    
   // Speed (PWM) of the motors.
   OCR1A = abs(leftSpeed);
   OCR1B = abs(rightSpeed);

   // Direction of rotation of the left motor.
   if (leftSpeed > 0) {
      PORTD = (PORTD &~ ((1 << PD4) | (1 << PD5))) | FORWARD;
   }
   if (leftSpeed < 0) {
      PORTD = (PORTD &~ ((1 << PD4) | (1 << PD5))) | REVERSE;
   }
   if (leftSpeed == 0) {
      PORTD = (PORTD &~ ((1 << PD4) | (1 << PD5))) | BRAKE;
   }

   // Direction of rotation of the right motor.
   if (rightSpeed > 0) {
      PORTB = (PORTB &~ ((1 << PB4) | (1 << PB5))) | FORWARD;
   }
   if (rightSpeed < 0) {
      PORTB = (PORTB &~ ((1 << PB4) | (1 << PB5))) | REVERSE;
   }
   if (rightSpeed == 0) {
      PORTB = (PORTB &~ ((1 << PB4) | (1 << PB5))) | BRAKE;
   }   
}


// ---------------------------------------------------------------------
// Routine for the Status LED.
//
// Call value  : OFF, GREEN, YELLOW of RED
//
// Return value: ---
// ---------------------------------------------------------------------
void StatusLED(unsigned char colour)
{
   if (colour == OFF)    {GREEN_LED_OFF; RED_LED_OFF;}
   if (colour == GREEN)  {GREEN_LED_ON;  RED_LED_OFF;} 
   if (colour == YELLOW) {GREEN_LED_ON;  RED_LED_ON;}
   if (colour == RED)    {GREEN_LED_ON;  RED_LED_ON;}
}


// ---------------------------------------------------------------------
// Routine for Line follow LED.
//
// Call value  : ON or OFF
//
// Return value: ---
// ---------------------------------------------------------------------
void LineLED(unsigned char status) {
   
   PORTD = (PORTD &~(1 << PD6)) | (status << PD6);
}


// ---------------------------------------------------------------------
// Routine for back LED's.
//
// Call value  : ON or OFF (per LED)
//
// Return value: ---
// ---------------------------------------------------------------------
void BackLEDs(unsigned char left, unsigned char right) {
   
   if (left || right) {
      ODOMETRY_LED_OFF;                           // Odometry LED's OFF.
      DDRC  |= (1 << PC0) | (1 << PC1);           // No odometry.
      PORTC |= (1 << PC0) | (1 << PC1); 
   }
   
   if (!left)  PORTC &= ~(1 << PC1);
   if (!right) PORTC &= ~(1 << PC0);
   
   if (!left && !right) ODOMETRY_LED_ON;
}


// ---------------------------------------------------------------------
// Routine for the serial communication, send.
//
// The reading if the odometry and line-follow sensors is switched off.
// Do not move the Asuro robot during the this routine, because the
// odometry is not updated during this routine.
//
// Call value  : Text (ASCII) + length of the text.
//
// Return value: ---
// ---------------------------------------------------------------------
void SerWrite(unsigned char *data, unsigned char length) {
   
   // Interrupt routine for reading the odometry and line-follow sensors OFF.
   SensorsViaInterrupt(OFF);
   
   unsigned char i = 0;
   UCSRB = 0x08;                                  // Start the transmitter.
   while (length > 0) {
      if (UCSRA & 0x20) {                         // Wait for an empty send buffer.
         UDR = data[i++];
         length--;
      }
   }
   while (!(UCSRA & 0x40)); 
   for (i = 0; i < 0xFE; i++)
      for(length = 0; length < 0xFE; length++); 
   
   // Interrupt routine for reading the odometry and line-follow sensors ON.
   SensorsViaInterrupt(ON);
}


// ---------------------------------------------------------------------
// Routine for the serial communication, receive.
//
// The reading if the odometry and line-follow sensors is switched off.
// Do not move the Asuro robot during the this routine, because the
// odometry is not updated during this routine.
//
// Call value  : Text (ASCII) + length of the text + time-out.
//
// Return value :  Text is avalable as pointer.
// ---------------------------------------------------------------------
void SerRead(unsigned char *data, unsigned char length,unsigned int timeout) {
   
   // Interrupt routine for reading the odometry and line-follow sensors OFF.
   SensorsViaInterrupt(OFF);
   
   unsigned char i = 0;
   unsigned int  time = 0;
   UCSRB = 0x10;                                  // Start the receiver.
   // non blocking
   if (timeout != 0) {
      while (i < length && time++ < timeout) {
         if (UCSRA & 0x80) {
            data[i++] = UDR;
            time = 0;
         }
      }
      if (time > timeout) data[0] = 'T';
   }
   // blocking
   else {
      while (i < length) {
         if (UCSRA & 0x80) 
            data[i++] = UDR;
      }
   }
   
   // Interrupt routine for reading the odometry and line-follow sensors ON.
   SensorsViaInterrupt(ON);   
}


// ---------------------------------------------------------------------
// Routine for sending an Integer via the IR communication.
//
// The reading if the odometry and line-follow sensors is switched off.
// Do not move the Asuro robot during the this routine, because the
// odometry is not updated during this routine.
//
// Call value  : Integer
//
// Return value: ---
// ---------------------------------------------------------------------
void PrintInt(int value) { 
   
   // Interrupt routine for reading the odometry and line-follow sensors OFF.
   SensorsViaInterrupt(OFF);
       
   char text[6];
   itoa(value, text, 10);
   SerWrite((unsigned char *)text, strlen(text));
   
   // Interrupt routine for reading the odometry and line-follow sensors ON.
   SensorsViaInterrupt(ON);
}


// ---------------------------------------------------------------------
// Routine for reading the battery value.
//
// The reading if the odometry and line-follow sensors is switched off.
// Do not move the Asuro robot during the this routine, because the
// odometry is not updated during this routine.
//
// Call value  : ---
//
// Return value :  The battery voltage (0 - 1024).
// ---------------------------------------------------------------------
int Battery(void) {
   
   volatile int battery;
   
   // Interrupt routine for reading the odometry and line-follow sensors OFF.
   SensorsViaInterrupt(OFF);
   
   ADMUX = (1 << REFS0) | (1 << REFS1) | BATTERY; // Internal 2.56V reference with external capacitor.
   Sleep(100);                                    // Short pause before start conversion.
   ADCSRA |= (1 << ADSC);                          // Start the conversion.
   while (!(ADCSRA & (1 << ADIF)));               // Wait until the ADC conversion is ready.
   ADCSRA |= (1 << ADIF);                         // Reset ADCIF
   battery = ADCL + (ADCH << 8);
   
   // Interrupt routine for reading the odometry and line-follow sensors ON.
   SensorsViaInterrupt(ON);
   
   return battery;
}


// ---------------------------------------------------------------------
// Routine for reading the switches.
//
// The reading if the odometry and line-follow sensors is switched off.
// Do not move the Asuro robot during the this routine, because the
// odometry is not updated during this routine.
//
// Call value  : ---
//
// Return value :  Switches in bit form bit 0 is switch 0
//                                      bit 1 is switch 1
//                                      etc.
//----------------------------------------------------------------------
unsigned char Switches(void) {

   long switchValue;                              // Measured value of the switches.
   
   // Interrupt routine for reading the odometry and line-follow sensors OFF.
   SensorsViaInterrupt(OFF);
   
   DDRD |= SWITCHES;                              // Switches as outputs.
   SWITCHES_ON;                                   // Analog reading of switches ON.
   
   ADMUX = (1 << REFS0) | COLLISION_SWITCH;       // AVCC referenc with external capacitor.
   Sleep(5);                                      // Short pause before start conversion.
   ADCSRA |= (1 << ADSC);                         // Start the conversion.
   while (!(ADCSRA & (1 << ADIF)));               // Wait until the ADC conversion is ready.
   ADCSRA |= (1 << ADIF);                         // Reset ADCIF
   switchValue = ADCL + (ADCH << 8);              // Measured value of the switches.
   
   SWITCHES_OFF;                                  // Analog reading for switches OFF.
   DDRD &= ~SWITCHES;                             // Switches as inputs.
   Sleep(100);                                    // Pause for stabilisation of the signal before starting the interrupt.
   
   // Interrupt routine for reading the odometry and line-follow sensors ON.
   SensorsViaInterrupt(ON);
   
   //return  ((unsigned char) ((( 1024.0/(float)switchValue - 1.0)) * 61.0 + 0.5));
   
   return ((10240000L/switchValue-10000L)*63L+5000L)/10000;                 // Richard Lagendijk value.
}


// ---------------------------------------------------------------------
// Routine for start/stop switch detection via an external interrupt.
//
// Call value  : ON or OFF
//
// Return value: ---
//----------------------------------------------------------------------
void SwitchDetection(int status) {
   
   if (status == ON) {   
      switchesActive = FALSE;
      SWITCHES_OFF;
      DDRD &= ~SWITCHES;                          // Switches as inputs. Interrupt 1.
      MCUCR &= ~((1 << ISC11) | (1 << ISC10));    // Low level interrupt.
      GICR |= (1 << INT1);                        // Enable external Interrupt 1.
   }
   
   if (status == OFF) {   
      GICR &= ~(1 << INT1);
   }
}


// ---------------------------------------------------------------------
// Routine for a pause in x/36kHz [sec].
//
// Call value  : 0-255
//
// Return value: ---
//----------------------------------------------------------------------
void Sleep(unsigned char time36kHz) {
    
   unsigned char target = (time36kHz + counter36kHz) & 0x00FF;
   while (counter36kHz != target);
}


// ---------------------------------------------------------------------
// Routine for a pause in ms.
//
// Call value  : Integer
//
// Return value: ---
//----------------------------------------------------------------------
void Msleep(int time) {
   
   int z;
   for(z = 0; z < time; z++) Sleep(36);
}


// ---------------------------------------------------------------------
// Routine for start/stop of reading odometry and line-follow sensors
// via interrupt.
//
// Call value  : ON or OFF
//
// Return value: ---
//----------------------------------------------------------------------
void SensorsViaInterrupt(int status) {
   
   if (status == ON) {
      TCCR0 = START_TMR0;                         // Set timer 0 ON.
   }
   
   if (status == OFF) {
      TCCR0 = STOP_TMR0;                          // Set timer 0 OFF.
   }
}  


// ---------------------------------------------------------------------
// Routine for setting the odometry encoders.
//
// Call value  : Integer (per side)
//
// Return value: ---
//----------------------------------------------------------------------
void SetEncoders(int left, int right) {
   
   encoderLeft  = left;
   encoderRight = right;
}


// ---------------------------------------------------------------------
// Routine for driving with the odometry. (Straight line)
//
// Call value  : Distance in mm + Speed
//
// Return value: ---
//----------------------------------------------------------------------
void Drive(int distance, int speed) {
   
   int encoderTarget;
   int leftSpeed  = speed;
   int rightSpeed = speed;
   
   // If the distance is -20 the switch detection is turned off. This makes it possible to driving back after a collision.
   if (distance == -20) {
      SwitchDetection(OFF);  
      switchesActive = FALSE;
   } else {
      // Start the interrupt routine for the switches (emergency-stop).
      SwitchDetection(ON);  
   }

   // Convert the distance to impulses for the odometry.
   encoderTarget = abs((100L * distance) / 270L);      // Value (230L) optimised for the Asuro of Richard Lagendijk.
   
   // Reset both encoders.
   SetEncoders(0, 0);
      
   // Drive to the distance or stop if there is a collision or emergency-stop.
   while (encoderLeft < encoderTarget) {
      
        if (encoderLeft > encoderRight) {         // Left goes faster than right.
         
         leftSpeed  = speed - 40;
         rightSpeed = speed;
      }
      
      if (encoderRight > encoderLeft) {           // Right goes faster than left.
         
         rightSpeed = speed - 30;
         leftSpeed  = speed;
      } 
      
      // Motor speeds.
      // With REVERSE (negative distance) then speed negative.
      if (distance < 0) {
         MotorSpeed(-leftSpeed, -rightSpeed);
      } else {
         MotorSpeed( leftSpeed,  rightSpeed);
      }
      
      // Check for collision or emergency-stop.
      if (switchesActive != 0) {
         EmergencyStop();
      }
      
      Msleep(10);                                 // Small pause for more stable control.
   }
   
   // Set the Asuro robot still and wait a small pause.
   MotorSpeed(0, 0);
   Msleep(200);
}


// ---------------------------------------------------------------------
// Routine for turning with the odometry. (Turn on own axis)
//
// Call value  : Degrees in mm + Speed
//
// Return value: ---
//----------------------------------------------------------------------
void Turn(int degrees, int speed) {

   int encoderTarget;
   int leftSpeed  = speed;
   int rightSpeed = speed;

   // Start the interrupt routine for the switches (emergency-stop).
   SwitchDetection(ON);

   // Convert the degrees to impulses for the odometry.
   encoderTarget = (abs(degrees) * 161L) / 400L; // Value (360L) optimised for the Asuro of Richard Lagendijk.
   
   // Reset both encoders.
   SetEncoders(0, 0);
   
   // Turn to the angle or stop if there is a collision or emergency-stop.
   while (encoderLeft < encoderTarget) {
      
        if (encoderLeft > encoderRight) {         // Left  goes faster than right.
         
         leftSpeed  = speed - 10;                 // Left motor little bit slower. 
         rightSpeed = speed + 10;                 // Right motor little bit faster. 
      }
      
      if (encoderRight > encoderLeft) {           // Right  goes faster than left.

         leftSpeed  = speed + 10;                 // Left motor little bit faster.          
         rightSpeed = speed - 10;                 // Right motor little bit slower.

      }
      
      // Motor speeds.
      // When left turn (negative degrees) then left motor negative, right motor positive.
      if (degrees < 0) {
         MotorSpeed(-leftSpeed,  rightSpeed);
      } else {
         MotorSpeed( leftSpeed, -rightSpeed);
      }
      
      // Check for collision or emergency-stop.
      if (switchesActive != 0) {
         EmergencyStop();
      }
      
      Msleep(10);                                 // Small pause for more stable control.
   }

   // Set the Asuro robot still and wait a small pause. 
   MotorSpeed(0, 0);
   Msleep(200);
}


// ---------------------------------------------------------------------
// Routine for the emergency-stop.
//
// - The Asuro robot waits until the emergency-stop is pressed again.
// - Reset via switches 1-6.
//
// Call value  : ---
//
// Return value: ---
// ---------------------------------------------------------------------
void EmergencyStop(void) {
   
   // Set the Asuro robot still.
   MotorSpeed(0, 0);
   
   // Wait until the emergency-stop is not actiev anymore. (LED lights continuously.)   
   while (Switches() != 0) {
      StatusLED(RED);
   }
   
   // Start the interrupt routine for the switches.
   SwitchDetection(ON);  

   // Wait until the emergency-stop is once more actiev.  = reset. (LED flashes.) 
   while (!switchesActive) {
      StatusLED(RED);
      Msleep(500);
      StatusLED(OFF);
      Msleep(500);
   }
   
   // Wait 0,25 seconds.
   Msleep(250);                                
      
   // Start the interrupt routine for the switches.
   SwitchDetection(ON);  
 
}
