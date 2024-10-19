 // this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15
# define EMIT_PIN 11
# define LS_LEFT_PIN 12
# define LS_CENTER_LEFT_PIN A0
# define LS_CENTER_PIN A2
# define LS_CENTER_RIGHT_PIN A3
# define LS_RIGHT_PIN A4

 
#define FWD LOW
#define REV HIGH

#define MAX_PWM 250.0

// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    
    void initialise() {
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    }
    
    void setMotorPower( float left_pwm, float right_pwm ) {
    analogWrite( L_PWM_PIN, left_pwm );
    analogWrite( R_PWM_PIN, right_pwm );
    }

    void setMotorDir( float left_dir, float right_dir ) {
    digitalWrite( L_DIR_PIN, left_dir );
    digitalWrite( R_DIR_PIN, right_dir );
    }
    // Write a function to operate
    // your motor(s)
    // ...

    void move_forward() {
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_DIR_PIN, LOW);
    
    //analogWrite( L_PWM_PIN, 20 );
    //analogWrite( R_PWM_PIN, 20 );

    while(1) {
    Serial.println("Program Halted");
    delay(8000);
    analogWrite( L_PWM_PIN, 0 );
    analogWrite( R_PWM_PIN, 0 );
  }
    }

   void line_sensor(){
   pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
   pinMode( LS_LEFT_PIN, INPUT ); 

   pinMode( EMIT_PIN, OUTPUT );
   digitalWrite( EMIT_PIN, HIGH );


   pinMode( LS_LEFT_PIN, OUTPUT );
   digitalWrite( LS_LEFT_PIN, HIGH );
   delayMicroseconds( 10 );


   unsigned long start_time = micros();

   while( digitalRead( LS_LEFT_PIN ) == HIGH ) {
      // Do nothing here (waiting).
   }

   unsigned long end_time = 1000 ;

   pinMode( LS_LEFT_PIN, INPUT );

   unsigned long elapsed_time = end_time - start_time;

   Serial.println( elapsed_time );

   }
    
};



#endif
