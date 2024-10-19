// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

# define EMIT_PIN    11    // Documentation says 11.
# define LS_LEFT_PIN 12   // Complete for DN1 pin
# define LS_MIDLEFT_PIN A0   // Complete for DN2 pin
# define LS_MIDDLE_PIN A2   // Complete for DN3 pin
# define LS_MIDRIGHT_PIN A3   // Complete for DN4 pin
# define LS_RIGHT_PIN A4   // Complete for DN5 pin
# define DN6 4
# define DN7 5

// Store our pin numbers into an array, which means
// we can conveniently select one later.
// ls(line sensor)_pin
int lpin[7] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN,
                  DN6,
                  DN7 
                  };
// Class to operate the linesensor(s).
class LineSensor_c {
  public:
    
    // Constructor, must exist.
    LineSensor_c() {

    } 

  void init(){

    // Set some initial pin modes and states
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( 12, INPUT );     // Set line sensor pin to input
  pinMode( A0, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A4, INPUT );
  pinMode( DN6, INPUT );
  pinMode( DN7, INPUT );

  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");



   
  }

  int ReadSensors(int n){
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, LOW );
      pinMode( lpin[n], OUTPUT );
      digitalWrite( lpin[n], HIGH );
      delayMicroseconds( 10 );

     pinMode( lpin[n], INPUT );
     unsigned long start_time = micros();
      while( digitalRead( lpin[n] ) == HIGH ) {
      // Do nothing here (waiting).
     }
     unsigned long end_time = micros();
      pinMode( EMIT_PIN, INPUT );
      unsigned long DN = end_time - start_time;

      return (float)DN;

    }
};



#endif
