#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "encoders.h"
#include "PololuBuzzer.h"
#include "timer3.h"

volatile float l_speed_t3;
volatile float r_speed_t3;

#define LED_PIN 13
#define left_power_pin 10
#define right_power_pin 9
#define left_direction_pin 16
#define right_direction_pin 15// Pin to activate the orange LED

PID_c spd_pid_left; // PID for Left or Right Wheel
PID_c spd_pid_right;

unsigned long update_ts; //timestamp

float ave_e1_spd; // low pass filter of speed

long count_e1_last; // for difference in encoder counts

boolean led_state;
// Variable to "remember" the state
// of the LED, and toggle it.
LineSensor_c line_sensors;
Motors_c motors;
Encoders_c encoders;
Kinematics_c kinematics;
float e0;
float e1;
float initl, initr = 0;
int a;
int count;
int l=47;
float dl, dr;
int le0,le1;
float tetha, last_tetha, angle, ia, ra, p;
float xr;
float x1,y1,cc;
int edge;
int first_e, second_e, third_e, fourth_e, fifth_e;
float first_a, second_a, third_a, fourth_a, fifth_a;
int edge_det;
PololuBuzzer buzzer;


void uturn(){
   motors.setMotorDir(FWD, REV);
   motors.setMotorPower(30, 30);
}
void movefwd(){
   motors.setMotorDir(FWD, FWD);
   motors.setMotorPower(20, 20);
}
void idle(){
   motors.setMotorDir(REV, FWD);
   motors.setMotorPower(0, 0);
}

void delaymillis(unsigned long x)
{
  unsigned long presentMillis = millis();
  while(millis() - presentMillis <= x)
  {
    
  }
}

char state;
int line;
int colour = 1000;

// put your setup code here, to run once:
void setup() {

  line_sensors.init();
  motors.initialise();
  encoders.setupEncoder0();
  encoders.setupEncoder1();

  //Setup kp, ki, kd.
  spd_pid_left.init(110.0,1.1,-200.0);
  spd_pid_right.init(110.0,1.1,-200.0);

  // Just to compare the speed measurement
  //setupTimer3();

  Serial.begin(9600);
  delay(1000);
  Serial.println("*RESET*");

  update_ts = millis();

  // Set initial value for our last e1 count within loop
  count_e1_last = count_e1;

  motors.setMotorPower(80, 0);

  //ave_e1_last = 0.0;

  //Reset PID before we use it
  spd_pid_left.reset();

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );

  // Set initial state of the LED
  led_state = false;
  line = 0;
  state = "ignore";
  a= 1;
  count=0;
  le0=0;
  le1=0;
  tetha=0;
  x1=0;
  y1=0;
  angle=0;
  edge=0;
  first_a=0, 
  second_a=0,
  third_a=0,
  fourth_a=0,
  fifth_a=0;

}


// put your main code here, to run repeatedly:
void loop() {

  count_e0 = encoders.encodl();
  count_e1 = encoders.encodr();

  //Serial.println(encoders.encodl());
  //Serial.println(encoders.encodr());

  unsigned long elapsed; //Calculate difference in time

  //Last time, Current time
  elapsed = millis() - update_ts;

  // Calculate speed estimate (For now just for the left motor)
  if (elapsed > 20) { // every 20ms
    update_ts = millis();

    long diff_e1; // difference in encoder count
    float e1_speed; // Speed calculated for e1

    // Calculate differences in e1
    diff_e1 = count_e0 - count_e1_last; // Difference in encoder count
    count_e1_last = count_e1; // Speed calculated for e1

    // Encoder counts per ms
    e1_speed = (float)diff_e1;
    e1_speed /= (float)elapsed; // Actual elapsed ms

    ave_e1_spd = (ave_e1_spd * 0.7) + (e1_speed* 0.3);

    // Encoder counts per ms
    //Serial.print(e1_speed*100);
    //Serial.print(",");
    //Serial.print(l_speed_t3*100);
    //Serial.print(",");
    //Serial.print(ave_e1_spd*100);
    //Serial.print("\n");

    float demand;
    float pwm;
// analogWRITE( L_PWM, 1.8);
    demand = 1.8; //1.8 encoder counts per ms
//    pwm(1.8) = spd_pid_left.update(ave_e1_speed, demand); // (measurement, demand)

    // P - term = 
    // I - term
    // D - term
    motors.setMotorPower(30, 0);

    //Serial.print(l_speed_t3);
    //Serial.print(",");
    //Serial.print(spd_pid_left.last_error);
    //Serial.print("");
    //Serial.println(demand);

  }// end of millis()block
  motors.setMotorPower(20, 20);

  //if(millis() - pid_test_ts > 1000){
   // pid_test_ts = millis();
   //demand = demand * - 1;
 // }
  //Line Sensor
  digitalWrite(LED_PIN, led_state);

  //Serial.println("loop");
  //Serial.println(encoders.left_enc());
  //Serial.println(encoders.right_enc());
  int DN[7];

  for ( int i = 0; i < 7; i++ ) {

    // This line calls a function within your class
    // and stores the returned value.
    DN[i] = line_sensors.ReadSensors( i );

    //Serial.print( reading[i] );
    //Serial.print(", ");
  }
  dl = encoders.encodl() - le0; //delta left
  dr = encoders.encodr() - le1; //delta right

  le0 = encoders.encodl(); 
  le1 = encoders.encodr();

  xr = ((dl*0.28)+(dr*0.28))/2.0; //X(r)
  tetha = (((dl*0.28)-(dr*0.28)))/(2.0*17); //Theta(r)
  x1 = x1+ (xr*cos(angle)); //X1
  y1 = y1+ (xr*sin(angle)); //Y1s
  angle = angle + tetha;
  Serial.print(x1);
  Serial.print(",");
  Serial.print(y1);
  Serial.print("//");
  Serial.print(angle);
  Serial.println(",");
  last_tetha =  tetha;
  Serial.println(count);
   Serial.println(edge);
  /*
  if(DN[5]>750 && DN[6]>950){
    uturn();
    delay(485);
  }
  else if(DN[5]<750 && DN[6]<950){
  movefwd();
  } */

  if(DN[5]>750 && DN[6]>950 && count<=1){
    count = count+1;
    uturn();
    delaymillis(485);
  }
  else if(count == 0 || count == 1){
    movefwd();
    delaymillis(200);
  }
  else if(count == 2){
    dl = 0;
    dr = 0;
    le0 = 0;
    le1 = 0;
    xr = 0;
    tetha = 0;
    x1 = 0;
    y1 = 0;
    angle = 0;
    last_tetha = 0;
    count_e0 = 0;
    count_e1 = 0;
    edge = edge+1;
    count = 3;
  }
  else if(count == 3){
    first_e = first_e+dl;
    movefwd();
    
    if(DN[5]>750 && DN[6]>950){
      edge_det = 1;
      
    }

    if(edge_det==1){
      if(angle<-3.4){
      count = 10;
    }
    else{
      
      uturn();
      first_a = first_a + tetha;
      if(DN[5]<710){
        count = 4;
        edge_det = 0;
        edge = edge + 1;
        delaymillis(100);
      }}
    }
    
  }
  else if(count == 4){
    second_e = second_e+dl;
    movefwd();
    
    if(DN[5]>750 && DN[6]>950){
      edge_det = 1;
      
    }

    if(edge_det==1){
      if(angle<-3.4){
      count = 10;
    }
    else{
      
      uturn();
      second_a = second_a + tetha;
      if(DN[5]<710){
        count = 5;
        edge_det = 0;
        edge = edge + 1;
        delaymillis(100);
      }}
    }
  }
  else if(count == 5){
    third_e = third_e+dl;
    movefwd();

    if(DN[5]>750 && DN[6]>950){
      edge_det = 1;
      
    }

    if(edge_det==1){
      if(angle<-3.4){
      count = 10;
    }
    else{
      
      uturn();
      third_a = third_a + tetha;
      if(DN[5]<710){
        count = 6;
        edge_det = 0;
        edge = edge + 1;
        if(first_a > 85 && second_a > 85 && third_a > 85){
          count=10;
        }
        else if(first_a > 100 && second_a > 100 && third_a > 100){
          count=10;
        }
        delaymillis(100);
      }}
    }
  }
  else if(count == 6){
    fourth_e = fourth_e+dl;
    movefwd();
 
    if(DN[5]>750 && DN[6]>950){
      edge_det = 1;
    }

    if(edge_det==1){
      if(angle<-3.4){
      count = 10;
    }
    else{
      
      uturn();
      fourth_a = fourth_a + tetha;
      if(DN[5]<710){
        count = 7;
        edge_det = 0;
        edge = edge + 1;
        delaymillis(100);
      }}
    }
  }
  else if(count == 7){
    fifth_e = fifth_e+dl;
    movefwd();

    if(DN[5]>750 && DN[6]>950){
      edge_det = 1;
      
    }

    if(edge_det==1){
      if(angle<-3.4){
      count = 10;
    }
    else{
      
      uturn();
      fifth_a = fifth_a + tetha;
      if(DN[5]<710){
        count = 5;
        edge_det = 0;
        edge = edge + 1;
        delaymillis(100);
      }}
    }
    }
  
  else if(count == 10){
    motors.setMotorDir(REV, FWD);
    motors.setMotorPower(0, 0);

    if(edge == 5){
      Serial.println("Rectangle");
      Serial.print("First Edge:");
     Serial.println(first_e);
     Serial.print("Second Edge:");
     Serial.println(second_e);
     Serial.println(first_a);
     Serial.println(second_a);
     Serial.println(third_a);
     Serial.println(fourth_a);
    }
    else if(edge == 4){
      Serial.println("Triangle");
       Serial.print("First Edge:");
      Serial.println(first_e);
      Serial.print("Second Edge:");
      Serial.println(second_e);
      Serial.print("Third Edge:");
      Serial.println(third_e);
      Serial.println(first_a);
      Serial.println(second_a);
      Serial.println(third_a);
    }
  }

  //Serial.println(""); // to create a new line


  // Using an if statement to toggle a variable
  // with each call of loop()
  if ( led_state == true ) {
    led_state = false;
  } else {
    led_state = true;
  }


}


// We use the variable to set the
// debug led on or off on the 3Pi+
