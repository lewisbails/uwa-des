/////////////////////////
// Uses NewPing library
// Distances from sensors is averaged over mutiple iterations
// Error is difference between left sensors and desired left distance
// PID for tracking left wall
// turning is angle calculation
// No speed control
////////////////////////


// Libraries
#include <NewPing.h>
#include <stdio.h>
#include <Servo.h>

//min max functions
#define MAX(a,b) ((a)>(b) ? a : b)
#define MIN(a,b) ((a)<(b) ? a : b)


// Servo Init.
Servo motor; //init. servo elements
Servo steer;
#define MOT_PIN 1
#define STR_PIN 2

// Sensors Init.
#define iterations 3
#define SONAR_NUM 5 //number of sensors
#define MAX_DIST 200 //min req distance (cm) in front on car to break from turn cycle
#define LL_TRIG 4
#define LL_ECHO 5
#define L_TRIG 6
#define L_ECHO 7
#define C_TRIG 8
#define C_ECHO 9
#define R_TRIG 10
#define R_ECHO 11
#define RR_TRIG 12
#define RR_ECHO 13
#define LL 0
#define L 1
#define C 2
#define R 3
#define RR 4

// Constants
int warning = 20;
int front_thresh = 50; //distance allowed to wall (cm)
int turn_thresh = 15; //degrees leeway on turn
int ll_des; //desired distance from left wall (cm)
float kp = 0.0005; //proportional gain
float ki = 0.00001; //integral gain
float kd = 0.00001; //derivative gain
int max_servo = 160; //max servo angle (degs)
int min_servo = 50l; //min servo angle (degs)
float max_rads = 55*3.141/180; //max heading angle (rads) 55 degrees left or right
float max_heading = 55; //degs
float P,I,D;

// Measurements - Global for PID function
int ll_dist;
int l_dist;
int c_dist;
int r_dist;
int rr_dist;
int angle_1=0; //heading in radians
int angle=105; //servo angle in degrees
int error_1=0,error_2=0,error_3=0; //

int cm[SONAR_NUM]; //array for ping distances


NewPing sonar[SONAR_NUM]={NewPing(LL_TRIG,LL_ECHO,MAX_DIST),
                          NewPing(L_TRIG,L_ECHO,MAX_DIST),
                          NewPing(C_TRIG,C_ECHO,MAX_DIST),
                          NewPing(R_TRIG,R_ECHO,MAX_DIST),
                          NewPing(RR_TRIG,RR_ECHO,MAX_DIST)};

// Update globals and check we have space around us
void global_update(){
  for (uint8_t i=0;i<SONAR_NUM;i++){
    cm[i] = sonar[i].convert_cm(sonar[i].ping_median(iterations));
  }
  ll_dist = cm[LL];
  l_dist = cm[L];
  c_dist = cm[C];
  r_dist = cm[R];
  rr_dist = cm[RR];
}

void pid(){
    // Pid control formula
    P = kp*(error_1-error_2);
    I = ki*(error_1+error_2)/2;
    D = kd*(error_1-2*error_2+error_3);
    //linear relationships angle/distance for small angles (rads)
    angle_1 = angle_1 + P + I + D;
    // Scale to limits of servo
    angle_1 = MIN(angle_1,max_rads*0.5);//scaled so we can keep the approximation
    angle_1 = MAX(angle_1,-max_rads*0.5);
    angle = (angle_1*180/3.141)*(max_servo-min_servo)/2/max_heading + (max_servo+min_servo)/2; //calculate servo in degs
    // Write to servo
    steer.write(angle);
    // Update old error
    error_3 = error_2;
    error_2 = error_1;
    // Update new distances
    global_update();
    // Update new error
    error_1 = ll_dist - ll_des;
}

// Drive Straight - Drive straight until the front wall is too close
void straight_function(){
  angle = (max_servo+min_servo)/2;
  steer.write(angle);
  global_update();
  error_1=ll_dist-ll_des;
  error_2=0;
  error_3=0;
  
  while(c_dist>front_thresh || c_dist==0){
    pid();
  }
}
    
// Turn - turn until we're parallel with the wall again
void turn_function(){
  angle = 45*(max_servo-min_servo)/2/max_heading + (max_servo+min_servo)/2; //convert heading to servo value
  steer.write(angle);
  global_update();

  // Using trig to work out when we are parallel to the wall
  while (!(l_dist*0.7071>(ll_dist-turn_thresh) && l_dist*0.7071<(ll_dist+turn_thresh) && (c_dist>(front_thresh*2)||c_dist==0))){ //turn until parallel with left wall
    global_update();  
  }
  
}

void setup() {
  Serial.begin(9600);
  pinMode(MOT_PIN, OUTPUT); //motor
  pinMode(STR_PIN, OUTPUT); //steer
  motor.attach(MOT_PIN); //output pins attached to servo 'motors'
  steer.attach(STR_PIN); //output pins attached to servo 'steer'
  global_update();
  ll_des = ll_dist;
  motor.write(50);
}

void loop() {
  straight_function();
  turn_function();
}
  


