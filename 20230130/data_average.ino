#include <NewPing.h>
#define MAX_DISTANCE 300

float old_average=0;
float new_average=0;

#define MOTOR_DIR 4
#define MOTOR_PWM 5

#include <Servo.h>
#define RC_SERVO_PIN 9
#define NEURAL_ANGLE 120
#define LEFT_STEER_ANGLE -60
#define RIGHT_STEER_ANGLE 30

NewPing R_sensor(52, 53,MAX_DISTANCE);
float R_Sonar_Distance = 0.0;
float R_Sonar_Distance_old = 0.0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
float R_Sonar_Error = 0.0;

NewPing L_sensor(48, 49,MAX_DISTANCE);
float L_Sonar_Distance = 0.0;
float L_Sonar_Distance_old = 0.0;
float L_Sonar_Error = 0.0;

NewPing F_sensor(50,51,MAX_DISTANCE);
float F_Sonar_Distance = 0.0;
float F_Sonar_Distance_old = 0.0;
float F_Sonar_Error = 0.0;

void read_sonar_sensor(void)
{ //초음파센서 측정
  R_Sonar_Distance = R_sensor.ping_cm()*10.0;
  L_Sonar_Distance = L_sensor.ping_cm()*10.0;
  F_Sonar_Distance = F_sensor.ping_cm()*10.0;
  if(R_Sonar_Distance == 0){R_Sonar_Distance = MAX_DISTANCE * 10.0;}
  if(L_Sonar_Distance == 0){L_Sonar_Distance = MAX_DISTANCE * 10.0;}
  if(F_Sonar_Distance == 0){F_Sonar_Distance = MAX_DISTANCE * 10.0;}
}

void data_average (float a,int K)
{
  new_average = (old_average*(K-1)/K) + (a/K);
  old_average=new_average;
}

void setup()
{
  Serial.begin(115200);
}

float b=0;
int K=1;

void loop() 
{
  read_sonar_sensor();
  data_average (F_Sonar_Distance,K);
  K = K+1;
  Serial.println(new_average);
}
