#define MOTOR_DIR 4
#define MOTOR_PWM 5
////////////// Steering Servo Control ///////////////
#include <Servo.h>
#define RC_SERVO_PIN 9
#define NEURAL_ANGLE 120
#define LEFT_STEER_ANGLE -60
#define RIGHT_STEER_ANGLE 30

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;
////////////// Steering Servo Control ///////////////

void steering_control(int steer_angle)
{
  Steeringservo.write(NEURAL_ANGLE + steer_angle);
}

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, direction);
  analogWrite(MOTOR_PWM, speed);
}

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  motor_control(1, 150);
}

void loop() 
{
  Serial.println("Motor Control");
  
  steering_control(LEFT_STEER_ANGLE);
  delay(500);
  steering_control(RIGHT_STEER_ANGLE);
  delay(500);
}
