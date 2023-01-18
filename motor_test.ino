#define MOTOR_DIR 4
#define MOTOR_PWM 5

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
}

void loop() 
{
  Serial.println("Motor Control");
  int i;
  for(i=0; i<255; i++)
  {
    motor_control(1, i);
    delay(10);
  }
}
