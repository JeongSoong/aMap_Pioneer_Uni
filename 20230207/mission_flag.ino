#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 20000 // Maximum distance (in cm) to ping.

float UltrasonicSensorData[SONAR_NUM];

NewPing sonar[SONAR_NUM] = 
{   // Sensor object array.
  NewPing(48, 49, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(52, 53, MAX_DISTANCE)
};

NewPing R_sensor(52,53,MAX_DISTANCE);
  float R_Sonar_Error = 0.0;
  float R_Sonar_Distance = 0.0;
  float R_Sonar_Distance_old = 0.0;

NewPing L_sensor(48,49,MAX_DISTANCE);
  float L_Sonar_Error = 0.0;
  float L_Sonar_Distance = 0.0;
  float L_Sonar_Distance_old = 0.0;

NewPing F_sensor(50,51,MAX_DISTANCE);
  float F_Sonar_Error = 0.0;
  float F_Sonar_Distance = 0.0;
  float F_Sonar_Distance_old = 0.0;

int mission_flag=-1;

#define AOpin  A0     
#define SIpin  12     
#define CLKpin 13     

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation

#define FASTADC 1
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm();
  UltrasonicSensorData[1] = sonar[1].ping_cm();
  UltrasonicSensorData[2] = sonar[2].ping_cm();
  if(UltrasonicSensorData[0] == 0) UltrasonicSensorData[0] = MAX_DISTANCE *10.0;
  if(UltrasonicSensorData[1] == 0) UltrasonicSensorData[1] = MAX_DISTANCE *10.0;
  if(UltrasonicSensorData[2] == 0) UltrasonicSensorData[2] = MAX_DISTANCE *10.0;
}

void send_sonar_sensor_data(void)
{
  int i;
  for(i=0; i<SONAR_NUM; i++)
  {
    Serial.print(UltrasonicSensorData[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

////////////// DC Motor Control ///////////////
#define MOTOR_DIR 4
#define MOTOR_PWM 5

int Motor_Speed=0;
#define NORMAL_SPEED 100
#define SLOW_SPEED 70

void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, direction);
  analogWrite(MOTOR_PWM, speed);
}
////////////// DC Motor Control ///////////////

////////////// Steering Servo Control ///////////////
#include <Servo.h>
#define RC_SERVO_PIN 9
#define NEURAL_ANGLE 70
#define LEFT_STEER_ANGLE -40
#define RIGHT_STEER_ANGLE 30

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void steering_control(int steer_angle)
{
  if(steer_angle>=RIGHT_STEER_ANGLE + NEURAL_ANGLE) 
  {
    Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  }
  if(steer_angle<=LEFT_STEER_ANGLE + NEURAL_ANGLE) 
  {
    Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  }
  Steeringservo.write(NEURAL_ANGLE + steer_angle);
}
////////////// Steering Servo Control ///////////////

#define threshold_value 60

void threshold(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if((byte)Pixel[i] >= threshold_value)
    {
      LineSensor_Data_Adaption[i] = 255;
    }
    else
    {
      LineSensor_Data_Adaption[i] = 0;
    }
  }
}

int steer_data = 0;
int steering_by_camera(void)
{
  int i;
  long sum = 0;
  long x_sum = 0;

  for(i=0; i<NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += LineSensor_Data_Adaption[i]* i;
  }
  steer_data = (x_sum/sum) - NPIXELS/2 + 1;
  //steering_control(steer_data*1.3);

  Serial.println(steer_data);

  return steer_data;
}

void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }
}
void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);  /* Integration time in microseconds */
  delay(10);              /* Integration time in miliseconds  */

  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);
  
  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }
}


void line_tracing()
{ 
  int re;
  re=steering_by_camera();
  motor_control(1, 150);
  
  if(re == 0)
  {
    steering_control(0);
  }
  else if(re >= -20  && re <= 20)
  {
    steering_control(steer_data*1.4);
  }
  else if(re >= -60 && re <= 60)
  {
    steering_control(steer_data*1.3);
  }
}

void stop_obs()
{
  motor_control(1,0);
}

void wall_following()
{
  motor_control(1, 150);

  if(UltrasonicSensorData[0] + 5 > UltrasonicSensorData[2])
  {
    steering_control(-15); //좌회전
  }
  else if(UltrasonicSensorData[0] < UltrasonicSensorData[2] + 5)
  {
    steering_control(5); //우회전
  }
  else
  {
    steering_control(0);
  }
}

int count=0;
int count1=0;
int count2=0;
void wall_following_CW1()
{
  read_ultrasonic_sensor();
  
  if(UltrasonicSensorData[1]<=80 && count==0)
  {
  motor_control(1, 150);
  Steeringservo.write(100);
  delay(1800);
  mission_flag=2;
  count2=millis();
  }
}

void wall_following_CW2()
{
  read_ultrasonic_sensor();
  
  if(UltrasonicSensorData[1]<=80 && count1==0 && millis()- count2 >6000)
  {
  motor_control(1, 150);
  Steeringservo.write(100);
  delay(1700);
  mission_flag=3;
  }
}


void setup() 
{
  // put your setup code here, to run once:
  int i;
 
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT); 

  mission_flag = 0;

  Serial.begin(115200);
}

void loop() 
{
  line_adaptation();
  read_line_sensor();
  read_ultrasonic_sensor();
  if(mission_flag == 0)
   {
    if(UltrasonicSensorData[1]<=20)
    {
      stop_obs();
      }
      else
      {
        motor_control(1,150);
        threshold();
        line_tracing();
        delay(50);
      }
    if(UltrasonicSensorData[0] + UltrasonicSensorData[2] <= 150)
    {
      mission_flag = 1;
    }
   }

   if(mission_flag == 1)
   {
    wall_following();
    wall_following_CW1();
   }

   if(mission_flag == 2)
   {
    wall_following();
    wall_following_CW2();
   }

   if(mission_flag == 3)
   {
    motor_control(1,150);
    threshold();
    line_tracing();
   }
}
