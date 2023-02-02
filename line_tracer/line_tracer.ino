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
#define NEURAL_ANGLE 77
#define LEFT_STEER_ANGLE -50
#define RIGHT_STEER_ANGLE 30

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void steering_control(int steer_angle)
{
  if(steer_angle>=RIGHT_STEER_ANGLE) steer_angle = RIGHT_STEER_ANGLE;
  if(steer_angle<=LEFT_STEER_ANGLE) steer_angle = LEFT_STEER_ANGLE;
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
  steer_data = (x_sum/sum) - NPIXELS/2 + 2;
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


void line_tracer()
{ 
  int re;
  re=steering_by_camera();
  motor_control(1, 150);
  
  if(re <= 1 && re >= -1)
  {
    steering_control(0);
  }
  else if(re > 1)
  {
    steering_control(RIGHT_STEER_ANGLE);
  }
  else if(re < -1)
  {
    steering_control(LEFT_STEER_ANGLE);
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

  Serial.begin(115200);
}

void loop() 
{
  steering_by_camera();
  motor_control(1, 150);
  line_adaptation();
  read_line_sensor();
  line_tracer();
  delay(50);
}
