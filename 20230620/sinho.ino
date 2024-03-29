#include <NewPing.h>
#include <Servo.h>

#define SONAR_NUM 3      // 센서의 개수
#define MAX_DISTANCE 300 // 측정 가능한 최대 거리 (cm)

#define MOTOR_DIR 4
#define MOTOR_PWM 5

#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 90
#define LEFT_STEER_ANGLE -10
#define RIGHT_STEER_ANGLE 5

const int encoderPinA = 2;
const int encoderPinB = 3;

int encoderPos = 0;

void doEncoderA() 
{
  // 엔코더 A의 인터럽트 핸들러
  // 엔코더 A의 변경에 따라 엔코더 포지션 값을 증가 또는 감소
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}

void doEncoderB()
{
  // 엔코더 B의 인터럽트 핸들러
  // 엔코더 B의 변경에 따라 엔코더 포지션 값을 증가 또는 감소
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}

int mission_flag = -1;

float UltrasonicSensorData[SONAR_NUM];
NewPing sonar[SONAR_NUM] =
{
  NewPing(52, 53, MAX_DISTANCE),
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(48, 49, MAX_DISTANCE)
};

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void read_ultrasonic_sensor() 
{
  // 초음파 센서로부터 거리 데이터를 읽어옴
  UltrasonicSensorData[0] = sonar[0].ping_cm();
  UltrasonicSensorData[1] = sonar[1].ping_cm();
  UltrasonicSensorData[2] = sonar[2].ping_cm();
}

void send_sonar_sensor_data() 
{
  // 거리 데이터를 시리얼 모니터로 전송
  int i;
  for (i = 0; i < SONAR_NUM; i++) 
  {
    Serial.print(UltrasonicSensorData[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void steering_control(int steer_angle) 
{
  // 스티어링 서보 모터를 조종하여 조향각 설정
  Steeringservo.write(NEURAL_ANGLE + steer_angle);
}

void motor_control(int direction, int speed) 
{
  // DC 모터 제어 함수
  digitalWrite(MOTOR_DIR, direction);
  analogWrite(MOTOR_PWM, speed);
}

void wallfollowing(void) 
{
  // 벽 따라가기 기능
  read_ultrasonic_sensor();

  // 가장 왼쪽에 위치한 센서는 인덱스 0으로 가정
  float left_distance = UltrasonicSensorData[0];

  // 왼쪽 벽으로부터의 원하는 거리 (필요에 따라 조정)
  float desired_distance = 15.0;

  // 원하는 거리와 측정된 거리 간의 차이 계산
  float distance_difference = left_distance - desired_distance;

  // 거리 차이에 따라 스티어 각도 설정
  int steer_angle = 0;  // 스티어 각도 초기화

  if (distance_difference > 0) 
  {
    // 거리 차이가 양수인 경우, 왼쪽으로 스티어
    steer_angle = LEFT_STEER_ANGLE;
  } else if (distance_difference < 0) 
  {
    // 거리 차이가 음수인 경우, 오른쪽으로 스티어
    steer_angle = RIGHT_STEER_ANGLE;
  }

  steering_control(steer_angle);
}

void cross_obs() 
{
  // 교차로 횡단 기능
  read_ultrasonic_sensor();

  int steer_angle = 0;
  bool move_forward = true;

  while (UltrasonicSensorData[1] < 30) 
  {
    motor_control(1, 100);
    steering_control(40);
    steering_control(steer_angle);
    read_ultrasonic_sensor();
  }

  steer_angle = -5;
  steering_control(steer_angle);
  motor_control(1, 150);
}

void setup() 
{
  Serial.begin(115200);

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);

  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);

  motor_control(1, 200);  // 초기 속도를 200으로 설정

  mission_flag = 0;
}

void loop() 
{
  read_ultrasonic_sensor();

  Serial.print("encoderPos : ");
  Serial.println(encoderPos);

  if (mission_flag == 0) 
  {
    wallfollowing();
    if (UltrasonicSensorData[1] <= 80)
    {
      motor_control(1, 0);
      delay(500);
      mission_flag = 1;
      encoderPos = 0;
    }
  }
  if (mission_flag == 1) 
  {
    cross_obs();
    if (encoderPos >= 10000)
    {
      motor_control(1, 0);
    }
  }
}
