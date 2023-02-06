#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

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

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm();
  UltrasonicSensorData[1] = sonar[1].ping_cm();
  UltrasonicSensorData[2] = sonar[2].ping_cm();
  if(R_Sonar_Distance == 0) R_Sonar_Distance = MAX_DISTANCE *10.0;
  if(L_Sonar_Distance == 0) L_Sonar_Distance = MAX_DISTANCE *10.0;
  if(F_Sonar_Distance == 0) F_Sonar_Distance = MAX_DISTANCE *10.0;
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

void setup()
{
  Serial.begin(115200);
  // trig를 출력모드로 설정, echo를 입력모드로 설정
}

void loop() 
{
  read_ultrasonic_sensor();
  send_sonar_sensor_data();
}
