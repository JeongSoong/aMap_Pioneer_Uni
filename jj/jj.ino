#include <NewPing.h>
#include <Servo.h>
#include <MsTimer2.h>

#define encodPinA1 2
#define encodPinB1 3
#define MOTOR_DIR 4
#define MOTOR_PWM 5
#define A0pin A0
#define SIpin 12
#define CLKpin 13
#define RC_SERVO_PIN 9
#define MaxDistance  300
#define LEFT_STEER_ANGLE  -40  // 실험으로 구할것-45
#define NEURAL_ANGLE 77
#define RIGHT_STEER_ANGLE  20  // 실험으로 구할35
#define NPIXELS 128

long timer = 0;
long what=0;
int A=0;
byte Pixel[NPIXELS];
int AAA=0;
int D=0;
int misson_flag=0;
int LineSenSor_Data[NPIXELS];
int LineSenSor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int DDD=0;
volatile long encoderPos = 0;
int encoder_error=0;
int count=130;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

NewPing R_sensor(48,49,MaxDistance);
  float R_Sonar_Error = 0.0;
  float R_Sonar_distance = 0.0;
  float R_Sonar_distance_old = 0.0;

NewPing L_sensor(52,53,MaxDistance);
  float L_Sonar_Error = 0.0;
  float L_Sonar_distance = 0.0;
  float L_Sonar_distance_old = 0.0;

NewPing F_sensor(50,51,MaxDistance);
  float F_Sonar_Error = 0.0;
  float F_Sonar_distance = 0.0;
  float F_Sonar_distance_old = 0.0;

Servo Steeringservo;

int Steering_Angle = NEURAL_ANGLE;


int swich = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE - D)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE - D;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE + D)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE + D;
  Steeringservo.write(Steering_Angle+7);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float sonar_data_F[3]={0};
float sonar_data_R[3]={0};
float sonar_data_L[3]={0};

void read_sonar_sensor(void){ //초음파센서 측정
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    L_Sonar_distance = L_sensor.ping_cm()*10.0;
    F_Sonar_distance = F_sensor.ping_cm()*10.0;
    if(R_Sonar_distance == 0){R_Sonar_distance = MaxDistance * 10.0;}
    if(L_Sonar_distance == 0){L_Sonar_distance = MaxDistance * 10.0;}
    if(F_Sonar_distance == 0){F_Sonar_distance = MaxDistance * 10.0;}
  }
void update_sonar_old(void){ //초음파 센서의 옛날값 저장
   R_Sonar_distance_old = R_Sonar_distance;
   L_Sonar_distance_old = L_Sonar_distance;
   F_Sonar_distance_old = F_Sonar_distance;
  }
void update_sonar_error(void){ //초음파 센서의 옛날값과 현재값의 차이 저장
   R_Sonar_Error = R_Sonar_distance - R_Sonar_distance_old;
   L_Sonar_Error = L_Sonar_distance - L_Sonar_distance_old;
   F_Sonar_Error = F_Sonar_distance - F_Sonar_distance_old;
   }
  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int timer1=0;
void setup() {
  for (int i=0; i<3; i++){
    read_sonar_sensor();
    }
  timer1=millis();
  for (int a=0; a<NPIXELS; a++){
   LineSenSor_Data[a] = 0;
   LineSenSor_Data_Adaption[a] = 0;
   MAX_LineSensor_Data[a] = 1023;
   MIN_LineSensor_Data[a] = 0;
   }
  // put your setup code here, to run once:
//  Serial.begin(115200);
  pinMode(40,OUTPUT);
  pinMode(SIpin,OUTPUT);
  pinMode(CLKpin,OUTPUT);
  digitalWrite(CLKpin,LOW);
  digitalWrite(SIpin,LOW);
  
#if FASTADC
   sbi(ADCSRA, ADPS2);
   cbi(ADCSRA, ADPS1);
   cbi(ADCSRA, ADPS0);
#endif
  
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  
  misson_flag=0;
  MsTimer2::set(100, interrupt_setup);
  MsTimer2::start();
  Serial.begin(115200);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP); 
  pinMode(encodPinB1, INPUT_PULLUP);  
  attachInterrupt(0, encoderB, RISING);
}

void encoderB()  
{
  delayMicroseconds(3);
  if(digitalRead(encodPinB1)==LOW)    encoderPos++;           
  else                                 encoderPos--; 
}

void encoder_Serial(){
  interrupt_setup();

 // Serial.print("encoderPos = ");
 // Serial.println(encoderPos);
  encoder_error=encoderPos;
//  delay(50);
  }


void moter_speed(){
  encoder_Serial();
  if(encoder_error<A){
    count++;
    }
  else if(encoder_error>A){
    count--;
    }
  if(count<=90){
    count=90;
    }
  if(count>=240){
    count=240;
    }
 // Serial.print("moter speed = ");
 // Serial.println(count);
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void line_adaptation(void){
  int i;
  for(i=0; i<NPIXELS; i++){
    if (LineSenSor_Data[i] >= MAX_LineSensor_Data[i]) MAX_LineSensor_Data[i]=LineSenSor_Data[i];
    if (LineSenSor_Data[i] <= MIN_LineSensor_Data[i]) MIN_LineSensor_Data[i]=LineSenSor_Data[i];
    }
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void read_line_sensor(void){
  int i;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin,LOW);
  digitalWrite (SIpin,HIGH);
  digitalWrite (CLKpin,HIGH);
  digitalWrite (SIpin,LOW);

  delayMicroseconds (1);

  for(i=0; i<NPIXELS; i++){
    Pixel[i]=analogRead (A0pin);
    digitalWrite (CLKpin,LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin,HIGH);
    }
  for(i=0; i<NPIXELS; i++){
    LineSenSor_Data_Adaption[i]=map(Pixel[i],MIN_LineSensor_Data[i],MAX_LineSensor_Data[i],0,256);
  //  Serial.print(LineSenSor_Data_Adaption[i]);
   //// Serial.print(" ");
    }
  //  Serial.print(LineSenSor_Data_Adaption[i]);
   //   Serial.println("  ");
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motor_control(int direction, int speed){
  digitalWrite(MOTOR_DIR,direction);
  analogWrite(MOTOR_PWM,speed);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int miro1_count=0;
int is=0;
void mode_miro1(void){
  read_sonar_sensor();
  if( F_Sonar_distance <= 1250 && F_Sonar_distance >= 1150){is++;}
  else{is=0;}
  if(miro1_count==0 && is>=3){
    digitalWrite(40,1);
    A=40;
    D=10;
    encoderPos=0;
    while(encoderPos<=1150){
    encoder_Serial();
    motor_control(1,160);
    Steering_Angle=120;
    steering_control();
    misson_flag=2;
    miro1_count++;
    }
    what=millis();
      }
   }
  

void mode_miro2(void){
  read_sonar_sensor();
  if(R_Sonar_distance+L_Sonar_distance<1150 && R_Sonar_distance+100 >= L_Sonar_distance && R_Sonar_distance-100 <= L_Sonar_distance){                                   Steering_Angle=93;      }
  else if(R_Sonar_distance>L_Sonar_distance && R_Sonar_distance <= 2000){       Steering_Angle=105;     }
  else if(R_Sonar_distance<L_Sonar_distance){                                   Steering_Angle=80;      } 
  else{                                                                         Steering_Angle=93;      }
  steering_control();
//  moter_speed();
  motor_control(1,200);
 // delay(10);
  }

int test_miro2=0;

void mode_miro3(void){
  
  read_sonar_sensor();
  if(F_Sonar_distance <= 1150 && F_Sonar_distance >= 1000){test_miro2++;}
  else{test_miro2=0;}
  if((test_miro2>=2 || R_Sonar_distance-L_Sonar_distance >= 2000) && millis()-what >= 3000){
    digitalWrite(40,1);
    A=40;
    D=10;
    encoderPos=0;
    while(encoderPos<=1100){
    encoder_Serial();
    motor_control(1,160);
    Steering_Angle=120;
    steering_control();
    }
     misson_flag=4;
   }
  }




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define threshold_value 60
void threshold(void){
  int i;
  
  for(i=0; i<NPIXELS; i++){
    if(LineSenSor_Data_Adaption[i]>=threshold_value){
      LineSenSor_Data_Adaption[i]=255;
      }
     else{
      LineSenSor_Data_Adaption[i]=0;
      }
   //   Serial.print(LineSenSor_Data_Adaption[i]);
   //  Serial.print(" ");
    }
  //  Serial.println("  ");
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int steer_data = 0;

void steering_by_camera(void){
  int i;
  long sum=0;
  long x_sum=0;
  steer_data = 0;
  for(i=15; i<NPIXELS; i++){
    //Serial.print(LineSenSor_Data_Adaption[i]);
    //Serial.print(" ");
    sum += LineSenSor_Data_Adaption[i];
    x_sum += (LineSenSor_Data_Adaption[i])*i;
    //Serial.print(LineSenSor_Data_Adaption[i]);
    //Serial.print(" ");
    }
     //Serial.println("  ");
   //AAA=sum;
   steer_data = (x_sum/sum) - NPIXELS/2;
 //  Serial.println("  ");
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mode_line(){
  //if(millis() - DDD >= 12000 || millis() - DDD<= 7000){motor_control(1,110);}
  //else{motor_control(1,130);}
  if (steer_data == 0){}
  
  else if(steer_data >= -15 && steer_data <= 15){
      Steering_Angle = NEURAL_ANGLE + (steer_data);
    //  motor_control(1,130);
   // delay(10);
    //delay(100);
   }
   else if(steer_data >= -60 && steer_data <= 60){
      Steering_Angle = NEURAL_ANGLE + (steer_data)*1.4;
   //   motor_control(1,140);
   //   delay(20);
   // delay(100);
   }
   steering_control();
  }

void mode_line2(){
  if(steer_data ==0) {
      motor_control(1,130);
      Steering_Angle = 130;
      steering_control();
      delay(50);
    }
  if(steer_data >= -15 && steer_data <= 15){
      Steering_Angle = NEURAL_ANGLE + (steer_data)*1.2;
    //  motor_control(1,130);
    //  delay(10);
    // delay(100);
   }
   else if(steer_data >= -60 && steer_data <= 100){
      Steering_Angle = NEURAL_ANGLE + (steer_data)*1.3;
   //   motor_control(1,140);
   //   delay(20);
   // delay(100);
   }
   steering_control();
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  int test=0;
  int test4=0;
  int sisi=0;
void loop() 
{
  motor_control(1,120);
      steering_by_camera();
      mode_line();
}
