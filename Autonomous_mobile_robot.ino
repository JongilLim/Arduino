#include <SoftwareSerial.h>
#include <Encoder.h>
#include <NewPing.h> 
#include <Servo.h>

//sonar sensors
#define trigger1 31
#define echo1 30

#define trigger2 33
#define echo2 32

#define trigger3 35
#define echo3 34

#define trigger4 37
#define echo4 36

#define MAX_DISTANCE 200

NewPing sensor1(trigger1, echo1, MAX_DISTANCE);
NewPing sensor2(trigger2, echo2, MAX_DISTANCE);
NewPing sensor3(trigger3, echo3, MAX_DISTANCE);
NewPing sensor4(trigger4, echo4, MAX_DISTANCE);

int sonar_measure1[3];
int sonar_measure2[3];
int sonar_measure3[3];
int sonar_measure4[3];
String sonar_measure_string;

//servo motor
Servo servo;
int pos = 0;

//bluetooth
int bluetoothTx = 50;
int bluetoothRx = 51;
char toSend;

//wheel motor
char movement[5];
int k=0;
float dir_and_dis[4];
int n=0;
int distance_L, distance_R;
int SPEED_LEFT = 0;
int SPEED_RIGHT = 0;
int high = HIGH;
int low = LOW;
int DIR_L, DIR_R, DIR_LL, DIR_RR;

//check_obstacle value
char rotation[2];
int degree_of_rotation = 0;
int l=0;
int m=0;
int num_and_degree[2];
int obstacle_distance = 0;

//wheel motor
const int LEFT_MOTOR_DIR_PIN = 4;
const int LEFT_MOTOR_PWM_PIN = 5;
const int RIGHT_MOTOR_DIR_PIN = 7;
const int RIGHT_MOTOR_PWM_PIN = 6;
const int LEFT_MOTOR_DIR = 10;
const int LEFT_MOTOR_PWM = 11;
const int RIGHT_MOTOR_DIR = 8;
const int RIGHT_MOTOR_PWM = 9;
int SPEED_L = 150;
int SPEED_R = 150;

//encoder
Encoder ENCODER_LEFT(2, 3);
Encoder ENCODER_RIGHT(20, 21);

//bluetooth
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup() 
{ 
  pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT );
  pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT );
  pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT );
  pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT );
  pinMode( LEFT_MOTOR_DIR, OUTPUT );
  pinMode( LEFT_MOTOR_PWM, OUTPUT );
  pinMode( RIGHT_MOTOR_DIR, OUTPUT );
  pinMode( RIGHT_MOTOR_PWM, OUTPUT );
  Serial.begin(9600);
  bluetooth.begin(9600);
  servo.attach(28);
}

void loop()
{
  digitalWrite( LEFT_MOTOR_DIR_PIN, HIGH );
  analogWrite( LEFT_MOTOR_PWM_PIN, 0 );
  digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH );
  analogWrite( RIGHT_MOTOR_PWM_PIN, 0 );
  digitalWrite( LEFT_MOTOR_DIR, HIGH );
  analogWrite( LEFT_MOTOR_PWM, 0 );
  digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH );
  analogWrite( RIGHT_MOTOR_PWM, 0 );
  
  if(bluetooth.available())
  {
    char toSend = (char)bluetooth.read();
    switch (toSend){
      case 'A':
        //measure distance
        Measurement();
        break;  
      case 'B':
        //move the robot
        Move();
        break; 
       case 'D':
        //rotate the robot
        Rotation();
        break;
       case 'F':
        //check obstacle
        Check_obstacle();
        break;
    }
  }
  
}

void Measurement()
{
  int i = 0;
  
  servo.write(0);
  delay(15);
  
  //measure surrounding distance      
  for(pos = 90; pos < 171; pos += 30)
  { 
    servo.write(pos);
    delay(300);
    sonar_measure1[i] = sensor1.ping_cm();
    delay(20);
    sonar_measure2[i] = sensor2.ping_cm();
    delay(20);
    sonar_measure3[i] = sensor3.ping_cm();
    delay(20);
    sonar_measure4[i] = sensor4.ping_cm();
    delay(20);
    i++;
  }
        
  int j = 0;
  
  //convert int to string and combine all stirngs
  for(j=0 ; j<3 ; j++)
  {
    sonar_measure_string =  sonar_measure_string + sonar_measure1[j] + ' ';
  }
  for(j=0 ; j<3 ; j++)
  {
    sonar_measure_string =  sonar_measure_string + sonar_measure2[j] + ' ';
  }
  for(j=0 ; j<3 ; j++)
  {
    sonar_measure_string =  sonar_measure_string + sonar_measure3[j] + ' ';
  }
  for(j=0 ; j<3 ; j++)
  {
    sonar_measure_string =  sonar_measure_string + sonar_measure4[j] + ' ';
  }
  
  //send string to smartphone
  bluetooth.println(sonar_measure_string);
  sonar_measure_string = 0;
  servo.write(90);
  delay(10);
}

void Move()
{
  long count_left_encoder = 0;  //count left encoder
  long count_right_encoder = 0; //count right encoder

  delay(100);
  
  //read movement values from smartphone
  do
  {
    if(bluetooth.available())
    {
      toSend = (char)bluetooth.read();
      movement[k++]=toSend;
      if(toSend == ' ')
      {
        dir_and_dis[n] = atof(movement); 
        k=0;
        n++;
      }
    }
  }while(toSend != '\n');
  n=0;

  //calculating distance L and R
  distance_L = 80 * dir_and_dis[1] * dir_and_dis[0];
  distance_R = 324 * dir_and_dis[3] * dir_and_dis[2];
  
  //determine direction of left wheel
  if( dir_and_dis[0] * 1 > 0 )
  {
     DIR_L = high;
     DIR_LL = low;
   }
  else if( dir_and_dis[0] * 1 < 0 )
  {
    DIR_L = low;
    DIR_LL = high;
    distance_L *= 1.98;
  }
  
  //determine direction of right wheel
  if( dir_and_dis[2] * 1 > 0 )
  { 
    DIR_R = high;
    DIR_RR = low;
  }
  else if( dir_and_dis[2] * 1 < 0 )
  {
    DIR_R = low;
    DIR_RR = high;
    distance_R *= 0.96;
  }
  
  delay(100);
  
  //move foreward
  if(distance_L > 0 && distance_R >0)
  {
    SPEED_LEFT = SPEED_L;
    SPEED_RIGHT = SPEED_R;
    while(count_left_encoder < distance_L || count_right_encoder < distance_R)
    {
      count_left_encoder = ENCODER_LEFT.read();
      count_right_encoder = ENCODER_RIGHT.read();
      digitalWrite( LEFT_MOTOR_DIR_PIN, DIR_L );
      analogWrite( LEFT_MOTOR_PWM_PIN, SPEED_LEFT );
      digitalWrite( RIGHT_MOTOR_DIR_PIN, DIR_R );
      analogWrite( RIGHT_MOTOR_PWM_PIN, SPEED_RIGHT );
      digitalWrite( LEFT_MOTOR_DIR, DIR_LL );
      analogWrite( LEFT_MOTOR_PWM, SPEED_LEFT );
      digitalWrite( RIGHT_MOTOR_DIR, DIR_RR );
      analogWrite( RIGHT_MOTOR_PWM, SPEED_RIGHT );

      if(count_left_encoder > distance_L)
      {
        SPEED_LEFT = 0;
        digitalWrite( LEFT_MOTOR_DIR_PIN, DIR_L );
        analogWrite( LEFT_MOTOR_PWM_PIN, SPEED_LEFT );
        digitalWrite( LEFT_MOTOR_DIR, DIR_LL );
        analogWrite( LEFT_MOTOR_PWM, SPEED_LEFT );
      }
      else if(count_right_encoder > distance_R)
      {
        SPEED_RIGHT = 0;
        digitalWrite( RIGHT_MOTOR_DIR_PIN, DIR_R );
        analogWrite( RIGHT_MOTOR_PWM_PIN, SPEED_RIGHT );
        digitalWrite( RIGHT_MOTOR_DIR, DIR_RR );
        analogWrite( RIGHT_MOTOR_PWM, SPEED_RIGHT );
      }
    }
    ENCODER_LEFT.write(0);
    ENCODER_RIGHT.write(0);
  }
  
  distance_L = 0;
  distance_R = 0;
  DIR_L = 0;
  DIR_R = 0;
  
  //stop robot
  digitalWrite( LEFT_MOTOR_DIR_PIN, HIGH );
  analogWrite( LEFT_MOTOR_PWM_PIN, 0 );
  digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH );
  analogWrite( RIGHT_MOTOR_PWM_PIN, 0 );
  digitalWrite( LEFT_MOTOR_DIR, HIGH );
  analogWrite( LEFT_MOTOR_PWM, 0 );
  digitalWrite( RIGHT_MOTOR_DIR, HIGH );
  analogWrite( RIGHT_MOTOR_PWM, 0 );
  delay(500);
  
  //send done signal 'C' to smartphone
  bluetooth.println('C');
  
}

void Rotation()
{
  delay(50);
  
  //receive direction vallues of robot from smartphone
  do
  {
    if(bluetooth.available())
    {
      toSend = (char)bluetooth.read();
      movement[k++]=toSend;
      if(toSend == ' ')
      {
        dir_and_dis[n] = atof(movement); 
        k=0;
        
        n++;
      }
    }
  }while(toSend != '\n');
  n=0;
  
  //determine direction of left wheel
  if( dir_and_dis[0] * 1 > 0 )
   {
     DIR_L = high;
     DIR_LL = low;
   }
  else if( dir_and_dis[0] * 1 < 0 )
  {
    DIR_L = low;
    DIR_LL = high;
  }
  
  //determine direction of right wheel
  if( dir_and_dis[2] * 1 > 0 ) 
  { 
    DIR_R = high;
    DIR_RR = low;
  }
  else if( dir_and_dis[2] * 1 < 0 )
  {
    DIR_R = low;
    DIR_RR = high;
  }
  
  //rotate robot
  do
  {
    if(bluetooth.available())
    {
      toSend = (char)bluetooth.read();
    }
    digitalWrite( LEFT_MOTOR_DIR_PIN, DIR_L );
    analogWrite( LEFT_MOTOR_PWM_PIN, 170 );
    digitalWrite( RIGHT_MOTOR_DIR_PIN, DIR_R );
    analogWrite( RIGHT_MOTOR_PWM_PIN, 170 );
    digitalWrite( LEFT_MOTOR_DIR, DIR_LL );
    analogWrite( LEFT_MOTOR_PWM, 170 );
    digitalWrite( RIGHT_MOTOR_DIR, DIR_RR );
    analogWrite( RIGHT_MOTOR_PWM, 170 );
  }while(toSend != 'E');
  
  //stop robot
  digitalWrite( LEFT_MOTOR_DIR_PIN, HIGH );
  analogWrite( LEFT_MOTOR_PWM_PIN, 0 );
  digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH );
  analogWrite( RIGHT_MOTOR_PWM_PIN, 0 );
  digitalWrite( LEFT_MOTOR_DIR, HIGH );
  analogWrite( LEFT_MOTOR_PWM, 0 );
  digitalWrite( RIGHT_MOTOR_DIR, HIGH );
  analogWrite( RIGHT_MOTOR_PWM, 0 );
  
  delay(500);
  //send done signal 'C' to smartphone
  bluetooth.println("C"); 
}

void Check_obstacle()
{
  delay(50);
  
  //receive angle for checking distance
  do
  {
    if(bluetooth.available())
    {
      toSend = (char)bluetooth.read();
      rotation[l++]=toSend;
      if(toSend == ' ')
      {
        num_and_degree[m] = atof(rotation); 
        l=0;
        m++;
      }
    }
  }while(toSend != '\n');
  m=0;
  
  servo.write(90);
  delay(50);

  degree_of_rotation = num_and_degree[1] + 90;
  
  servo.write(degree_of_rotation);
  delay(1000);
  
  //check distance
  if(num_and_degree[0] == 1)
  {
    obstacle_distance = sensor1.ping_cm();
  }
  else if(num_and_degree[0] == 2)
  {
    obstacle_distance = sensor2.ping_cm();
  }
  else if(num_and_degree[0] == 3)
  {
    obstacle_distance = sensor3.ping_cm();
  }
  else if(num_and_degree[0] == 4)
  {
    obstacle_distance = sensor4.ping_cm();
  }  
  
  //send distance to smartphone
  bluetooth.println(obstacle_distance);
  obstacle_distance = 0;
  servo.write(90);
  delay(10);

}
