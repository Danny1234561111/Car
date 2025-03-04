#include <PID_v1.h>
#define TRIGGER_PIN_LEFT  8  // Боковой датчик
#define ECHO_PIN_LEFT     2
#define TRIGGER_PIN_FORWARD  9 // Передний датчик
#define ECHO_PIN_FORWARD     3

#define DESIRED_DISTANCE 15 // Желаемое расстояние до стены (в см)
#define FRONT_THRESHOLD 15 // Мин расстояние до препятствия впереди

bool flag = false;
unsigned long int start_time_left = 0;
unsigned long int time_left = 0;
unsigned int distance_left = 0;
unsigned long int start_time_forward = 0;
unsigned long int time_forward = 0;
unsigned int distance_forward = 0;
bool left_recieved = false;
bool forward_recieved = false;
int delay 100;
unsigned long int start_time = 0;

// PID параметры
double input, output, setpoint;
double Kp = 2, Ki = 5, Kd = 1; // Настройте эти значения для вашей системы
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

#define LEFT_DIR_PIN 7
#define LEFT_SPEED_PIN 6
#define RIGHT_SPEED_PIN 5
#define RIGHT_DIR_PIN 4

#define LEFT_SIDE_FORWARD HIGH
#define RIGHT_SIDE_FORWARD LOW
#define LEFT_SIDE_BACKWARD LOW
#define RIGHT_SIDE_BACKWARD HIGH
void setup() {
  for (int i=4;i<=7;i++){
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);
  }
  setpoint = DESIRED_DISTANCE;
  myPID.SetMode(AUTOMATIC);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_LEFT), sound_dist_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FORWARD), sound_dist_forward, CHANGE);

  cli(); 
  TCCR1A=0; 
  TCCR1B=0; 
  TCNT1=0; 
  OCR1A = 159;
  TCCR1B |= (1 << WGM12); 
  TCCR1B = TCCR1B | (1 << CS10); 
  TIMSK1 |= (1 << OCIE1A); 
  sei();
}

ISR(TIMER1_COMPA_vect) {
  if (flag){
    PORTD|=(1<<trig);
    flag=false;
  }
  else{
    PORTD&=~(1<<trig);
  }
}

void sound_dist_left(){
  if (start_time_left == 0) start_time_left = micros();
  else {
    time_left = micros() - start_time_left;
    start_time_left=0;
    distance_left = (time_left * 0.034) / 2;
    left_recieved=true;
  }
}

void sound_dist_forward(){
  if (start_time_forward == 0) start_time_forward = micros();
  else {
    time_forward = micros() - start_time_forward;
    start_time_forward=0;
    distance_forward = (time_forward * 0.034) / 2;
    forward_recieved=true;
  }
}

void move(bool dir_left,int speed_left,bool dir_right,int speed_right){
  if (speed_left>255){
    speed_left=255;
  }
  if (speed_right>255){
    speed_right=255;
  }
  digitalWrite(LEFT_DIR_PIN,dir_left);
  analogWrite(LEFT_SPEED_PIN,int(speed_left));
  digitalWrite(RIGHT_DIR_PIN,dir_right);
  analogWrite(RIGHT_SPEED_PIN,speed_right);

}
void move_only_right(){
  move(true,0,true,200);
}
void go_forward(int speed){
  move(LEFT_SIDE_FORWARD,speed,RIGHT_SIDE_FORWARD,speed);
}
void go_backward(int speed){
  move(LEFT_SIDE_BACKWARD,speed,RIGHT_SIDE_BACKWARD,speed);
}
void turn_left(int speed,float ratio){
  move(LEFT_SIDE_FORWARD,int(speed*ratio),RIGHT_SIDE_FORWARD,speed);
}
void turn_left_onspot(int speed){
  move(LEFT_SIDE_BACKWARD,speed,RIGHT_SIDE_FORWARD,speed);
}
void turn_right_onspot(int speed){
  move(LEFT_SIDE_FORWARD,speed,RIGHT_SIDE_BACKWARD,speed);
}
void stop(){
  move(0,0,0,0);
}
void loop() {
  if (millis()>start_time+delay){
    start_time=millis();
    if (left_recieved && forward_recieved){
      left_recieved=false;
      forward_recieved=false;
      input = distance_left;
      myPID.Compute();
      controlMotors(output);
    }
    else{
      flag=true;
    }
  }
  //go_forward(200);
  //turn_left(200,0.2);
  //turn_left_onspot(200);
  //move_only_right();
  // put your main code here, to run repeatedly:

}

void controlMotors(double pidOutput) {
  int baseSpeed = 150; // Базовая скорость

  // Устанавливаем скорость для левого и правого моторов
  int leftMotorSpeed = baseSpeed - pidOutput;
  int rightMotorSpeed = baseSpeed + pidOutput;

  // Ограничение скорости
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Движение в зависимости от рассчитанных скоростей
  if (leftMotorSpeed > 0) {
    move(LEFT_SIDE_FORWARD, leftMotorSpeed, RIGHT_SIDE_FORWARD, rightMotorSpeed);
  } else {
    move(LEFT_SIDE_BACKWARD, -leftMotorSpeed, RIGHT_SIDE_FORWARD, rightMotorSpeed);
  }
}
