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
  go_forward(200);
  //turn_left(200,0.2);
  //turn_left_onspot(200);
  //move_only_right();
  // put your main code here, to run repeatedly:

}
