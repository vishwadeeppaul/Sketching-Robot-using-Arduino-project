//Xtrobots.in
//13-09-2018

#include <Servo.h>

//motor variable declaration
#define omotM1 13 //speed control pin of motor 1
#define omotM2  8 //speed control pin of motor 2
#define cmot1M1  12 //direction control pin of motor 1
#define cmot2M1  10 //direction control pin of motor 1
#define cmot1M2  11 //direction control pin of motor 2
#define cmot2M2  9 //direction control pin of motor 2

#define outputAM1 2 //pin at which outputA is connected for motor 1
#define outputBM1 3 //pin at which outputB is connected for motor 1
#define outputAM2 18 //pin at which outputA is connected for motor 2
#define outputBM2 19 //pin at which outputB is connected for motor 2

volatile long counterM1 = 0;
volatile long counterM2 = 0;
float theta = 0;
float error = 0;
float errorIntegral = 0;
float derror = 0;
float lerror = 0;
int errorT = 0;
float kp = 3.5; //values are still not checked by hit n trial
int ki = 0;
float kd = 1.5;
int errorM1 = 0 , errorM2 = 0;
int edgelength = 0;
int j = 0 , n = 0;
float Aerror = 0;
float angle = 0;
char input;
int xcor = 0 , ycor = 0;
int lxcor = 0 , lycor = 0;
float ltheta = 0 ;
float dis =0 , ldis = 0; 
float sqdis = 0 ;
int temp = 0 ;

Servo servo;


void setup() {
  
  Serial.begin(9600);
  pinMode(omotM1, OUTPUT);
  pinMode(omotM2, OUTPUT);
  pinMode(cmot1M1, OUTPUT);
  pinMode(cmot1M2, OUTPUT);
  pinMode(cmot2M1, OUTPUT);
  pinMode(cmot2M2, OUTPUT);

  //encoder setup
  pinMode(outputAM2, INPUT);
  pinMode(outputBM2, INPUT);
  pinMode(outputAM1, INPUT);
  pinMode(outputBM1, INPUT);
  attachInterrupt(digitalPinToInterrupt(outputAM1), countAM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputBM1), countBM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputAM2), countAM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputBM2), countBM2, CHANGE);

  servo.attach(4);
  servo.write(120);


}

void loop() {
  while (Serial.available() == 0);
  input = Serial.read();
  if (input == 'X' || input == 'x') {
    delay(100);
    while (Serial.available() != 0 ) {
      temp = int(Serial.read());
      if(temp == 32)break;
      xcor = temp - int('0') + xcor * 10;
      delay(10);
    }
  }
  while (Serial.available() == 0);
  input = Serial.read();
  if (input == 'Y' || input == 'y') {
    delay(10);
    while (Serial.available() != 0 ) {
      ycor = int(Serial.read()) - int('0') + ycor * 10;
      delay(10);
    }
  }
  Serial.print(xcor);
  Serial.println(ycor);
  sqdis = sq(xcor-lxcor) + sq(ycor - lycor);
  dis = sqrt(sqdis);
  theta = atan2((ycor - lycor),(xcor - lxcor));
  theta = theta * 180 / 3.14;
  errorM1 = 0;
  errorM2 = 0;
  counterM1 = 0;
  counterM2 = 0;
   Serial.println(dis);
  if (theta -ltheta < 0){
    right(theta - ltheta);
  }
  else if (theta -ltheta > 0){
    left(theta - ltheta);
  }
  else {
    if(xcor > lxcor){
      delay(400);
    }
    else{
      right(180);
    }
  }
  errorM1 = 0;
  errorM2 = 0;
  counterM1 = 0;
  counterM2 = 0;
  
  while ((counterM1+counterM2)/2 < (dis) * 66.31456) {
    apid();
    Serial.print("errorM1 :");
    Serial.println(errorM1);
    Serial.print("errorM2 :");
    Serial.println(errorM2);
    if ((counterM1+counterM2)/2 * 0.25 + 100 <=150) {
      analogWrite(omotM1, counterM1 * 0.25 + 100 - errorM1);
      analogWrite(omotM2, counterM1 * 0.25 + 100 - errorM2);

      digitalWrite(cmot1M1, LOW);
      digitalWrite(cmot1M2, HIGH);
      digitalWrite(cmot2M1, LOW);
      digitalWrite(cmot2M2, HIGH);
    }
    else if ((dis) * 66.31456 - counterM1 * 0.25 <= 150) {
      analogWrite(omotM1, 150 - (dis) * 66.31456 + counterM1 * 0.25 + 100 - errorM1);
      analogWrite(omotM2, 150 - (dis) * 66.31456 + counterM1 * 0.25 + 100 - errorM2);

      digitalWrite(cmot1M1, LOW);
      digitalWrite(cmot1M2, HIGH);
      digitalWrite(cmot2M1, LOW);
      digitalWrite(cmot2M2, HIGH);
    }
    else {
      analogWrite(omotM1, 150 - errorM1);
      analogWrite(omotM2, 150 - errorM2);

      digitalWrite(cmot1M1, LOW);
      digitalWrite(cmot1M2, HIGH);
      digitalWrite(cmot2M1, LOW);
      digitalWrite(cmot2M2, HIGH);
    }
  }

  Stop();
  delay(800);
  //    do{
  //    while (-counterM1 <= theta * 9.5) {
  //      analogWrite(omotM1, 90);
  //      analogWrite(omotM2, 80);
  //
  //      digitalWrite(cmot1M1, HIGH);
  //      digitalWrite(cmot1M2, LOW);
  //      digitalWrite(cmot2M1, LOW);
  //      digitalWrite(cmot2M2, HIGH);
  //    }
  //    while(-counterM1 >= theta * 10.5) {
  //
  //      analogWrite(omotM1, 90);
  //      analogWrite(omotM2, 80);
  //
  //      digitalWrite(cmot1M1, LOW);
  //      digitalWrite(cmot1M2, HIGH);
  //      digitalWrite(cmot2M1, HIGH);
  //      digitalWrite(cmot2M2, LOW);
  //    }
  //    delay(300);
  //  }
  //
  //    while(-counterM1 <= 9.5 * theta && -counterM1 >=  theta * 11);
  //
  //  analogWrite(omotM1, 0);
  //  analogWrite(omotM2, 0);
  //
  //  digitalWrite(cmot1M1, HIGH);
  //  digitalWrite(cmot1M2, HIGH);
  //  digitalWrite(cmot2M1, HIGH);
  //  digitalWrite(cmot2M2, HIGH);
  //  //  while (-counterM1 < theta * 11.1  ){
  //  //
  //  //    analogWrite(omotM1, 100);
  //  //    analogWrite(omotM2, 90);
  //  //
  //  //    digitalWrite(cmot1M1, HIGH);
  //  //    digitalWrite(cmot1M2, LOW);
  //  //    digitalWrite(cmot2M1, LOW);
  //  //    digitalWrite(cmot2M2, HIGH);

  analogWrite(omotM1, 0);
  analogWrite(omotM2, 0);

  digitalWrite(cmot1M1, LOW);
  digitalWrite(cmot1M2, LOW);
  digitalWrite(cmot2M1, LOW);
  digitalWrite(cmot2M2, LOW);
  lxcor = xcor;
  lycor = ycor;
  ltheta = theta;
  ldis = dis;
  xcor = 0;
  ycor = 0;
}



void countAM1() {
  if ( digitalRead ( outputAM1 ) == HIGH) {
    if ( digitalRead ( outputBM1 ) == LOW) {
      counterM1 = counterM1 + 1 ;
    }
    else {
      counterM1 = counterM1 - 1 ;
    }
  }
  else {
    if ( digitalRead(outputBM1 ) == HIGH) {
      counterM1 = counterM1 + 1 ;
    }
    else {
      counterM1 = counterM1 - 1 ;

    }
  }
}
void countBM1 () {
  if (digitalRead(outputBM1) == HIGH) {
    if (digitalRead(outputAM1) == HIGH) {
      counterM1 = counterM1 + 1;
    }
    else {
      counterM1 = counterM1 - 1;
    }
  }
  else {
    if (digitalRead(outputAM1) == LOW) {
      counterM1 = counterM1 + 1;
    }
    else {
      counterM1 = counterM1 - 1;
    }
  }
}
void countAM2() {
  if ( digitalRead ( outputAM2 ) == HIGH) {
    if ( digitalRead ( outputBM2 ) == LOW) {
      counterM2 = counterM2 + 1 ;
    }
    else {
      counterM2 = counterM2 - 1 ;
    }
  }
  else {
    if ( digitalRead(outputBM2 ) == HIGH) {
      counterM2 = counterM2 + 1 ;
    }
    else {
      counterM2 = counterM2 - 1 ;
    }
  }

}
void countBM2 () {
  if (digitalRead(outputBM2) == HIGH) {
    if (digitalRead(outputAM2) == HIGH) {
      counterM2 = counterM2 + 1;
    }
    else {
      counterM2 = counterM2 - 1;
    }
  }
  else {
    if (digitalRead(outputAM2) == LOW) {
      counterM2 = counterM2 + 1;
    }
    else {
      counterM2 = counterM2 - 1;
    }
  }
}
void apid() {
  error = abs(counterM1 - counterM2);
  derror = error - lerror;
  errorT = kp * error + kd * derror;
  lerror = error;
  if (errorT > 80) {
    errorT = 80;
  }
  if (counterM1 > counterM2) {
    errorM1 = errorT;
    errorM2 = 0;
  }
  else {
    errorM2 = errorT;
    errorM1 = 0;
  }
}
void Stop() {

  analogWrite(omotM1, 255 );
  analogWrite(omotM2, 255 );

  digitalWrite(cmot1M1, HIGH);
  digitalWrite(cmot1M2, LOW);
  digitalWrite(cmot2M1, HIGH);
  digitalWrite(cmot2M2, LOW);
  delay(20);

  analogWrite(omotM1, 255 );
  analogWrite(omotM2, 255 );

  digitalWrite(cmot1M1, HIGH);
  digitalWrite(cmot1M2, HIGH);
  digitalWrite(cmot2M1, HIGH);
  digitalWrite(cmot2M2, HIGH);
}
void right(double angle){
  while ((counterM2 - counterM1) / 2 < -angle * 11.8) {
    analogWrite(omotM1, 92);
    analogWrite(omotM2, 85);

    digitalWrite(cmot1M1, HIGH);
    digitalWrite(cmot1M2, LOW);
    digitalWrite(cmot2M1, LOW);
    digitalWrite(cmot2M2, HIGH);
  }

  analogWrite(omotM1, 0);
  analogWrite(omotM2, 0);

  digitalWrite(cmot1M1, HIGH);
  digitalWrite(cmot1M2, HIGH);
  digitalWrite(cmot2M1, HIGH);
  digitalWrite(cmot2M2, HIGH);

  delay(800);
}
void left( double angle) {
  while (-(counterM2 - counterM1) / 2 < angle * 11.8) {
    analogWrite(omotM1, 92);
    analogWrite(omotM2, 85);

    digitalWrite(cmot1M1, LOW);
    digitalWrite(cmot1M2, HIGH);
    digitalWrite(cmot2M1, HIGH);
    digitalWrite(cmot2M2, LOW);
  }

  analogWrite(omotM1, 0);
  analogWrite(omotM2, 0);

  digitalWrite(cmot1M1, HIGH);
  digitalWrite(cmot1M2, HIGH);
  digitalWrite(cmot2M1, HIGH);
  digitalWrite(cmot2M2, HIGH);

  delay(800);
 }
