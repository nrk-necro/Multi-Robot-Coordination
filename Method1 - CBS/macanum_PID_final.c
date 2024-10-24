#include <bits/stdc++.h> 

// Motor pin definitions

// Motor FR
#define ENB_FR 8// Enable B
#define IN_FR1 22
#define IN_FR2 23

// Define encoder connections
#define ENCODER_PIN_FR_A 2
#define ENCODER_PIN_FR_B 4


// Motor FL
#define ENB_FL 9// Enable B
#define IN_FL1 24
#define IN_FR2 25

// Define encoder connections
#define ENCODER_PIN_FL_A 3
#define ENCODER_PIN_FL_B 5


// Motor BL
#define ENB_BL 10// Enable B
#define IN_BL1 26
#define IN_BL2 27

// Define encoder connections
#define ENCODER_PIN_BL_A 18
#define ENCODER_PIN_BL_B 6


// Motor BR
#define ENB_BR 11// Enable B
#define IN_BR1 28
#define IN_BR2 2

// Define encoder connections
#define ENCODER_PIN_BR_A 19
#define ENCODER_PIN_BR_B 7

////////////////////////////////


// Define variables for encoder for single motor
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile int lastEncoded[4] = {0, 0, 0, 0};

int speed = 50;
int direction = 1; // 1: Forward, 2: Backward, 3: LeftR, 4: RightR

float RotateAngle = 90.0;
double countsPerRevolution=334; // Variable to store counts per revolution
long int LinearCountWheel= 100; // No of counts each wheel of the robot should rotate through.
float RotateCountWheel= (RotateAngle/360)* countsPerRevolution 

if (direction=(1 or 2)){
volatile int targetCountPerWheel[4]={LinearCountWheel,LinearCountWheel,LinearCountWheel,LinearCountWheel}
}
else if (direction=(3 or 4)){
volatile int targetCountPerWheel[4]={RotateCountWheel,RotateCountWheel,RotateCountWheel,RotateCountWheel}
}

/*  Target counts will be written for each wheel
1: Front Right
2: Front Left
3: Back Right
4: Back Left
*/

unsigned long previousMillis = 0;
const int sampleTime = 50; // Sampling time in milliseconds


// PID constants
double Kp[4] = {4, 4, 4, 4}; // Proportional constant
double Ki[4] = {0.00118, 0.00118, 0.00118, 0.00118}; // Integral constant
double Kd[4] = {0.88, 0.88, 0.88, 0.88}; // Derivative constant

// Previous error and integral for PID control (separate for each motion)
double motorError[4] = {0, 0, 0, 0};
double motorIntegral[4] = {0, 0, 0, 0};

//////////////////////////////

void setup() {
  
 // Motor driver pins as outputs
  pinMode(ENB_FR, OUTPUT);
  pinMode(IN_FR1, OUTPUT);
  pinMode(IN_FR2, OUTPUT);
  
  pinMode(ENB_FL, OUTPUT);
  pinMode(IN_FL1, OUTPUT);
  pinMode(IN_FL2, OUTPUT);
  
  pinMode(ENB_BR, OUTPUT);
  pinMode(IN_BR1, OUTPUT);
  pinMode(IN_BR2, OUTPUT);
  
  pinMode(ENB_BL, OUTPUT);
  pinMode(IN_BL1, OUTPUT);
  pinMode(IN_BL2, OUTPUT);
  
  
  // Encoder pins as inputs
  pinMode(ENCODER_PIN_FR_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_FR_B, INPUT_PULLUP);
  
  pinMode(ENCODER_PIN_FL_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_FL_B, INPUT_PULLUP);
    
  pinMode(ENCODER_PIN_BR_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_BR_B, INPUT_PULLUP);
    
  pinMode(ENCODER_PIN_BL_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_BL_B, INPUT_PULLUP);
  
  //Interrupt sequence
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_FR_A), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_FL_A), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_BR_A), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_BL_A), updateEncoder, RISING);

  // Initialize serial communication (for user input)
  Serial.begin(9600);

}

///////////////////////////

void loop() {

  calculateTargetCounts(direction, targetCountPerWheel);
  
  while (EncoderCount != targetCountPerWheel) {
    updateEncoder();

    // Implement PID control for each motor
    for (int i = 1; i <= 4; i++) {
      motorControlPID(i, targetCounts[i]);
    }
    
    delay(sampleTime);
    
  }
  stop();
}

////////////////////////////

// C++ program to find the index  of an element in a vector 
int getIndex(vector<int> v, int K) {
 
  auto it = find(v.begin(), v.end(), K); 
  // If element was found 
  if (it != v.end()) {
  // calculating the index of K 
      int index = it - v.begin(); 
      
   } 
  return index;
} 


// Function to update encoder count (interrupt service routine)
void updateEncoder() {
  vector<int> l[4] ={ENCODER_PIN_FR_B,ENCODER_PIN_FL_B,ENCODER_PIN_BR_B,ENCODER_PIN_BL_B};
  for (i in l){
  int b = digitalRead(i);
  if (b > 0) {
    encoderCount[getIndex(vector<int> l, int i)]+=1;
  } 
  else {
    encoderCount[getIndex(vector<int> l, int i))]-=1;
  }
  }
}

////////////////////////////

void motorControlPID(int motorIndex, long targetCount, int direction) {
 
  double error[4] = targetCount - encoderCount;
  motorIntegral[motorIndex] += error;
  double derivative = ((error - motorError[motorIndex]) * 1000) / sampleTime;
  motorError[motorIndex] = error;
  double controlSignal = Kp * error + Ki * motorIntegral[motorIndex] + Kd * derivative;
  

  for (int i =0, i<4, i++ ) {
   
   // Control motor based on control signal
  if ((abs(targetCount[i]-encoderCount[i]))<(0.03*(targetCount[i]))){
    stop();
  }
  else if (controlSignal[i] > 0){
    analogWrite(ENB, controlSignal[i]); // Set PWM value for speed control
    digitalWrite(IN3, HIGH); // Rotate motor in one direction
    digitalWrite(IN4, LOW);
    EnableMotion(int direction);
    
  } 
  else if (controlSignal[i] < 0) {
    analogWrite(ENB, abs(controlSignal[i])); // Set PWM value for speed control (positive value)
    digitalWrite(IN3, LOW); // Rotate motor in the other direction
    digitalWrite(IN4, HIGH);
    EnableMotion(int direction);
  } 
  else {
    stop();
  }  
  
  prevError = error;
  
  // Sampling time delay
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sampleTime) {
    previousMillis = currentMillis;
  }
  }
}
  
//////////////////////////////

void EnableMotion(int direction ){
  if (direction ==1){
    forward()
  }
  if (direction ==2){
    backward()
  }
  if (direction ==3){
    rotateL()
  }
  if (direction ==4){
    rotateR()
  }
}

/////////////////////////////

void forward(){
  analogWrite(ENB_FR,speed);
  analogWrite(ENB_FL,speed);
  analogWrite(ENB_BR,speed);
  analogWrite(ENB_BL,speed);

  digitalWrite(IN_FR1,LOW);
  digitalWrite(IN_FR2,HIGH);
  digitalWrite(IN_FL1,LOW);
  digitalWrite(IN_FL2,HIGH);
  digitalWrite(IN_BR1,LOW);
  digitalWrite(IN_BR2,HIGH);
  digitalWrite(IN_BL1,LOW);
  digitalWrite(IN_BL2,HIGH);

  delay(1000);
  }


void backward(targetCountPerWheel){
  analogWrite(ENB_FR,speed);
  analogWrite(ENB_FL,speed);
  analogWrite(ENB_BR,speed);
  analogWrite(ENB_BL,speed);

  digitalWrite(IN_FR1,HIGH);
  digitalWrite(IN_FR2,LOW);
  digitalWrite(IN_FL1,HIGH);
  digitalWrite(IN_FL2,LOW);
  digitalWrite(IN_BR1,HIGH);
  digitalWrite(IN_BR2,LOW);
  digitalWrite(IN_BL1,HIGH);
  digitalWrite(IN_BL2,LOW);

  delay(1000);
  }

void rotateL(targetCountPerWheel){
  analogWrite(ENB_FR,speed);
  analogWrite(ENB_FL,speed);
  analogWrite(ENB_BR,speed);
  analogWrite(ENB_BL,speed);

  digitalWrite(IN_FR1,LOW);
  digitalWrite(IN_FR2,HIGH);
  digitalWrite(IN_FL1,HIGH);
  digitalWrite(IN_FL2,LOW);
  digitalWrite(IN_BR1,LOW);
  digitalWrite(IN_BR2,HIGH);
  digitalWrite(IN_BL1,HIGH);
  digitalWrite(IN_BL2,LOW);

  delay(1000);
  }

void rotateR(targetCountPerWheel){
  analogWrite(ENB_FR,speed);
  analogWrite(ENB_FL,speed);
  analogWrite(ENB_BR,speed);
  analogWrite(ENB_BL,speed);

  digitalWrite(IN_FR1,HIGH);
  digitalWrite(IN_FR2,LOW);
  digitalWrite(IN_FL1,LOW);
  digitalWrite(IN_FL2,HIGH);
  digitalWrite(IN_BR1,HIGH);
  digitalWrite(IN_BR2,LOW);
  digitalWrite(IN_BL1,LOW);
  digitalWrite(IN_BL2,HIGH);

  delay(1000);
  }


void stop(){
  analogWrite(ENB_FR,0);
  analogWrite(ENB_FL,0);
  analogWrite(ENB_BR,0);
  analogWrite(ENB_BL,0);
  
  digitalWrite(IN_FR1,HIGH);
  digitalWrite(IN_FR2,HIGH);
  digitalWrite(IN_FL1,HIGH);
  digitalWrite(IN_FL2,HIGH);
  digitalWrite(IN_BR1,HIGH);
  digitalWrite(IN_BR2,HIGH);
  digitalWrite(IN_BL1,HIGH);
  digitalWrite(IN_BL2,HIGH);
}



