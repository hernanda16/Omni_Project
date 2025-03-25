#include <ros.h>
#include <PinChangeInt.h>
#include <digitalWriteFast.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_srvs/SetBool.h>

#define MSG_PUB_RATE 20 // publishing rate in Hz.
#define PWD_TIMEOUT 3 // motor power time-out in s.

// control loop timing constants
#define LOOPTIME 1000000 / MSG_PUB_RATE // loop time in us
#define PWD_TIMEOUT_VAL PWD_TIMEOUT * MSG_PUB_RATE // nof loop iterations before entering the low power mode when receiving no commands.

#define M0_PWM_PIN 3 // TIM2 OC2B
#define M0_DIR_PIN 2
#define M1_PWM_PIN 11 // TIM2 OC2A
#define M1_DIR_PIN 12
#define M2_PWM_PIN 9 // TIM1 OC1A
#define M2_DIR_PIN 8
#define M3_PWM_PIN 10 // TIM1 OC1B
#define M3_DIR_PIN 7
#define ENC0_A_PIN 4
#define ENC0_B_PIN 5
#define ENC1_A_PIN 14
#define ENC1_B_PIN 15
#define ENC2_A_PIN 16
#define ENC2_B_PIN 17
#define ENC3_A_PIN 18
#define ENC3_B_PIN 19
#define LED 13 //led is flashing when message received.

//encoders
volatile int encTicks0;
volatile int encTicks1;
volatile int encTicks2;
volatile int encTicks3;
int encTicks0_prev;
int encTicks1_prev;
int encTicks2_prev;
int encTicks3_prev;
volatile int pwmVal[4];
unsigned long loopTimer;
volatile int timeOutCnt;
volatile int ledCnt = 0;
volatile boolean stopmotors = false; //initially the controller is on.
// function prototype
void disableMotors(void);

std_msgs::Int16MultiArray enc_msg;
ros::NodeHandle nh; 
ros::Publisher pub("/device/raw_enc", &enc_msg);

void cmdMotors_CallBack(const std_msgs::Int16MultiArray& msg) {

    pwmVal[0] = constrain(msg.data[0], -255, 255);
    pwmVal[1] = constrain(msg.data[1], -255, 255);
    pwmVal[2] = constrain(msg.data[2], -255, 255);
    pwmVal[3] = constrain(msg.data[3], -255, 255);

    if(!stopmotors){
        if(pwmVal[0] >= 0) {
        digitalWriteFast(M0_DIR_PIN, HIGH);
        }
        else {
        digitalWriteFast(M0_DIR_PIN, LOW);
        }
        if(pwmVal[1] >= 0) {
        digitalWriteFast(M1_DIR_PIN, HIGH);
        }
        else {
        digitalWriteFast(M1_DIR_PIN, LOW);
        }
        if(pwmVal[2] >= 0) {
        digitalWriteFast(M2_DIR_PIN, HIGH);
        }
        else {
        digitalWriteFast(M2_DIR_PIN, LOW);
        }
        if(pwmVal[3] >= 0) {
        digitalWriteFast(M3_DIR_PIN, HIGH);
        }
        else {
        digitalWriteFast(M3_DIR_PIN, LOW);
        }

        if(timeOutCnt==0) {
        analogWrite(M0_PWM_PIN, abs(pwmVal[0])); // first time write to start PWM
        analogWrite(M1_PWM_PIN, abs(pwmVal[1]));
        analogWrite(M2_PWM_PIN, abs(pwmVal[2]));
        analogWrite(M3_PWM_PIN, abs(pwmVal[3]));
        }
        else {
        OCR2B = abs(pwmVal[0]); // fast PWM update M0
        OCR2A = abs(pwmVal[1]); // fast PWM update M1
        OCR1A = abs(pwmVal[2]); // fast PWM update M2
        OCR1B = abs(pwmVal[3]); // fast PWM update M3
        }
        timeOutCnt = PWD_TIMEOUT_VAL; //set timer
        digitalWriteFast(LED, HIGH);
        ledCnt = 20;
    }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("/device/cmd_motor", cmdMotors_CallBack);

void EmergencyStop_CallBack(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    bool enable = req.data;
    if(enable) {
        disableMotors();
        stopmotors = true;
    }
    res.success = true;
    res.message = "Emergency stop";
}
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> halt_srv("emergency_stop_enable", &EmergencyStop_CallBack);


void ArmingEnable_CallBack(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    bool enable = req.data;
    if(enable) {
        stopmotors = false;
    }
    res.success = true;
    res.message = "Arming enable";
}
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> arming_srv("arming_enable", &ArmingEnable_CallBack);


void disableMotors(void) {
    analogWrite(M0_PWM_PIN, 0);
    analogWrite(M1_PWM_PIN, 0);
    analogWrite(M2_PWM_PIN, 0);
    analogWrite(M3_PWM_PIN, 0);
}


void IsrEnc_0_A() {
  encTicks0 -= digitalReadFast(ENC0_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_1_A() {
  encTicks1 -= digitalReadFast(ENC1_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_2_A() {
  encTicks2 -= digitalReadFast(ENC2_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_3_A() {
  encTicks3 -= digitalReadFast(ENC3_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void setup() {
  pinMode(M0_PWM_PIN, OUTPUT); //left front wheel
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT); //left bottom wheel
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT); //right bottom wheel
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT); //right front wheel
  pinMode(M3_DIR_PIN, OUTPUT);

  pinMode(ENC0_A_PIN, INPUT); //left front enc
  pinMode(ENC0_B_PIN, INPUT);
  pinMode(ENC1_A_PIN, INPUT); //left bottom enc
  pinMode(ENC1_B_PIN, INPUT);
  pinMode(ENC2_A_PIN, INPUT); //right bottom enc
  pinMode(ENC2_B_PIN, INPUT);
  pinMode(ENC3_A_PIN, INPUT); //right front enc
  pinMode(ENC3_B_PIN, INPUT);

  pinMode(LED, OUTPUT);

  PCattachInterrupt(ENC0_A_PIN, IsrEnc_0_A, RISING);
  PCattachInterrupt(ENC1_A_PIN, IsrEnc_1_A, RISING);
  PCattachInterrupt(ENC2_A_PIN, IsrEnc_2_A, RISING);
  PCattachInterrupt(ENC3_A_PIN, IsrEnc_3_A, RISING);

  // modify PWM frequency of motors
  TCCR1B = (TCCR1B & 0xF8) | 0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B = (TCCR2B & 0xF8) | 0x01;    // Pin3,Pin11 PWM 31250Hz

  enc_msg.data_length = 4;
  enc_msg.data = (int16_t *)malloc(enc_msg.data_length * sizeof(int16_t));

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  nh.advertiseService(halt_srv);
  nh.advertiseService(arming_srv);

  timeOutCnt = PWD_TIMEOUT_VAL;
  loopTimer = micros() + LOOPTIME; //Set the loopTimer variable.
}

void loop() {
  //calculate delta-counts for the current cycle
  enc_msg.data[0] = encTicks0 - encTicks0_prev;
  enc_msg.data[1] = encTicks1 - encTicks1_prev;
  enc_msg.data[2] = encTicks2 - encTicks2_prev;
  enc_msg.data[3] = encTicks3 - encTicks3_prev;

  encTicks0_prev = encTicks0;
  encTicks1_prev = encTicks1;
  encTicks2_prev = encTicks2;
  encTicks3_prev = encTicks3;

  pub.publish(&enc_msg);

  timeOutCnt--;
  if(timeOutCnt < 0) {
    disableMotors();
    timeOutCnt = 0;
  }
  ledCnt--;
  if(ledCnt == 0) {
    digitalWriteFast(LED, LOW);
  }

  nh.spinOnce();
  
  // Wait for the remaining time in the loop and set the new loopTimer value.
  while(loopTimer > micros()) {;}
  loopTimer += LOOPTIME;
}