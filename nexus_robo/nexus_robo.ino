#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>
#include <stdio.h>

irqISR(irq1, isr1);
MotorWheel wheel1(3, 2, 4, 5, &irq1);

irqISR(irq2, isr2);
MotorWheel wheel2(11, 12, 14, 15, &irq2);

irqISR(irq3, isr3);
MotorWheel wheel3(9, 8, 16, 17, &irq3);

irqISR(irq4, isr4);
MotorWheel wheel4(10, 7, 18, 19, &irq4);

Omni4WD Omni(&wheel1, &wheel2, &wheel3, &wheel4);

unsigned long previousMillis = 0;
unsigned long interval = 20;  // Default interval (50Hz)

float vx = 0;
float vy = 0;
float omega = 0;

struct WheelData {
  int wheelULPulse;
  int wheelLLPulse;
  int wheelLRPulse;
  int wheelURPulse;
};

float buffer[3];
float pidParams[3];  // Buffer for PID parameters: Kp, Ki, Kd
unsigned int pid_interval = 1;

void setup() {
  Serial.begin(115200);
  TCCR1B = TCCR1B & 0xf8 | 0x01;  // Pin9, Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01;  // Pin3, Pin11 PWM 31250Hz

  // Buffer to receive PID parameters and interval
  float receivedParams[4];  // Kp, Ki, Kd, interval

  // Wait for PID parameters and interval from serial
  while (Serial.available() < sizeof(receivedParams)) {
    // Wait until all parameters are received
  }
  Serial.readBytes((char*)receivedParams, sizeof(receivedParams));

  // Extract PID parameters and interval
  pidParams[0] = receivedParams[0];  // Kp
  pidParams[1] = receivedParams[1];  // Ki
  pidParams[2] = receivedParams[2];  // Kd
  pid_interval = (unsigned int)receivedParams[3];  // Interval in milliseconds

  // Enable PID with received parameters
  Omni.PIDEnable(pidParams[0], pidParams[1], pidParams[2], pid_interval);
}

void loop() {
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    // Receive velocity commands from RPi
    if (Serial.available() >= sizeof(buffer)) {
      Serial.readBytes((char*)buffer, sizeof(buffer));
      vx = buffer[0];
      vy = buffer[1];
      omega = buffer[2];

      vx = vx * 1000;
      vy = vy * 1000;
      omega = omega * 1000;

      Omni.kinematics(vx, vy, omega);
    }

    // Prepare wheel pulse data to send to RPi
    WheelData wheelData;
    wheelData.wheelULPulse = -wheel1.getCurrPulse();
    wheelData.wheelLLPulse = -wheel2.getCurrPulse();
    wheelData.wheelLRPulse = -wheel3.getCurrPulse();
    wheelData.wheelURPulse = -wheel4.getCurrPulse();

    // Send wheel pulse data
    Serial.write((char*)&wheelData, sizeof(wheelData));

    Omni.PIDRegulate();
  }
}
