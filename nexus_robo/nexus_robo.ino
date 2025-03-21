#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

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
const long interval = 20;  // 50Hz

void setup() {
  Serial.begin(115200);
  TCCR1B = TCCR1B & 0xf8 | 0x01;  // Pin9, Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01;  // Pin3, Pin11 PWM 31250Hz
  Omni.PIDEnable(1.5, 0.05, 0.0, 1);
}

void loop() {
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      data.trim();  // Remove any leading or trailing whitespace

      int firstSpace = data.indexOf(' ');
      int secondSpace = data.indexOf(' ', firstSpace + 1);

      if (firstSpace != -1 && secondSpace != -1) {
        float vx = data.substring(0, firstSpace).toFloat();
        float vy = data.substring(firstSpace + 1, secondSpace).toFloat();
        float omega = data.substring(secondSpace + 1).toFloat();

        Omni.kinematics(vx, vy, omega);
      }
    }

    int wheelULPulse = -wheel1.getCurrPulse();
    int wheelLLPulse = -wheel2.getCurrPulse();
    int wheelLRPulse = -wheel3.getCurrPulse();
    int wheelURPulse = -wheel4.getCurrPulse();

    Serial.print(wheelULPulse);
    Serial.print(" ");
    Serial.print(wheelLLPulse);
    Serial.print(" ");
    Serial.print(wheelLRPulse);
    Serial.print(" ");
    Serial.println(wheelURPulse);
  }
  Omni.PIDRegulate();
}