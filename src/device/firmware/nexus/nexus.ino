#include <PinChangeInt.h>
#include <digitalWriteFast.h>
#include <math.h>

// Pin Definitions - Eksplisit untuk setiap motor
#define M0_PWM_PIN 3
#define M0_DIR_PIN 2
#define M1_PWM_PIN 11
#define M1_DIR_PIN 12
#define M2_PWM_PIN 9
#define M2_DIR_PIN 8
#define M3_PWM_PIN 10
#define M3_DIR_PIN 7

#define ENC0_A_PIN 4
#define ENC0_B_PIN 5
#define ENC1_A_PIN 14
#define ENC1_B_PIN 15
#define ENC2_A_PIN 16
#define ENC2_B_PIN 17
#define ENC3_A_PIN 18
#define ENC3_B_PIN 19

#define LED_PIN 13

#define COUNTS_PER_REV 770.0  // Encoder counts per revolution
#define LOOP_FREQUENCY 20.0   // Hz
#define LOOP_PERIOD 50        // ms


// Mechanical Parameters
const float WHEEL_RADIUS = 0.05;     // meters (jari-jari roda)
const float ROBOT_RADIUS = 0.2;      // meters from center to wheel (jarak antar roda)

// Encoder Variables
volatile long encTicks[4] = {0, 0, 0, 0};
volatile long prev_encTicks[4] = {0, 0, 0, 0};

int32_t counts_per_period[4];
float vel_fb[4];      // Feedback velocity in rad/s
float vel_set[4];     // Setpoint velocity in rad/s

// PID Parameters
float kp = 18;   // Proportional gain
float ki = 0.8;   // Integral gain
float kd = 0.15;  // Derivative gain

float error[4], sum_error[4], last_error[4];
float pid_output[4];

unsigned long lastTime = 0;

// Interrupt Service Routines for Encoders
void isrEnc0A() { 
  encTicks[0] -= digitalReadFast(ENC0_B_PIN) ? -1 : 1; 
}

void isrEnc1A() { 
  encTicks[1] -= digitalReadFast(ENC1_B_PIN) ? -1 : 1; 
}

void isrEnc2A() { 
  encTicks[2] -= digitalReadFast(ENC2_B_PIN) ? -1 : 1; 
}

void isrEnc3A() { 
  encTicks[3] -= digitalReadFast(ENC3_B_PIN) ? -1 : 1; 
}

uint16_t state = 0;

void setup() {
  Serial.begin(115200);
  
  setupMotorPins();
  setupEncoderInterrupts();
}

void loop() {
  static char buffer[4]; // Buffer untuk menyimpan header
  static int bufferIndex = 0; // Indeks buffer
  static float linx = 0.0, liny = 0.0, angz = 0.0; // Default setpoint jika tidak ada data baru

  // Periksa apakah ada data serial yang tersedia
  while (Serial.available() > 0) {
      char incomingByte = Serial.read();

      // Tambahkan byte ke buffer
      buffer[bufferIndex] = incomingByte;
      bufferIndex++;

      // Debug: Cetak isi buffer
      Serial.print("Buffer: ");
      for (int i = 0; i < bufferIndex; i++) {
          Serial.print(buffer[i]);
          Serial.print(" ");
      }
      Serial.println();

      // Jika buffer penuh, periksa apakah itu header yang valid
      if (bufferIndex == 4) {
          if (buffer[0] == 'e' && buffer[1] == 'l' && buffer[2] == 'k' && buffer[3] == 'a') {
              // Header valid ditemukan, baca data float
              Serial.println("Header valid ditemukan!");

              if (Serial.available() >= 12) { // Pastikan ada cukup data untuk 3 float
                  float x, y, z;
                  Serial.readBytes((char*)&x, sizeof(float));
                  Serial.readBytes((char*)&y, sizeof(float));
                  Serial.readBytes((char*)&z, sizeof(float));

                  // Perbarui setpoint kinematics
                  linx = x;
                  liny = y;
                  angz = z;

                  // Debug: Cetak nilai setpoint yang diterima
                  Serial.print("Setpoint diterima - linx: ");
                  Serial.print(linx);
                  Serial.print(", liny: ");
                  Serial.print(liny);
                  Serial.print(", angz: ");
                  Serial.println(angz);
              } else {
                  Serial.println("Data float tidak cukup!");
              }

              // Reset buffer setelah data berhasil diproses
              bufferIndex = 0;
          } else {
              // Header tidak valid, geser buffer
              Serial.println("Header tidak valid, geser buffer!");
              buffer[0] = buffer[1];
              buffer[1] = buffer[2];
              buffer[2] = buffer[3];
              bufferIndex = 3; // Tetap di 3 karena kita geser buffer
          }
      }
  }

  // Jalankan PID dan kinematics pada interval tetap
  if (millis() - lastTime >= LOOP_PERIOD) {
    lastTime = millis();

    // Hitung kecepatan feedback (vel_fb) dari encoder
    for (int i = 0; i < 4; i++) {
      counts_per_period[i] = (int32_t)(encTicks[i] - prev_encTicks[i]);
      vel_fb[i] = ((float)counts_per_period[i] / COUNTS_PER_REV) * (2 * M_PI) * LOOP_FREQUENCY;
      prev_encTicks[i] = encTicks[i];
    }

    // Kirim data feedback ke serial
    uint8_t serial_send[20] = {'e', 'l', 'k', 'a'};
    memcpy(serial_send + 4, vel_fb, 16);
    for (int i = 0; i < 20; i++) {
      Serial.write(serial_send[i]);
    }

    // Jalankan PID untuk mengontrol motor
    Serial.println(linx);
    kinematics(linx, liny, angz);
    pidMotor(vel_set);
  }
}

void kinematics(float vx, float vy, float w) {
  // Input: vx, vy in m/s, w in rad/s
  float wheelDeg[4] = {45, 135, 225, 315};
  float rad[4];
  
  // Convert wheel angles to radians
  for (int i = 0; i < 4; i++) {
    rad[i] = wheelDeg[i] * M_PI / 180.0;
  }
  
  // Omni-directional wheel velocity calculation
  // Rumus: v_wheel = vx * cos(θ) + vy * sin(θ) + w * R
  // θ = sudut roda, R = jarak roda ke pusat robot
  // Konversi ke kecepatan sudut roda dengan membagi kecepatan linier dengan jari-jari roda
  vel_set[0] = (vx * cos(rad[0]) + vy * sin(rad[0]) + w * ROBOT_RADIUS) / WHEEL_RADIUS;
  vel_set[1] = (vx * cos(rad[1]) + vy * sin(rad[1]) + w * ROBOT_RADIUS) / WHEEL_RADIUS;
  vel_set[2] = (vx * cos(rad[2]) + vy * sin(rad[2]) + w * ROBOT_RADIUS) / WHEEL_RADIUS;
  vel_set[3] = (vx * cos(rad[3]) + vy * sin(rad[3]) + w * ROBOT_RADIUS) / WHEEL_RADIUS;
}

void pidMotor(float speedSP[4]) {
  for(int i = 0; i < 4; i++) {
    // Calculate error
    error[i] = speedSP[i] - vel_fb[i];
    
    // Integral term with anti-windup
    sum_error[i] += error[i];
    sum_error[i] = constrain(sum_error[i], -255, 255);

    // Reset integral if setpoint is zero
    if(speedSP[i] == 0) {
      sum_error[i] = 0;
    }

    // PID calculation
    float p_term = kp * error[i];
    float i_term = ki * sum_error[i];
    float d_term = kd * (error[i] - last_error[i]);

    pid_output[i] = p_term + i_term + d_term;
    
    // Output saturation
    pid_output[i] = constrain(pid_output[i], -255, 255);

    last_error[i] = error[i];
  }
  
  setMotorPwm(pid_output);
}

void setMotorPwm(float pwm[4]) {
  // Set motor directions secara manual
  digitalWrite(M0_DIR_PIN, pwm[0] >= 0 ? HIGH : LOW);
  digitalWrite(M1_DIR_PIN, pwm[1] >= 0 ? HIGH : LOW);
  digitalWrite(M2_DIR_PIN, pwm[2] >= 0 ? HIGH : LOW);
  digitalWrite(M3_DIR_PIN, pwm[3] >= 0 ? HIGH : LOW);
     
  // Analog write secara manual untuk setiap motor
  analogWrite(M0_PWM_PIN, abs(pwm[0]));
  analogWrite(M1_PWM_PIN, abs(pwm[1]));
  analogWrite(M2_PWM_PIN, abs(pwm[2]));
  analogWrite(M3_PWM_PIN, abs(pwm[3]));

  // Serial.print("PWM Values: ");
  // Serial.print(abs(pwm[0])); Serial.print("\t");
  // Serial.print(abs(pwm[1])); Serial.print("\t");
  // Serial.print(abs(pwm[2])); Serial.print("\t");
  // Serial.print(abs(pwm[3])); Serial.println();
}

void setupEncoderInterrupts() {
  // Encoder pin setup
  pinMode(ENC0_A_PIN, INPUT);
  pinMode(ENC0_B_PIN, INPUT);
  pinMode(ENC1_A_PIN, INPUT);
  pinMode(ENC1_B_PIN, INPUT);
  pinMode(ENC2_A_PIN, INPUT);
  pinMode(ENC2_B_PIN, INPUT);
  pinMode(ENC3_A_PIN, INPUT);
  pinMode(ENC3_B_PIN, INPUT);

  // Use PinChangeInt for pin change interrupts
  PCattachInterrupt(ENC0_A_PIN, isrEnc0A, RISING);
  PCattachInterrupt(ENC1_A_PIN, isrEnc1A, RISING);
  PCattachInterrupt(ENC2_A_PIN, isrEnc2A, RISING);
  PCattachInterrupt(ENC3_A_PIN, isrEnc3A, RISING);
}

void setupMotorPins() {
  // Setup pin motor secara manual dan eksplisit
  pinMode(M0_PWM_PIN, OUTPUT);
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Modify PWM frequency (if needed)
  TCCR1B = (TCCR1B & 0xF8) | 0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B = (TCCR2B & 0xF8) | 0x01;    // Pin3,Pin11 PWM 31250Hz
}