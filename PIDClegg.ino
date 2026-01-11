/*
 * PROYECTO: Helicóptero 1-DOF - FINAL V10 (ZONA SEGURA)
 * MEJORA: Se añade una "Zona de Estabilidad" de 0.3 grados.
 * Si el error es menor a 0.3, NO se resetea la integral
 * para evitar caídas bruscas cuando ya está flotando estable.
 */

#include <Wire.h>
#include <Arduino.h>

// --- ESTADO ---
enum ControlMode { MANUAL_PWM, AUTO_PID };
ControlMode currentMode = MANUAL_PWM; 

// --- VARIABLES PID ---
float Kp = 1.2;   
float Ki = 0.5;   
float Kd = 0.15; 

// --- PARAMETROS CLEGG & ZONA SEGURA ---
float clegg_factor = 0.995;   // Factor de conservación (99.5%)
float umbral_estabilidad = 0.3; // GRADOS: Si el error es menor a esto, NO reseteamos.

float target_value = 0.0; 
float current_pwm = 0.0;

// --- HARDWARE ---
#define SDA_PIN 8
#define SCL_PIN 9
#define MOTOR_PIN 4
#define MPU_ADDR 0x68

const int frecuencia = 450; 
const int resolucion = 14; 
const int maxValor = 16383; 
const float MIN_DUTY_REAL = 35.0; 
const float MAX_DUTY_REAL = 92.0; 

#define SAMPLE_TIME_US 10000 
#define DT 0.01              
hw_timer_t *timer = NULL;
volatile bool trigger_sample = false;

// IMU
int16_t ax, ay, az, gx, gy, gz;
float gyro_offset_y = 0.0;
float angle_offset_pitch = 0.0; 
float angle_pitch = 0.0;

// VARIABLES PID POSICIONAL
float e_k = 0.0;   
float e_k1 = 0.0;  
float integral_sum = 0.0; 
float u_k = 0.0;   

// KALMAN
class SimpleKalmanFilter {
  private:
    float _err_measure, _err_estimate, _q, _current_estimate = 0, _last_estimate = 0, _kalman_gain = 0;
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) {
      _err_measure = mea_e; _err_estimate = est_e; _q = q;
    }
    float updateEstimate(float mea, float rate, float dt) {
      _current_estimate = _last_estimate + rate * dt;
      _err_estimate = _err_estimate + _q * dt;
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _current_estimate + _kalman_gain * (mea - _current_estimate);
      _err_estimate = (1.0 - _kalman_gain) * _err_estimate;
      _last_estimate = _current_estimate;
      return _current_estimate;
    }
    void setAngle(float angle) { _last_estimate = angle; }
};
SimpleKalmanFilter kalman(10.0, 2.0, 0.01); 

void IRAM_ATTR onTimer() { trigger_sample = true; }

void computePID_SmartClegg(); // <--- NOMBRE ACTUALIZADO
void runMotor(float pwm_val);
void readIMU();
void writeReg(uint8_t reg, uint8_t data);
void parseCommand(String cmd);

void setup() {
  Serial.begin(115200);
  delay(100); 
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 

  ledcAttach(MOTOR_PIN, frecuencia, resolucion);
  ledcWrite(MOTOR_PIN, 0); 

  writeReg(0x6B, 0x00); delay(50);
  writeReg(0x1A, 0x03); 
  writeReg(0x1B, 0x00); 
  writeReg(0x1C, 0x10); 
  
  Serial.println("CALIBRANDO..."); 
  long sum_gyro = 0; double sum_acc = 0.0;
  for (int i = 0; i < 1000; i++) {
    readIMU();
    sum_gyro += gy;
    sum_acc += atan2(ax, az) * 180.0 / PI; 
    delay(2);
  }
  gyro_offset_y = sum_gyro / 1000.0;
  angle_offset_pitch = sum_acc / 1000.0; 
  kalman.setAngle(0.0);
  Serial.println("LISTO.");

  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, SAMPLE_TIME_US, true, 0); 
}

void loop() {
  if (Serial.available() > 0) {
    String comandoCompleto = Serial.readStringUntil('\n');
    parseCommand(comandoCompleto);
  }

  if (trigger_sample) {
    trigger_sample = false; 
    readIMU();
    
    float acc_raw = (atan2(ax, az) * 180.0 / PI) - angle_offset_pitch;
    float gyro_rate = -((gy - gyro_offset_y) / 131.0); 
    angle_pitch = kalman.updateEstimate(acc_raw, gyro_rate, DT);

    if (currentMode == MANUAL_PWM) {
        current_pwm = target_value;
        runMotor(current_pwm);
        integral_sum = 0.0; 
        e_k1 = 0.0;
    } 
    else {
        computePID_SmartClegg(); 
    }

    Serial.printf("%lu,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f\n", 
                  millis(), target_value, angle_pitch, current_pwm, (int)currentMode, Kp, Ki, Kd);   
  }
}

void parseCommand(String cmd) {
  cmd.trim(); if (cmd.length() < 2) return;
  char type = cmd.charAt(0);
  float val = cmd.substring(1).toFloat();

  if (type == 'M') { 
      if (val == 0) { 
        currentMode = MANUAL_PWM; target_value = 0; 
      } 
      else { 
        currentMode = AUTO_PID; target_value = 0; integral_sum = 0; 
      }
  }
  else if (type == 'V') { 
      if (val == -1.0) { target_value = 0; runMotor(0); integral_sum = 0; } 
      else { target_value = val; }
  }
  else if (type == 'P') Kp = val;
  else if (type == 'I') Ki = val;
  else if (type == 'D') Kd = val;
}

/* --- PID INTELIGENTE: CLEGG CON BANDA DE ESTABILIDAD --- */
void computePID_SmartClegg() {
  // 1. Calcular Error
  e_k = target_value - angle_pitch;
  
  float term_P = Kp * e_k;
  float term_D = (Kd / DT) * (e_k - e_k1);

  // 2. DETECCIÓN DE CRUCE + ZONA SEGURA
  bool cruce_por_cero = ((e_k > 0 && e_k1 < 0) || (e_k < 0 && e_k1 > 0));
  
  // VERIFICAMOS SI VALE LA PENA RESETEAR
  // Si el error anterior era MAYOR a 0.3, significa que veníamos rápido o de lejos -> SI BORRAMOS
  // Si el error anterior era MENOR a 0.3, significa que ya estábamos estables -> NO BORRAMOS
  if (cruce_por_cero && abs(e_k1) > umbral_estabilidad) {
      integral_sum = integral_sum * clegg_factor; 
  } 
  else {
      // 3. INTEGRACIÓN NORMAL + ANTI-WINDUP
      float i_increment = (Ki * DT) * e_k;
      float u_predicted = term_P + (integral_sum + i_increment) + term_D;
      
      bool saturated_high = (u_predicted > 100.0);
      bool saturated_low  = (u_predicted < 0.0);
      
      if (!saturated_high && !saturated_low) {
          integral_sum += i_increment;
      }
  }

  // 4. Salida Final
  float u_new = term_P + integral_sum + term_D;

  if (u_new > 100.0) u_new = 100.0;
  if (u_new < 0.0) u_new = 0.0;
  
  u_k = u_new;
  e_k1 = e_k;

  current_pwm = u_k;
  runMotor(current_pwm);
}

void runMotor(float val_percent) {
  float dutyReal = 0.0;
  if (val_percent < 1.0) dutyReal = 0.0;
  else dutyReal = MIN_DUTY_REAL + (val_percent * ((MAX_DUTY_REAL - MIN_DUTY_REAL) / 100.0));
  
  if (dutyReal > 100) dutyReal = 100;
  int dutyTicks = (int)((dutyReal / 100.0) * maxValor);
  ledcWrite(MOTOR_PIN, dutyTicks);
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = Wire.read()<<8|Wire.read(); ay = Wire.read()<<8|Wire.read(); az = Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read(); 
  gx = Wire.read()<<8|Wire.read(); gy = Wire.read()<<8|Wire.read(); gz = Wire.read()<<8|Wire.read();
}
void writeReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(data); Wire.endTransmission(true);
}