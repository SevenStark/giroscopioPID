#include <Wire.h>

// --- PINES DEL MOTOR (L298N) ---
const int pinENA = 14; 
const int pinIN1 = 27; 
const int pinIN2 = 26; 

// --- VARIABLES DEL MPU6050 ---
const int MPU_ADDR = 0x68;
float anguloX = 0.0; 

// --- VARIABLES DEL CONTROLADOR PID ---
float posicionDeseada = 0.0; // Queremos mantener la cámara a 0 grados (Horizontal)

// VALORES PID PARA ÁNGULOS (Tendrás que volver a sintonizar)
// Sugerencia de arranque: Kp alto, Ki apagado, Kd mediano
float Kp = 2.0;  // Fuerza para corregir el ángulo
float Ki = 0.0;   // Apagado para empezar
float Kd = 0.5;   // Freno para evitar que vibre al llegar a 0

// Memoria del PID
float errorAnterior = 0.0;
long tiempoAnterior = 0;
float sumaIntegral = 0.0;

void setup() {
  Serial.begin(115200);

  // Setup Motor
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);

  // Setup MPU6050
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Calibrando... Mantén el sensor quieto.");
  delay(2000); // Dar tiempo a estabilizar

  tiempoAnterior = micros();
  Serial.println("¡Gimbal Activado!");
}

void loop() {
  // 1. CALCULAR TIEMPO (dt)
  long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  if (dt <= 0.0) return;

  // 2. LEER MPU6050 (EN ORDEN ESTRICTO)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Registro de inicio (0x3B = Acelerómetro X)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Pedimos los 14 bytes

  // Leemos los 7 valores (2 bytes cada uno) en su orden exacto de fábrica:
  int16_t accX  = Wire.read() << 8 | Wire.read();
  int16_t accY  = Wire.read() << 8 | Wire.read();
  int16_t accZ  = Wire.read() << 8 | Wire.read();
  int16_t temp  = Wire.read() << 8 | Wire.read();
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  // 3. MATEMÁTICAS DEL EJE X (ROLL)
  // atan2 con Y y Z nos da la rotación sobre el eje X.
  float accAngleX = (atan2(accY, accZ) * 180.0) / PI;
  float gyroRateX = gyroX / 131.0;
  
  // Filtro Complementario sano
  anguloX = 0.98 * (anguloX + gyroRateX * dt) + 0.02 * accAngleX;

  // 4. MATEMÁTICAS DEL PID
  float posicionActual = anguloX;
  float error = posicionDeseada - posicionActual;

  sumaIntegral = sumaIntegral + (error * dt);
  if (sumaIntegral > 100) sumaIntegral = 100;
  if (sumaIntegral < -100) sumaIntegral = -100;

  float derivada = (error - errorAnterior) / dt;

  // Señal de control U
  float u = (Kp * error) + (Ki * sumaIntegral) + (Kd * derivada);

  // 5. MOVER EL MOTOR
  int velocidadPWM = abs(u);
  if (velocidadPWM > 255) velocidadPWM = 255; 

  if (u > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (u < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }

  analogWrite(pinENA, velocidadPWM);

  // 6. GUARDAR DATOS PARA EL SIGUIENTE CICLO
  errorAnterior = error;
  tiempoAnterior = tiempoActual;

  delay(2);
}