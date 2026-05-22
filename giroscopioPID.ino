#include <Wire.h>

//PINES MOTOR 1 (ROLL)
const int pinENA = 14; 
const int pinIN1 = 27; 
const int pinIN2 = 26; 

//PINES MOTOR 2 (PITCH)
const int pinENB = 32; 
const int pinIN3 = 33; 
const int pinIN4 = 25; 

//VARIABLES DEL giroscopio
const int MPU_ADDR = 0x68;
float anguloX = 0.0; 
float anguloY = 0.0; 

//VARIABLES PID EJE X (ROLL)
float posicionDeseadaX = 1; //esta la cambie de 0 a 1 
float Kp_X = 15.0;  
float Ki_X = 0.0;   
float Kd_X = 0.5;   
float errorAnteriorX = 0.0;
float sumaIntegralX = 0.0;
float derivadaFiltradaX = 0.0;

//VARIABLES PID EJE Y (PITCH)
float posicionDeseadaY = 0.0; 
float Kp_Y = 15.0;  
float Ki_Y = 0.0;   
float Kd_Y = 0.5;   
float errorAnteriorY = 0.0;
float sumaIntegralY = 0.0;
float derivadaFiltradaY = 0.0;

// Memoria de tiempo
long tiempoAnterior = 0;

void setup() {
  Serial.begin(115200);

  //Setup Motores
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  
  pinMode(pinENB, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);

  // Setup giroscopio
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Serial.println("Calibrando 2 Ejes... Mantén el sensor quieto.");
  //delay(2000); 

  tiempoAnterior = micros();
  //Serial.println("GIMBAL_READY");
}

void procesarComando() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      // Formato: X Kp Ki Kd  o  Y Kp Ki Kd
      char eje = buffer.charAt(0);
      int i1 = buffer.indexOf(' ');
      int i2 = buffer.indexOf(' ', i1 + 1);
      if (eje == 'X' || eje == 'Y') {
        float kp = buffer.substring(i1 + 1, i2).toFloat();
        float ki = buffer.substring(i2 + 1).toFloat();
        // Buscar tercer espacio para Kd
        int i3 = buffer.indexOf(' ', i2 + 1);
        float kd = 0;
        if (i3 > 0) {
          ki = buffer.substring(i2 + 1, i3).toFloat();
          kd = buffer.substring(i3 + 1).toFloat();
        }
        if (eje == 'X') {
          Kp_X = kp; Ki_X = ki; Kd_X = kd;
        } else {
          Kp_Y = kp; Ki_Y = ki; Kd_Y = kd;
        }
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

unsigned long lastSerialSend = 0;

void loop() {
  procesarComando();

  // 1. CALCULAR TIEMPO (dt)
  long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  if (dt < 0.002) dt = 0.002;

  // 2. LEER MPU6050 (EN ORDEN ESTRICTO)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 

  int16_t accX  = Wire.read() << 8 | Wire.read();
  int16_t accY  = Wire.read() << 8 | Wire.read();
  int16_t accZ  = Wire.read() << 8 | Wire.read();
  int16_t temp  = Wire.read() << 8 | Wire.read(); // Ignorado
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read(); // Ignorado

  
  // 3. MATEMÁTICAS Y PID: EJE X (ROLL)
  float accAngleX = -(atan2(accY, accZ) * 180.0) / PI; // INVERTIDO
  float gyroRateX = -(gyroX / 131.0);                  // INVERTIDO
  
  anguloX = 0.98 * (anguloX + gyroRateX * dt) + 0.02 * accAngleX;

  float errorX = posicionDeseadaX - anguloX;
  sumaIntegralX = constrain(sumaIntegralX + (errorX * dt), -100, 100);
  float derivadaCrudaX = (errorX - errorAnteriorX) / dt;
  derivadaFiltradaX = 0.7 * derivadaFiltradaX + 0.3 * derivadaCrudaX;
  derivadaFiltradaX = constrain(derivadaFiltradaX, -150, 150);
  float uX = (Kp_X * errorX) + (Ki_X * sumaIntegralX) + (Kd_X * derivadaFiltradaX);
  if (fabs(uX) < 5) uX = 0;

  int pwmX = constrain(abs(uX), 0, 255); 

  if (uX > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (uX < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }
  analogWrite(pinENA, pwmX);

  
  // 4. MATEMÁTICAS Y PID: EJE Y (PITCH)
  float accAngleY = (atan2(accX, accZ) * 180.0) / PI; // INVERTIDO
  float gyroRateY = -(gyroY / 131.0);                  // INVERTIDO
  
  anguloY = 0.98 * (anguloY + gyroRateY * dt) + 0.02 * accAngleY;

  float errorY = posicionDeseadaY - anguloY;
  sumaIntegralY = constrain(sumaIntegralY + (errorY * dt), -100, 100);
  float derivadaCrudaY = (errorY - errorAnteriorY) / dt;
  derivadaFiltradaY = 0.7 * derivadaFiltradaY + 0.3 * derivadaCrudaY;
  derivadaFiltradaY = constrain(derivadaFiltradaY, -150, 150);
  float uY = (Kp_Y * errorY) + (Ki_Y * sumaIntegralY) + (Kd_Y * derivadaFiltradaY);
  if (fabs(uY) < 5) uY = 0;

  int pwmY = constrain(abs(uY), 0, 255); 

  if (uY > 0) {
    digitalWrite(pinIN3, HIGH);
    digitalWrite(pinIN4, LOW);
  } else if (uY < 0) {
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, HIGH);
  } else {
    digitalWrite(pinIN3, LOW);
    digitalWrite(pinIN4, LOW);
  }
  analogWrite(pinENB, pwmY);

  
  // 5. GUARDAR DATOS Y FIN DEL CICLO
  errorAnteriorX = errorX;
  errorAnteriorY = errorY;
  tiempoAnterior = tiempoActual;

  // Enviar datos a Python cada ~50ms
  if (millis() - lastSerialSend > 40) {
    Serial.print("X:");
    Serial.print(anguloX);
    Serial.print(",Y:");
    Serial.print(anguloY);
    Serial.print(",PWMX:");
    Serial.print(pwmX);
    Serial.print(",PWMY:");
    Serial.print(pwmY);
    Serial.print(",SPX:");
    Serial.print(posicionDeseadaX);
    Serial.print(",SPY:");
    Serial.print(posicionDeseadaY);
    Serial.println();
    lastSerialSend = millis();
  }

  delay(4);
}
