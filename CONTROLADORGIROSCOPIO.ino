#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// =====================================================================
//  GIROSCOPIO PID DUAL (Roll X + Pitch Y) con sintonia por WiFi
// =====================================================================
//  - El ESP32 crea su propia red WiFi (AP). Conectate desde el celu/PC.
//  - Abri http://192.168.4.1 en el navegador para ver los sliders.
//  - Los valores se guardan en flash (Preferences) y persisten al apagar.
// =====================================================================

// --- PINES MOTOR 1 (Eje X / Roll) ---
const int pinENA = 14;
const int pinIN1 = 27;
const int pinIN2 = 26;

// --- PINES MOTOR 2 (Eje Y / Pitch) ---
const int pinENB = 13;
const int pinIN3 = 25;
const int pinIN4 = 33;

// --- MPU6050 ---
const int MPU_ADDR = 0x68;
float anguloX = 0.0;  // Roll
float anguloY = 0.0;  // Pitch

// --- WiFi AP ---
const char* AP_SSID = "GimbalTuner";
const char* AP_PASS = "12345678";   // min 8 caracteres
WebServer server(80);
Preferences prefs;

// --- PID 1 (X / Roll) ---
float Kp1 = 2.0, Ki1 = 0.0, Kd1 = 0.5;
float errorAnterior1 = 0.0;
float sumaIntegral1  = 0.0;

// --- PID 2 (Y / Pitch) ---
float Kp2 = 2.0, Ki2 = 0.0, Kd2 = 0.5;
float errorAnterior2 = 0.0;
float sumaIntegral2  = 0.0;

float posicionDeseada = 0.0;   // 0 grados = horizontal

// telemetria
volatile float ultimoU1 = 0.0;
volatile float ultimoU2 = 0.0;

long tiempoAnterior = 0;

// ---------------------------------------------------------------------
//  HTML embebido (servido como un solo PROGMEM string)
// ---------------------------------------------------------------------
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gimbal PID Tuner</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: system-ui, sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  h1 { font-size:20px; margin:0 0 12px; }
  .card { background:#1c1c1c; border:1px solid #333; border-radius:10px; padding:14px; margin-bottom:14px; }
  .row { display:flex; align-items:center; gap:10px; margin:8px 0; }
  .row label { width:34px; font-weight:600; color:#9cf; }
  .row input[type=range] { flex:1; }
  .row input[type=number] { width:90px; background:#222; color:#eee; border:1px solid #444; border-radius:6px; padding:4px 6px; }
  .axis-title { font-size:16px; font-weight:700; margin-bottom:6px; color:#fc6; }
  button { background:#2a6; color:#000; font-weight:700; border:0; border-radius:8px; padding:10px 16px; cursor:pointer; margin-right:8px; }
  button.alt { background:#48a; color:#fff; }
  .telem { font-family: ui-monospace, monospace; font-size:13px; color:#9f9; white-space:pre; }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
  @media (max-width:640px){ .grid{ grid-template-columns:1fr; } }
</style>
</head>
<body>
  <h1>Gimbal PID Tuner</h1>

  <div class="grid">
    <div class="card">
      <div class="axis-title">PID 1 &mdash; Eje X (Roll)</div>
      <div class="row"><label>Kp</label><input type="range" id="kp1_s" min="0" max="20" step="0.01"><input type="number" id="kp1_n" step="0.01"></div>
      <div class="row"><label>Ki</label><input type="range" id="ki1_s" min="0" max="5"  step="0.01"><input type="number" id="ki1_n" step="0.01"></div>
      <div class="row"><label>Kd</label><input type="range" id="kd1_s" min="0" max="5"  step="0.01"><input type="number" id="kd1_n" step="0.01"></div>
    </div>

    <div class="card">
      <div class="axis-title">PID 2 &mdash; Eje Y (Pitch)</div>
      <div class="row"><label>Kp</label><input type="range" id="kp2_s" min="0" max="20" step="0.01"><input type="number" id="kp2_n" step="0.01"></div>
      <div class="row"><label>Ki</label><input type="range" id="ki2_s" min="0" max="5"  step="0.01"><input type="number" id="ki2_n" step="0.01"></div>
      <div class="row"><label>Kd</label><input type="range" id="kd2_s" min="0" max="5"  step="0.01"><input type="number" id="kd2_n" step="0.01"></div>
    </div>
  </div>

  <div class="card">
    <button onclick="saveFlash()">Guardar en flash</button>
    <button class="alt" onclick="resetInt()">Reset integrales</button>
  </div>

  <div class="card">
    <div class="axis-title">Telemetria</div>
    <div class="telem" id="telem">cargando...</div>
  </div>

<script>
const ids = ["kp1","ki1","kd1","kp2","ki2","kd2"];

function bindPair(name){
  const s = document.getElementById(name+"_s");
  const n = document.getElementById(name+"_n");
  const send = ()=>{ fetch("/set?"+name+"="+encodeURIComponent(n.value)); };
  s.addEventListener("input", ()=>{ n.value = s.value; send(); });
  n.addEventListener("change", ()=>{ s.value = n.value; send(); });
}
ids.forEach(bindPair);

async function loadGains(){
  const r = await fetch("/gains"); const j = await r.json();
  for (const k of ids){
    document.getElementById(k+"_s").value = j[k];
    document.getElementById(k+"_n").value = j[k];
  }
}
async function poll(){
  try {
    const r = await fetch("/state"); const j = await r.json();
    document.getElementById("telem").textContent =
      "angX = " + j.angX.toFixed(2) + " deg\n" +
      "angY = " + j.angY.toFixed(2) + " deg\n" +
      "U1   = " + j.u1.toFixed(2)   + "\n" +
      "U2   = " + j.u2.toFixed(2);
  } catch(e) {}
}
function saveFlash(){ fetch("/save", {method:"POST"}).then(()=>alert("Guardado")); }
function resetInt(){ fetch("/resetI", {method:"POST"}); }

loadGains();
setInterval(poll, 200);
</script>
</body>
</html>
)HTML";

// ---------------------------------------------------------------------
//  Persistencia
// ---------------------------------------------------------------------
void cargarGanancias() {
  prefs.begin("pid", true);  // read-only
  Kp1 = prefs.getFloat("kp1", Kp1);
  Ki1 = prefs.getFloat("ki1", Ki1);
  Kd1 = prefs.getFloat("kd1", Kd1);
  Kp2 = prefs.getFloat("kp2", Kp2);
  Ki2 = prefs.getFloat("ki2", Ki2);
  Kd2 = prefs.getFloat("kd2", Kd2);
  prefs.end();
}

void guardarGanancias() {
  prefs.begin("pid", false);
  prefs.putFloat("kp1", Kp1);
  prefs.putFloat("ki1", Ki1);
  prefs.putFloat("kd1", Kd1);
  prefs.putFloat("kp2", Kp2);
  prefs.putFloat("ki2", Ki2);
  prefs.putFloat("kd2", Kd2);
  prefs.end();
}

// ---------------------------------------------------------------------
//  Handlers HTTP
// ---------------------------------------------------------------------
void handleIndex() {
  server.send_P(200, "text/html; charset=utf-8", PAGE_INDEX);
}

void handleGains() {
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"kp1\":%.3f,\"ki1\":%.3f,\"kd1\":%.3f,\"kp2\":%.3f,\"ki2\":%.3f,\"kd2\":%.3f}",
    Kp1, Ki1, Kd1, Kp2, Ki2, Kd2);
  server.send(200, "application/json", buf);
}

void handleState() {
  char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"angX\":%.2f,\"angY\":%.2f,\"u1\":%.2f,\"u2\":%.2f}",
    anguloX, anguloY, ultimoU1, ultimoU2);
  server.send(200, "application/json", buf);
}

void handleSet() {
  auto getF = [&](const char* k, float& dst){
    if (server.hasArg(k)) {
      float v = server.arg(k).toFloat();
      if (v < 0) v = 0;
      dst = v;
    }
  };
  getF("kp1", Kp1); getF("ki1", Ki1); getF("kd1", Kd1);
  getF("kp2", Kp2); getF("ki2", Ki2); getF("kd2", Kd2);
  server.send(200, "text/plain", "ok");
}

void handleSave() {
  guardarGanancias();
  server.send(200, "text/plain", "saved");
}

void handleResetI() {
  sumaIntegral1 = 0;
  sumaIntegral2 = 0;
  server.send(200, "text/plain", "ok");
}

// ---------------------------------------------------------------------
//  setup
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motores
  pinMode(pinENA, OUTPUT); pinMode(pinIN1, OUTPUT); pinMode(pinIN2, OUTPUT);
  pinMode(pinENB, OUTPUT); pinMode(pinIN3, OUTPUT); pinMode(pinIN4, OUTPUT);

  // MPU6050
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Calibrando... Manten el sensor quieto.");
  delay(2000);

  // Cargar ganancias guardadas
  cargarGanancias();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP listo. SSID: "); Serial.print(AP_SSID);
  Serial.print("  IP: ");           Serial.println(WiFi.softAPIP());

  // Rutas HTTP
  server.on("/",        HTTP_GET,  handleIndex);
  server.on("/gains",   HTTP_GET,  handleGains);
  server.on("/state",   HTTP_GET,  handleState);
  server.on("/set",     HTTP_GET,  handleSet);
  server.on("/save",    HTTP_POST, handleSave);
  server.on("/resetI",  HTTP_POST, handleResetI);
  server.begin();

  tiempoAnterior = micros();
  Serial.println("Gimbal dual activado!");
}

// ---------------------------------------------------------------------
//  Driver de motor con signo
// ---------------------------------------------------------------------
void driveMotor(float u, int pinIN_A, int pinIN_B, int pinEN) {
  int pwm = (int) fabsf(u);
  if (pwm > 255) pwm = 255;
  if (u > 0)      { digitalWrite(pinIN_A, HIGH); digitalWrite(pinIN_B, LOW);  }
  else if (u < 0) { digitalWrite(pinIN_A, LOW);  digitalWrite(pinIN_B, HIGH); }
  else            { digitalWrite(pinIN_A, LOW);  digitalWrite(pinIN_B, LOW);  }
  analogWrite(pinEN, pwm);
}

// ---------------------------------------------------------------------
//  loop
// ---------------------------------------------------------------------
void loop() {
  server.handleClient();

  long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  if (dt <= 0.0) return;

  // Leer MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t accX  = Wire.read() << 8 | Wire.read();
  int16_t accY  = Wire.read() << 8 | Wire.read();
  int16_t accZ  = Wire.read() << 8 | Wire.read();
  int16_t temp  = Wire.read() << 8 | Wire.read();
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  (void)temp; (void)gyroZ;

  // Angulos por filtro complementario
  float accAngleX = (atan2((float)accY, (float)accZ) * 180.0) / PI;          // Roll
  float accAngleY = (atan2(-(float)accX, (float)accZ) * 180.0) / PI;         // Pitch
  float gyroRateX = gyroX / 131.0;
  float gyroRateY = gyroY / 131.0;
  anguloX = 0.98 * (anguloX + gyroRateX * dt) + 0.02 * accAngleX;
  anguloY = 0.98 * (anguloY + gyroRateY * dt) + 0.02 * accAngleY;

  // ---- PID 1 (X) ----
  float error1 = posicionDeseada - anguloX;
  sumaIntegral1 += error1 * dt;
  if (sumaIntegral1 >  100) sumaIntegral1 =  100;
  if (sumaIntegral1 < -100) sumaIntegral1 = -100;
  float deriv1 = (error1 - errorAnterior1) / dt;
  float u1 = Kp1 * error1 + Ki1 * sumaIntegral1 + Kd1 * deriv1;
  errorAnterior1 = error1;
  ultimoU1 = u1;

  // ---- PID 2 (Y) ----
  float error2 = posicionDeseada - anguloY;
  sumaIntegral2 += error2 * dt;
  if (sumaIntegral2 >  100) sumaIntegral2 =  100;
  if (sumaIntegral2 < -100) sumaIntegral2 = -100;
  float deriv2 = (error2 - errorAnterior2) / dt;
  float u2 = Kp2 * error2 + Ki2 * sumaIntegral2 + Kd2 * deriv2;
  errorAnterior2 = error2;
  ultimoU2 = u2;

  // Salida a motores
  driveMotor(u1, pinIN1, pinIN2, pinENA);
  driveMotor(u2, pinIN3, pinIN4, pinENB);

  tiempoAnterior = tiempoActual;
  delay(2);
}
