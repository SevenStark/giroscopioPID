# Giroscopio PID Dual

Sistema de estabilización tipo *gimbal* de 2 ejes basado en **ESP32**, **MPU6050** y **L298N**, con sintonización de las ganancias PID en tiempo real desde servida por el propio ESP32. **No hace falta recompilar para cambiar `Kp`, `Ki` o `Kd`.**

---

## ¿Qué hace?

- Lee la inclinación con un MPU6050 (acelerómetro + giroscopio).
- Fusiona los datos con un **filtro complementario** (98% giro / 2% acel) para obtener los ángulos **Roll (X)** y **Pitch (Y)**.
- Aplica **dos controladores PID independientes**, uno por eje, para mantener la plataforma horizontal (0°).
- Saca la señal de control como PWM con signo a dos canales de un **L298N** (un motor DC por eje).
- Levanta una red WiFi propia (modo AP) y sirve una **interfaz web** con sliders para tunear las 6 ganancias en vivo.
- Guarda las ganancias en la **memoria flash** del ESP32 (`Preferences`), así sobreviven a un reinicio.

---

## Hardware

| Componente   | Detalle                                              |
|--------------|------------------------------------------------------|
| MCU          | ESP32 (cualquier DevKit con WiFi)                    |
| IMU          | MPU6050 (I²C)                                        |
| Driver       | L298N (los 2 canales: uno por eje)                   |
| Motores      | 2 × motor DC (uno por eje del gimbal)                |
| Alimentación | La del L298N para los motores + 5V/USB para el ESP32 |

### Conexionado

**MPU6050 (I²C):**
| MPU6050 | ESP32   |
|---------|---------|
| VCC     | 3V3     |
| GND     | GND     |
| SDA     | GPIO 21 |
| SCL     | GPIO 22 |

**Motor 1 — Eje X / Roll (canal A del L298N):**
| L298N | ESP32   |
|-------|---------|
| ENA   | GPIO 14 |
| IN1   | GPIO 27 |
| IN2   | GPIO 26 |

**Motor 2 — Eje Y / Pitch (canal B del L298N):**
| L298N | ESP32   |
|-------|---------|
| ENB   | GPIO 13 |
| IN3   | GPIO 25 |
| IN4   | GPIO 33 |

> ⚠️ **Importante:** sacá los jumpers de ENA y ENB del L298N para que el PWM funcione. Conectá GND del L298N con GND del ESP32 (masa común).

---

## Software / Dependencias

Todo lo necesario viene incluido en el **ESP32 Arduino core**, no hay librerías extra que instalar:

- `Wire.h`
- `WiFi.h`
- `WebServer.h`
- `Preferences.h`

En el Arduino IDE: instalar el board *ESP32 by Espressif Systems* y seleccionar tu placa (p. ej. `ESP32 Dev Module`).

---

## Cómo usarlo

1. **Compilá y subí** `CONTROLADORGIROSCOPIO.ino` al ESP32.
2. Abrí el **Monitor Serie** a `115200 baudios`. Vas a ver algo así:
   ```
   Calibrando... Manten el sensor quieto.
   AP listo. SSID: GimbalTuner  IP: 192.168.4.1
   Gimbal dual activado!
   ```
3. Desde el celular o la PC, conectate a la red WiFi:
   - **SSID:** `GimbalTuner`
   - **Password:** `12345678`
4. Abrí el navegador en **http://192.168.4.1**.
5. Ajustá las ganancias con los sliders o los inputs numéricos — los cambios se aplican **al instante**.
6. Cuando quede lindo, apretá **"Guardar en flash"** para que persistan al apagar.

---

## La interfaz web

La página muestra:

- **PID 1 — Eje X (Roll):** sliders + inputs para `Kp`, `Ki`, `Kd`.
- **PID 2 — Eje Y (Pitch):** ídem.
- **Guardar en flash:** persiste las 6 ganancias en `Preferences`.
- **Reset integrales:** pone en cero `sumaIntegral1` y `sumaIntegral2` (útil después de un cambio grande de `Ki`, para que no se vaya de mambo el *windup*).
- **Telemetría en vivo** (refresca cada 200 ms):
  - `angX` — ángulo Roll (°)
  - `angY` — ángulo Pitch (°)
  - `U1` — salida del PID 1
  - `U2` — salida del PID 2

### Rangos por defecto en los sliders

| Ganancia | Mínimo | Máximo | Paso |
|----------|--------|--------|------|
| Kp       | 0      | 20     | 0.01 |
| Ki       | 0      | 5      | 0.01 |
| Kd       | 0      | 5      | 0.01 |

Si necesitás un valor fuera de rango, escribilo directo en el cuadro numérico al lado del slider.

---

## API HTTP (por si querés integrar otra cosa)

| Método | Ruta       | Qué hace                                                        |
|--------|------------|-----------------------------------------------------------------|
| GET    | `/`        | Devuelve la página HTML.                                        |
| GET    | `/gains`   | JSON con las 6 ganancias actuales.                              |
| GET    | `/state`   | JSON con `angX`, `angY`, `u1`, `u2`.                            |
| GET    | `/set?kp1=…&ki1=…&kd1=…&kp2=…&ki2=…&kd2=…` | Actualiza una o varias ganancias en vivo. |
| POST   | `/save`    | Guarda las ganancias actuales en flash.                         |
| POST   | `/resetI`  | Pone en cero las integrales de los dos PIDs.                    |

Ejemplo desde la terminal:
```
curl "http://192.168.4.1/set?kp1=3.5&kd1=0.8"
curl -X POST http://192.168.4.1/save
```

---

## Guía rápida de sintonización

1. Empezá con `Ki = 0` en los dos ejes.
2. Subí `Kp` hasta que la plataforma reaccione fuerte pero **oscile** un poco.
3. Subí `Kd` hasta que las oscilaciones se calmen (el `Kd` actúa como freno).
4. Si queda un error en estado estacionario (no llega a 0°), agregá un toquecito de `Ki`. Si se descontrola, apretá **"Reset integrales"**.
5. Cuando los dos ejes queden firmes, **"Guardar en flash"**.

> Tip: si un eje empuja para el lado contrario, dale vuelta los cables del motor en el L298N **o** intercambiá `IN3` ↔ `IN4` (o `IN1` ↔ `IN2`) en el código.

---

## Estructura del proyecto

```
giroscopioPID/
├── CONTROLADORGIROSCOPIO.ino   ← firmware del ESP32
└── README.md                    ← este archivo
```

---

## Estado

> *"Funciona en un 8%"* — versión inicial. La parte de control dual + interfaz web ya está armada; queda afinar las ganancias en hardware real y ajustar los signos de los motores según cómo quede el armado mecánico del gimbal.
