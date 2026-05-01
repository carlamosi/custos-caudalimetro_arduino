/*
 *  CUSTOS — ESP32-C3 Firmware V3.1 (Bug-Free Production)
 *  ─────────────────────────────────────────────────────────
 *  Arquitectura:  Delta-Push Event-Driven (ahorro ~99% cuota Firebase)
 *  Reconexión:    WiFi State-Machine con backoff progresivo
 *  LED:           Parpadeo proporcional al caudal (sin bloqueos)
 *
 *  Changelog V3.1 vs V3.0:
 *    - FIX: LED_ON / LED_OFF centralizado (lógica invertida ESP32-C3 SuperMini)
 *    - FIX: Eliminado delay() dentro de gestionarWiFi() → reemplazado por millis()
 *    - FIX: NTP timezone corregido (Europe/Madrid: GMT+1/+2 con DST automático)
 *    - FIX: LED no se congela durante operaciones Firebase (timer dedicado)
 *    - FIX: Delta-check separado del cálculo para evitar doble disparo
 *    - OPT: Serial.printf() en lugar de concatenaciones de String (evita heap fragmentation)
 *    - OPT: Historial solo si hay caudal o anomalía activa (ahorra cuota extra)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <time.h>
#include <math.h>
#include <Wire.h>
#include <U8g2lib.h>

/* ═══════════════════════════════════════════════
   CONFIGURACIÓN — Editar solo esta sección
═══════════════════════════════════════════════ */
#define WIFI_SSID           "CasaWifi"
#define WIFI_PASS           "LuisMar1412"

#define FIREBASE_HOST       "custos-database-default-rtdb.europe-west1.firebasedatabase.app"
#define FIREBASE_AUTH       "gt0C2QtjtM688tUiNAVRU2MIdvy6hSqJs1jbWoBh"

// ── PINES ──
#define PIN_SENSOR          3   // Sensor de flujo (interrupción)
#define PIN_BTN_ANOMALIA    5   // Botón physical de anomalía
#define PIN_LED             7   // LED onboard ESP32-C3 SuperMini
#define PIN_OLED_SDA        8  // SDA para pantalla OLED I2C
#define PIN_OLED_SCL        9   // SCL para pantalla OLED I2C

// ── LÓGICA LED ESP32-C3 SuperMini (LED activo a nivel BAJO) ──
#define LED_ON              LOW
#define LED_OFF             HIGH

// ── CALIBRACIÓN SENSOR ──
#define PULSOS_POR_LITRO    450.0f  // Ajustar según modelo de sensor

// ── INTERVALOS (ms) ──
#define T_CALCULO           1000    // Frecuencia de cálculo de caudal
#define T_HEARTBEAT         30000   // Firebase heartbeat si no hay cambios
#define T_HISTORIAL         60000   // Guardar historial (ahorra cuota)
#define T_LED_BLINK         100     // Semiperíodo de parpadeo (100ms = 5Hz)
#define T_WIFI_TIMEOUT      10000   // Tiempo máximo esperando conexión
#define T_WIFI_BACKOFF_BASE 10000   // Espera base entre reintentos
#define T_WIFI_BACKOFF_MAX  60000   // Espera máxima entre reintentos
#define DELTA_LPM           0.05f   // Cambio mínimo para disparar telemetría

/* ═══════════════════════════════════════════════
   MÁQUINA DE ESTADOS WIFI
═══════════════════════════════════════════════ */
enum WifiState : uint8_t {
  WIFI_OK,          // Conectado y estable
  WIFI_CONNECTING,  // WiFi.begin() lanzado, esperando resultado
  WIFI_IDLE         // Desconectado, esperando el backoff para reintentar
};

/* ═══════════════════════════════════════════════
   OBJETOS FIREBASE Y PANTALLA OLED
═══════════════════════════════════════════════ */
FirebaseData   fbData;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

// Instancia global U8g2 para OLED 1.3" (Driver SH1106)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);

/* ═══════════════════════════════════════════════
   VARIABLES GLOBALES
═══════════════════════════════════════════════ */
// Sensor (acceso atómico desde ISR)
volatile uint32_t pulsosBrutos = 0;

// Métricas
float caudalLPM    = 0.0f;
float caudalLPS    = 0.0f;
float litrosSesion = 0.0f;
bool  anomaliaActiva = false;

// Delta (última vez que se envió a Firebase)
float lastLPM      = -999.0f;   // Valor imposible → primer envío siempre ocurre
bool  lastAnomalia = false;

// Temporizadores non-blocking
unsigned long tCalculo    = 0;
unsigned long tHeartbeat  = 0;
#define logo_width 32
#define logo_height 32
static const unsigned char logo_bits[] PROGMEM = {0};
unsigned long tHistorial  = 0;
unsigned long tLed        = 0;      // Timer del LED independiente
unsigned long tWifiAction = 0;      // Timer para acciones WiFi
unsigned long tPantalla   = 0;      // Timer para actualizar OLED
unsigned long tBoot       = 0;      // Timer para la animacion de boot

bool bootCompletado       = false;

// Estado LED
bool ledEncendido = false;

// Nuevo: Estado Toggle Botón
bool lastBtnState = LOW;
bool anomaliaBloqueada = false; // Estado interno del interruptor
unsigned long tLastBtn = 0;

// WiFi State Machine
WifiState wifiState       = WIFI_IDLE;
unsigned long wifiBackoff = T_WIFI_BACKOFF_BASE;
uint8_t wifiReintentos    = 0;

/* ═══════════════════════════════════════════════
   ISR — CONTADOR DE PULSOS (IRAM para velocidad)
═══════════════════════════════════════════════ */
void IRAM_ATTR ISR_Pulso() {
  pulsosBrutos++;
}

/* ═══════════════════════════════════════════════
   ENVIAR TELEMETRÍA A FIREBASE
═══════════════════════════════════════════════ */
void enviarFirebase() {
  if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;

  FirebaseJson json;
  json.set("caudal_lpm",     caudalLPM);
  json.set("litros_sesion",  litrosSesion);
  json.set("anomalia",       anomaliaActiva);
  json.set("estado",         anomaliaActiva ? "ANOMALIA" : (caudalLPM > 0.01f ? "MIDIENDO" : "PARADO"));
  json.set("timestamp_ms",   (uint32_t)millis());

  if (Firebase.updateNode(fbData, "/caudalimetro/tiempo_real", json)) {
    Serial.printf("  ↑ Firebase OK | LPM=%.2f | Ses=%.3f | Anomalia=%d\n",
                  caudalLPM, litrosSesion, (int)anomaliaActiva);
  } else {
    Serial.printf("  ✗ Firebase error: %s\n", fbData.errorReason().c_str());
  }
}

/* ═══════════════════════════════════════════════
   GUARDAR HISTORIAL (60s, ahorra cuota)
═══════════════════════════════════════════════ */
void guardarHistorial() {
  if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;

  time_t ts = time(nullptr);
  if (ts < 86400) return;  // NTP aún no está sincronizado

  // Solo guarda si hay actividad relevante (ahorra escrituras gratuitas)
  if (caudalLPM < 0.01f && !anomaliaActiva) return;

  FirebaseJson json;
  json.set("caudal_lpm",     roundf(caudalLPM * 100.0f) / 100.0f);
  json.set("litros_sesion",  roundf(litrosSesion * 1000.0f) / 1000.0f);
  json.set("anomalia",       anomaliaActiva);
  json.set("ts",             (uint32_t)ts);

  String ruta = "/caudalimetro/historial/" + String((uint32_t)ts);
  if (!Firebase.set(fbData, ruta, json)) {
    Serial.printf("  ✗ Historial error: %s\n", fbData.errorReason().c_str());
  }
}

/* ═══════════════════════════════════════════════
   GESTIÓN WIFI — State Machine (sin delay())
═══════════════════════════════════════════════ */
void gestionarWiFi(unsigned long ahora) {
  wl_status_t status = WiFi.status();

  switch (wifiState) {

    /* ── Conectado: vigilar caídas ── */
    case WIFI_OK:
      if (status != WL_CONNECTED) {
        Serial.println("\n⚠ WiFi perdido. Iniciando reconexión con backoff...");
        wifiState   = WIFI_IDLE;
        tWifiAction = ahora;   // Empezar backoff ahora mismo
      }
      break;

    /* ── Esperando backoff para reintentar ── */
    case WIFI_IDLE:
      if (ahora - tWifiAction >= wifiBackoff) {
        wifiReintentos++;
        Serial.printf("🔄 Reintento WiFi #%d (backoff=%lus)...\n",
                      wifiReintentos, wifiBackoff / 1000UL);

        WiFi.disconnect(true);  // Limpiar credenciales en driver
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        wifiState   = WIFI_CONNECTING;
        tWifiAction = ahora;    // Empezar a contar timeout de conexión
      }
      break;

    /* ── WiFi.begin() lanzado, esperando resultado ── */
    case WIFI_CONNECTING:
      if (status == WL_CONNECTED) {
        Serial.printf("✓ WiFi reconectado: %s (RSSI=%ddBm, intento #%d)\n",
                      WiFi.localIP().toString().c_str(),
                      WiFi.RSSI(), wifiReintentos);

        wifiState      = WIFI_OK;
        wifiBackoff    = T_WIFI_BACKOFF_BASE;  // Resetear backoff
        wifiReintentos = 0;

        // Resincronizar NTP tras reconexión
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
        tzset();

      } else if (ahora - tWifiAction >= T_WIFI_TIMEOUT) {
        // Timeout: este intento falló
        Serial.println("  ✗ Timeout de conexión. Ajustando backoff...");
        wifiBackoff = min((unsigned long)(wifiBackoff * 3 / 2), (unsigned long)T_WIFI_BACKOFF_MAX);
        wifiState   = WIFI_IDLE;
        tWifiAction = ahora;
      }
      break;
  }
}

/* ═══════════════════════════════════════════════
   LÓGICA PANTALLA OLED (U8G2)
═══════════════════════════════════════════════ */
void actualizarOLED() {
  if (!bootCompletado) {
    if (millis() - tBoot > 3500) {  // Mantener boot 3.5 segundos
      bootCompletado = true;
    } else {
      u8g2.clearBuffer();
      u8g2.setContrast(255); // Brillo máximo
      // Dibujar logo XBM a la izquierda
      u8g2.drawXBMP(5, 12, logo_width, logo_height, logo_bits);
      
      u8g2.setFont(u8g2_font_logisoso16_tr);
      u8g2.drawStr(42, 30, "CUSTOS");
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(50, 48, "V3.2 - IoT");
      u8g2.sendBuffer();
      return; 
    }
  }

  u8g2.clearBuffer();

  if (anomaliaActiva) {
    // ── PANTALLA ALERTA INVERTIDA ──
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 64); // Fondo todo encendido ("blanco")
    
    u8g2.setDrawColor(0); // Texto "negro" (agujeros en el fondo blanco)
    u8g2.setFont(u8g2_font_logisoso18_tr);
    
    // Parpadeo del texto "¡ALERTA!" a 2 Hz aprox.
    if ((millis() / 400) % 2 == 0) {
      u8g2.drawStr(5, 35, "¡ALERTA!");
    }
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(12, 55, "Flujo Anomalo Activo");
    u8g2.setDrawColor(1); // Restaurar a color blanco
  } 
  else if (caudalLPM > 0.01f) {
    // ── PANTALLA FLUJO ACTIVO ──
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "-> MIDIENDO...");

    // Animación visual estilo "disco girando" o "bombeando"
    if ((millis() / 200) % 2 == 0) u8g2.drawDisc(122, 6, 3);
    else u8g2.drawCircle(122, 6, 3);

    u8g2.setFont(u8g2_font_logisoso24_tr);
    char bufLPM[10];
    dtostrf(caudalLPM, 4, 1, bufLPM); 
    // Dibujamos en posición X fija (X=15 lo reserva bien para 4 chars)
    u8g2.drawStr(15, 42, bufLPM); 
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(85, 42, "L/min");

    char bufSes[24];
    dtostrf(litrosSesion, 4, 1, bufLPM);
    sprintf(bufSes, "Sesion: %s L", bufLPM);
    u8g2.drawStr(15, 60, bufSes);
  }
  else {
    // ── PANTALLA REPOSO ──
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (WiFi.status() == WL_CONNECTED) u8g2.drawStr(15, 10, "WiFi Conectado");
    else u8g2.drawStr(15, 10, "Buscando Red...");

    u8g2.setFont(u8g2_font_logisoso24_tr);
    u8g2.drawStr(15, 42, " 0.0"); 
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(85, 42, "L/min");

    u8g2.drawStr(15, 60, "Sistema listo");
  }

  u8g2.sendBuffer(); // Empujar imagen a la pantalla real
}

/* ═══════════════════════════════════════════════
   SETUP
═══════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(600);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  CUSTOS V3.1 — Bug-Free Production     ║");
  Serial.println("║  Delta-Push · LED-Aware · NTP-Safe     ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // ── Inicializar I2C y Pantalla OLED ──
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  u8g2.begin();

  // ── Configurar pines ──
  pinMode(PIN_SENSOR,       INPUT_PULLUP);
  pinMode(PIN_BTN_ANOMALIA, INPUT_PULLDOWN);
  pinMode(PIN_LED,          OUTPUT);
  digitalWrite(PIN_LED,     LED_OFF);  // Apagado al inicio

  // ── Interrupt del sensor de flujo ──
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), ISR_Pulso, RISING);

  // ── WiFi inicial ──
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);  // Nosotros gestionamos la reconexión
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando WiFi");

  // Parpadeo LED durante conexión (bloqueante solo aquí, en setup)
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 24) {
    delay(500);
    Serial.print(".");
    // Parpadeo lento: encendido 500ms / apagado 500ms
    digitalWrite(PIN_LED, intentos % 2 == 0 ? LED_ON : LED_OFF);
    intentos++;
  }
  digitalWrite(PIN_LED, LED_OFF);  // Apagar siempre al salir del bucle

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✓ WiFi OK: %s (RSSI=%ddBm)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    wifiState = WIFI_OK;

    // ── NTP: Timezone Europe/Madrid con DST automático ──
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);  // GMT+1 invierno GMT+2 verano
    tzset();
    Serial.println("⟳ NTP sincronizando en segundo plano...");
  } else {
    Serial.println("\n✗ WiFi offline en arranque — reconexión automática activa");
    wifiState   = WIFI_IDLE;
    tWifiAction = millis();
  }

  // ── Firebase ──
  fbConfig.host                        = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token  = FIREBASE_AUTH;
  fbConfig.timeout.serverResponse      = 5000;

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(false);  // ⚡ CRÍTICO: Desactivado — gestionamos nosotros

  // ── Inicializar timers ──
  unsigned long now = millis();
  tCalculo    = now;
  tHeartbeat  = now;
  tHistorial  = now;
  tLed        = now;
  tWifiAction = now;
  tPantalla   = now;
  tBoot       = now;

  Serial.println("\n── Sistema activo. Esperando flujo de agua... ──\n");
}

/* ═══════════════════════════════════════════════
   LOOP PRINCIPAL (Non-Blocking Total)
═══════════════════════════════════════════════ */
void loop() {
  unsigned long ahora = millis();

  /* ══ 1. WIFI STATE MACHINE ══ */
  gestionarWiFi(ahora);

  /* ══ 0. BOTÓN ANOMALÍA (TOGGLE 5V) ══ */
  bool currentBtn = (digitalRead(PIN_BTN_ANOMALIA) == HIGH);
  if (currentBtn && !lastBtnState && (ahora - tLastBtn > 300)) {
    // Detectamos el flanco de subida (cuando pasa de 0V a 5V)
    anomaliaBloqueada = !anomaliaBloqueada; 
    anomaliaActiva = anomaliaBloqueada;
    tLastBtn = ahora;
    Serial.printf("\n[!] BOTON PULSADO: Anomalía = %s\n", anomaliaActiva ? "ON" : "OFF");
  }
  lastBtnState = currentBtn;

  // Forzar estado si está bloqueado por el botón
  if (anomaliaBloqueada) anomaliaActiva = true;

  /* ══ 2. CÁLCULO DE CAUDAL (cada 1s) ══ */
  if (ahora - tCalculo >= T_CALCULO) {
    unsigned long delta = ahora - tCalculo;
    tCalculo = ahora;

    // Leer pulsos de forma atómica (sin corromper el contador por la ISR)
    noInterrupts();
    uint32_t pulsos = pulsosBrutos;
    pulsosBrutos = 0;
    interrupts();

    // Calcular caudal
    float segundos       = (float)delta / 1000.0f;
    float litrosPeriodo  = (float)pulsos / PULSOS_POR_LITRO;
    caudalLPS            = litrosPeriodo / segundos;
    caudalLPM            = caudalLPS * 60.0f;
    litrosSesion        += litrosPeriodo;

    Serial.printf("[%s] LPM=%.3f | Ses=%.4fL\n",
                  anomaliaActiva ? "ANOMALIA" : (caudalLPM > 0.01f ? "FLUJO" : "REPOSO"),
                  caudalLPM, litrosSesion);
  }

  /* ══ 3. TELEMETRÍA DELTA (Event-Driven) ══ */
  {
    bool cambioFlujo  = fabs(caudalLPM - lastLPM) > DELTA_LPM;
    bool cambioAlerta = anomaliaActiva != lastAnomalia;
    bool heartbeat    = ahora - tHeartbeat >= T_HEARTBEAT;

    if ((cambioFlujo || cambioAlerta || heartbeat) &&
        WiFi.status() == WL_CONNECTED &&
        Firebase.ready()) {

      enviarFirebase();
      lastLPM      = caudalLPM;
      lastAnomalia = anomaliaActiva;
      tHeartbeat   = ahora;
    }
  }

  /* ══ 4. HISTORIAL PERIÓDICO (60s) ══ */
  if (ahora - tHistorial >= T_HISTORIAL) {
    tHistorial = ahora;
    guardarHistorial();
  }

  /* ══ 5. LED — Parpadeo proporcional al flujo (timer independiente) ══
   *
   *  Sin agua  → LED apagado
   *  Agua leve (< 2 L/min) → Parpadeo lento (500ms ciclo)
   *  Agua media (2–8 L/min) → Parpadeo rápido (200ms ciclo)
   *  Agua intensa (> 8 L/min) → Parpadeo muy rápido (80ms ciclo)
   *  Anomalía → LED fijo encendido (señal de alerta permanente)
   */
  unsigned long ledPeriod;
  if (anomaliaActiva) {
    // Alerta: LED fijo ON (sin parpadeo)
    digitalWrite(PIN_LED, LED_ON);
  } else if (caudalLPM > 0.01f) {
    // Determinar velocidad de parpadeo según intensidad
    if      (caudalLPM > 8.0f) ledPeriod = 40;   // Muy rápido: 12.5 Hz
    else if (caudalLPM > 2.0f) ledPeriod = 100;  // Rápido: 5 Hz
    else                        ledPeriod = 250;  // Lento: 2 Hz

    if (ahora - tLed >= ledPeriod) {
      tLed = ahora;
      ledEncendido = !ledEncendido;
      digitalWrite(PIN_LED, ledEncendido ? LED_ON : LED_OFF);
    }
  } else {
    // Sin flujo: LED apagado, resetear estado
    digitalWrite(PIN_LED, LED_OFF);
    ledEncendido = false;
    tLed = ahora;  // Evitar parpadeo instantáneo al retomar flujo
  }

  /* ══ 6. ACTUALIZAR PANTALLA OLED ══ */
  if (ahora - tPantalla >= 250) {
    tPantalla = ahora;
    actualizarOLED();
  }

  /* ══ 7. YIELD — Evitar watchdog reset ══ */
  yield();
}
