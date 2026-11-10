/*
 * Detector de gases con semáforo analógico (ESP32)
 * Sensores : DHT11, MQ-2 (gases combustibles), MQ-7 (monóxido de carbono)
 * Salidas  : módulo relé/buzzer + LED RGB (ánodo común a 3.3V)
 * Comunica : MQTT (Mosquitto) -> Node-RED
 *
 * CAMBIOS respecto a la versión anterior:
 *  - LED RGB de ánodo común (R=GPIO15, G=GPIO2, B=GPIO16, común a 3.3V):
 *    verde = normal, amarillo = precaución, rojo = peligro, azul = calentando.
 *  - La calibración (R0) ya no se hace a los 60 s: se espera CALIBRACION_MS (3 min)
 *    porque a los 60 s el MQ-7 todavía no estabilizó y guardaba un R0 incorrecto.
 *  - RESETEAR_CALIBRACION borra el R0 guardado en flash (que quedó mal) al arrancar.
 *  - Se publican los mV crudos (control/co_mv y control/gas_mv) y se muestran por Serie.
 *  - Los sensores se leen cada 1 s (la alarma es inmediata), pero se publica a MQTT
 *    (y por lo tanto se guarda en la base) cada 5 min, o al instante si cambia el estado.
 */


#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Preferences.h>
#include <math.h>


// ================== PINES ==================
#define DHT_PIN     4
#define DHT_TYPE    DHT11
#define MQ2_PIN     35     // A0 del MQ-2 (ADC1, solo entrada)
#define MQ7_PIN     34    // A0 del MQ-7 (ADC1)
#define BUZZER_PIN  14     // señal (S.I.G) del módulo relé/buzzer
#define LED_R_PIN   15     // RGB: rojo
#define LED_G_PIN   2      // RGB: verde
#define LED_B_PIN   16     // RGB: azul (el pin común va a 3.3V)


// ================== CONFIGURACIÓN DE HARDWARE ==================
#define RELE_ACTIVO_BAJO true    // true = se activa con LOW | false = se activa con HIGH


// Sensores MQ
const float VCC_SENSOR = 5.0;        // alimentación del sensor
const float DIVISOR    = 1.0;        // cambiar si usás divisor de tensión en A0
const float RL_MQ2     = 10.0;       // kΩ
const float RL_MQ7     = 10.0;       // kΩ
const float AIRE_LIMPIO_MQ2 = 9.83;  // Rs/R0 en aire limpio
const float AIRE_LIMPIO_MQ7 = 27.5;
const unsigned long CALENTAMIENTO_MS = 60000;   // 60 s: recién ahí se empieza a leer/publicar
const unsigned long CALIBRACION_MS   = 180000;  // 3 min: recién ahí se calibra R0 (aire limpio!)

// true = borra el R0 guardado y recalibra a los 3 min de cada arranque.
// Subí el código una vez con true (con los sensores en aire limpio) y después ponelo en false.
const bool RESETEAR_CALIBRACION = true;


// ================== UMBRALES ==================
float co_amarillo   = 35;     // ppm
float co_rojo       = 100;    // ppm
float gas_amarillo  = 300;    // ppm (MQ-2)
float gas_rojo      = 1000;   // ppm (MQ-2)
float temp_amarillo = 35;     // °C
float temp_rojo     = 45;     // °C
const float HISTERESIS = 0.9;


// ================== RED Y MQTT ==================
const char* ssid        = "TounaRedAURA";
const char* password    = "freakytouna";
const char* mqtt_server = "10.164.53.250";
const int   mqtt_port   = 1883;
const char* mqtt_user   = nullptr;
const char* mqtt_pass   = nullptr;
const char* CLIENT_ID   = "ESP32_Detector_Gases";


const char* T_STATUS    = "control/status";
const char* T_ESTADO    = "control/estado";


WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);
Preferences prefs;


// ================== ESTADO ==================
enum Estado { VERDE = 0, AMARILLO = 1, ROJO = 2 };
Estado estadoActual = VERDE;


float R0_mq2 = 10.0;
float R0_mq7 = 12.4;   // valor típico en aire limpio con RL = 10k, hasta que se calibre
bool  pedirCalibracion = false;


float co = 0, gas = 0;
float mv_mq7 = 0, mv_mq2 = 0;   // lectura cruda en mV (para diagnóstico)
float temp = NAN, hum = NAN;
bool  dhtOk = false;


bool silenciado = false;


const unsigned long INTERVALO_LECTURA     = 1000;
const unsigned long INTERVALO_DHT         = 2500;
const unsigned long INTERVALO_PUBLICACION = 300000UL;  // 5 min (publica y guarda en la base)
const unsigned long INTERVALO_CALENTANDO  = 3000;      // aviso "calentando" durante el precalentamiento
unsigned long lastLectura = 0, lastDht = 0, lastPublish = 0;
unsigned long lastWifiTry = 0, lastMqttTry = 0;


unsigned long lastBeep = 0;
bool buzzerOn = false;
bool wifiPrev = false;
bool primeraPublicacion = true;


bool calentando() { return millis() < CALENTAMIENTO_MS; }


void setBuzzer(bool on) {
  digitalWrite(BUZZER_PIN, RELE_ACTIVO_BAJO ? !on : on);
}


// RGB de ánodo común (común a 3.3V): LOW = encendido, HIGH = apagado
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r ? LOW : HIGH);
  digitalWrite(LED_G_PIN, g ? LOW : HIGH);
  digitalWrite(LED_B_PIN, b ? LOW : HIGH);
}


void actualizarSalidas() {
  if (calentando()) {
    setBuzzer(false);
    buzzerOn = false;
    setRGB(false, false, true);        // azul mientras calienta
    return;
  }


  switch (estadoActual) {
    case VERDE:
      setRGB(false, true, false);      // verde
      setBuzzer(false);
      buzzerOn = false;
      break;


    case AMARILLO:
      setRGB(true, true, false);       // rojo + verde = amarillo
      setBuzzer(false);
      buzzerOn = false;
      break;


    case ROJO:
      setRGB(true, false, false);      // rojo
      if (silenciado) {
        setBuzzer(false);
        buzzerOn = false;
      } else if (millis() - lastBeep >= 500) {
        lastBeep = millis();
        buzzerOn = !buzzerOn;
        setBuzzer(buzzerOn);
      }
      break;
  }
}


// ================== LECTURA DE SENSORES MQ ==================
// Devuelve el voltaje en V y, si se pasa vmv, también los mV crudos.
float leerVoltaje(int pin, float *vmv = nullptr) {
  uint32_t suma = 0;
  for (int i = 0; i < 16; i++) {
    suma += analogReadMilliVolts(pin);
    delayMicroseconds(500);
  }
  float mv = suma / 16.0f;
  if (vmv) *vmv = mv;
  return mv / 1000.0f * DIVISOR;
}


float calcularRs(int pin, float RL, float *vmv = nullptr) {
  float v = leerVoltaje(pin, vmv);
  v = constrain(v, 0.01f, VCC_SENSOR - 0.01f);
  return RL * (VCC_SENSOR - v) / v;
}


float ppmCO(float rs) {
  float p = 99.042f * pow(rs / R0_mq7, -1.518f);
  return constrain(p, 0.0f, 10000.0f);
}


float ppmGas(float rs) {
  float p = 574.25f * pow(rs / R0_mq2, -2.222f);
  return constrain(p, 0.0f, 20000.0f);
}


void calibrarSensores() {
  Serial.println("Calibrando en aire limpio (no acerques gas ni humo)...");
  setBuzzer(false);


  float s2 = 0, s7 = 0;
  for (int i = 0; i < 30; i++) {
    s2 += calcularRs(MQ2_PIN, RL_MQ2);
    s7 += calcularRs(MQ7_PIN, RL_MQ7);
    delay(100);
  }
  R0_mq2 = (s2 / 30) / AIRE_LIMPIO_MQ2;
  R0_mq7 = (s7 / 30) / AIRE_LIMPIO_MQ7;
  prefs.putFloat("r0_mq2", R0_mq2);
  prefs.putFloat("r0_mq7", R0_mq7);
  Serial.printf("R0 MQ-2 = %.2f kΩ | R0 MQ-7 = %.2f kΩ (guardados)\n", R0_mq2, R0_mq7);
}


void leerSensores(unsigned long now) {
  co  = ppmCO(calcularRs(MQ7_PIN, RL_MQ7, &mv_mq7));
  gas = ppmGas(calcularRs(MQ2_PIN, RL_MQ2, &mv_mq2));


  if (now - lastDht >= INTERVALO_DHT) {
    lastDht = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    dhtOk = !isnan(h) && !isnan(t);
    if (dhtOk) { hum = h; temp = t; }
    else Serial.println("Error leyendo DHT11.");
  }
}


// ================== ESTADO SEMÁFORO ==================
bool sobre(float valor, float umbral, Estado nivel) {
  float u = (estadoActual >= nivel) ? umbral * HISTERESIS : umbral;
  return valor >= u;
}


Estado evaluarEstado() {
  bool tOk = dhtOk;
  if (sobre(co, co_rojo, ROJO) || sobre(gas, gas_rojo, ROJO) ||
      (tOk && sobre(temp, temp_rojo, ROJO))) return ROJO;
  if (sobre(co, co_amarillo, AMARILLO) || sobre(gas, gas_amarillo, AMARILLO) ||
      (tOk && sobre(temp, temp_amarillo, AMARILLO))) return AMARILLO;
  return VERDE;
}


const char* nombreEstado(Estado e) {
  return e == VERDE ? "verde" : (e == AMARILLO ? "amarillo" : "rojo");
}


// ================== MQTT ==================
void publicar(const char* topic, float valor, int decimales = 1) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", decimales, valor);
  client.publish(topic, buf);
}


// Devuelve true si realmente publicó (había conexión con el broker).
bool publicarTodo() {
  if (!client.connected()) return false;
  if (dhtOk) {
    publicar("control/temperatura", temp);
    publicar("control/humedad", hum);
  }
  publicar("control/co", co);
  publicar("control/gas", gas);
  publicar("control/co_mv", mv_mq7, 0);    // crudo, útil para ver que el sensor reacciona
  publicar("control/gas_mv", mv_mq2, 0);
  client.publish(T_ESTADO, nombreEstado(estadoActual), true);
  client.publish("control/silenciado_estado", silenciado ? "1" : "0");
  return true;
}


bool esVerdadero(const char* msg) {
  String s(msg);
  s.trim();
  s.toLowerCase();
  return s == "1" || s == "true" || s == "on";
}


void setUmbral(float &var, const char* clave, float valor) {
  if (valor <= 0) { Serial.println("Umbral inválido, se ignora."); return; }
  var = valor;
  prefs.putFloat(clave, valor);
}


void callback(char* topic, byte* payload, unsigned int length) {
  char msg[32];
  unsigned int n = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
  memcpy(msg, payload, n);
  msg[n] = '\0';
  String t = String(topic);
  float valor = atof(msg);


  Serial.printf("MQTT [%s]: %s\n", topic, msg);


  if      (t == "control/umbral_co_amarillo")   setUmbral(co_amarillo,   "co_a",  valor);
  else if (t == "control/umbral_co_rojo")       setUmbral(co_rojo,       "co_r",  valor);
  else if (t == "control/umbral_gas_amarillo")  setUmbral(gas_amarillo,  "gas_a", valor);
  else if (t == "control/umbral_gas_rojo")      setUmbral(gas_rojo,      "gas_r", valor);
  else if (t == "control/umbral_temp_amarillo") setUmbral(temp_amarillo, "t_a",   valor);
  else if (t == "control/umbral_temp_rojo")     setUmbral(temp_rojo,     "t_r",   valor);
  else if (t == "control/silenciar")            silenciado = esVerdadero(msg);
  else if (t == "control/calibrar")             { if (esVerdadero(msg)) pedirCalibracion = true; }
}


bool conectarMQTT() {
  Serial.print("Conectando a MQTT... ");
  if (client.connect(CLIENT_ID, mqtt_user, mqtt_pass, T_STATUS, 1, true, "offline")) {
    Serial.println("OK");
    client.publish(T_STATUS, "online", true);
    client.subscribe("control/umbral_co_amarillo");
    client.subscribe("control/umbral_co_rojo");
    client.subscribe("control/umbral_gas_amarillo");
    client.subscribe("control/umbral_gas_rojo");
    client.subscribe("control/umbral_temp_amarillo");
    client.subscribe("control/umbral_temp_rojo");
    client.subscribe("control/silenciar");
    client.subscribe("control/calibrar");
    return true;
  }
  Serial.printf("fallo, rc=%d\n", client.state());
  return false;
}


void gestionarRed() {
  unsigned long now = millis();


  if (WiFi.status() != WL_CONNECTED) {
    wifiPrev = false;
    if (now - lastWifiTry > 10000) {
      lastWifiTry = now;
      Serial.println("WiFi desconectado, reintentando...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
    return;
  }


  if (!wifiPrev) {
    wifiPrev = true;
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  }


  if (!client.connected()) {
    if (now - lastMqttTry > 5000) {
      lastMqttTry = now;
      conectarMQTT();
    }
  } else {
    client.loop();
  }
}


void cargarConfig() {
  co_amarillo   = prefs.getFloat("co_a",  co_amarillo);
  co_rojo       = prefs.getFloat("co_r",  co_rojo);
  gas_amarillo  = prefs.getFloat("gas_a", gas_amarillo);
  gas_rojo      = prefs.getFloat("gas_r", gas_rojo);
  temp_amarillo = prefs.getFloat("t_a",   temp_amarillo);
  temp_rojo     = prefs.getFloat("t_r",   temp_rojo);


  float r2 = prefs.getFloat("r0_mq2", 0);
  float r7 = prefs.getFloat("r0_mq7", 0);
  if (r2 > 0 && r7 > 0) {
    R0_mq2 = r2;
    R0_mq7 = r7;
    Serial.printf("R0 cargados: MQ-2 = %.2f | MQ-7 = %.2f kΩ\n", R0_mq2, R0_mq7);
  } else {
    pedirCalibracion = true;
    Serial.println("Sin calibración guardada: se calibrará a los 3 min (aire limpio).");
  }
}


// ================== SETUP ==================
void setup() {
  Serial.begin(115200);


  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  setRGB(false, false, true);          // azul al arrancar (calentando)


  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(MQ7_PIN, ADC_11db);


  prefs.begin("detector", false);
  if (RESETEAR_CALIBRACION) {
    prefs.remove("r0_mq2");
    prefs.remove("r0_mq7");
    Serial.println("Calibración guardada borrada.");
  }
  cargarConfig();


  dht.begin();


  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);


  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(30);
  client.setSocketTimeout(2);
}


// ================== LOOP ==================
void loop() {
  gestionarRed();
  unsigned long now = millis();


  // Calibra solo cuando los sensores ya estabilizaron (>= 3 min) o si se pide por MQTT.
  if (pedirCalibracion && now >= CALIBRACION_MS) {
    calibrarSensores();
    pedirCalibracion = false;
  }


  if (now - lastLectura >= INTERVALO_LECTURA) {
    lastLectura = now;


    if (calentando()) {
      if (client.connected() && now - lastPublish >= INTERVALO_CALENTANDO) {
        lastPublish = now;
        client.publish(T_ESTADO, "calentando");
      }
    } else {
      leerSensores(now);


      Estado nuevo = evaluarEstado();
      bool cambio = (nuevo != estadoActual);
      if (cambio) {
        if (nuevo == ROJO) silenciado = false;
        estadoActual = nuevo;
      }


      // Publica: al cambiar de estado, la primera vez, y luego cada 5 min.
      // Si no hay conexión, no pierde el turno: reintenta en el siguiente ciclo.
      if (cambio || primeraPublicacion || now - lastPublish >= INTERVALO_PUBLICACION) {
        if (publicarTodo()) {
          lastPublish = now;
          primeraPublicacion = false;
        }
      }


      Serial.printf("Temp: %.1f C | Hum: %.1f %% | CO: %.1f ppm (%.0f mV, R0=%.1f) | Gas MQ-2: %.1f ppm (%.0f mV) | Estado: %s\n",
                    temp, hum, co, mv_mq7, R0_mq7, gas, mv_mq2, nombreEstado(estadoActual));
    }
  }


  actualizarSalidas();
}
