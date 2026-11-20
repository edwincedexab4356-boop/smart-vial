/*
  SMART VIAL - Transito inteligente
  Semaforo inteligente con cruce peatonal para Arduino UNO

  - Ultrasonico 1 (peaton): detecta a la persona que llega al cruce.
  - Ultrasonico 2 (vehiculo): detecta el auto que se acerca y calcula su velocidad.
  - Modulo de semaforo LED (R, Y, G, GND) para autos.
  - Semaforo de peaton con 2 LEDs (rojo y verde).
  - LCD 16x2 I2C: "PUEDE CRUZAR 9s" (cuenta 9 -> 0), "ESPERE", etc.
  - Matriz 8x8 MAX7219: muñeco parado / muñeco caminando.
  - Buzzer: pitidos de cruce (ayuda a personas con poca vision) y alarma.
  - Bluetooth HC-06 en D0/D1 (9600, viene asi de fabrica; nombre "HC-06", clave 1234):
    manda el estado y recibe ordenes del simulador web o de una app Bluetooth del celular
    (por ejemplo "Serial Bluetooth Terminal" o "Arduino Bluetooth Controller").

  CONEXIONES
  D0  RX  <- TXD del HC-06                       D1  TX  -> RXD del HC-06 con divisor (1k arriba, 2k abajo)
  HC-06: VCC -> 5V, GND -> GND
  D2  TRIG ultrasonico peaton                     D3  ECHO ultrasonico peaton
  D4  TRIG ultrasonico vehiculo                   D5  ECHO ultrasonico vehiculo
  D6  Buzzer (+)
  D7  LED rojo peaton  (+ resistencia de 220 ohm)
  D8  LED verde peaton (+ resistencia de 220 ohm)
  D10 MAX7219 CS            D11 MAX7219 DIN                     D13 MAX7219 CLK
  A0  Modulo semaforo AUTOS   R      A1  Modulo semaforo AUTOS   Y
  A2  Modulo semaforo AUTOS   G      GND del modulo autos  -> GND
  A4  SDA LCD          A5 SCL LCD
  El modulo de semaforo ya trae sus resistencias: se conecta directo.
  Los LEDs del peaton: pata larga al pin con su resistencia de 220 ohm, pata corta a GND.
  Quedan libres: D9, D12 y A3.

  IMPORTANTE: el HC-06 comparte D0/D1 con el USB. Desconecta el cable de D0 (TXD del HC-06)
  mientras subes el programa y vuelve a ponerlo despues.
  Comandos cortos (una letra, sirven en apps que no mandan "enter"):
    P = peaton   M = mantenimiento   N = normal   S = silenciar   A = activar buzzer   E = estado
  Libreria necesaria: LiquidCrystal I2C (Frank de Brabander).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------- pines ----------
const byte P_TRIG = 2, P_ECHO = 3;      // ultrasonico peaton
const byte V_TRIG = 4, V_ECHO = 5;      // ultrasonico vehiculo
const byte BUZZER = 6;
const byte MX_CS = 10, MX_DIN = 11, MX_CLK = 13;    // MAX7219 (matriz)
const byte LED_AUTO_R = A0, LED_AUTO_A = A1, LED_AUTO_V = A2;   // modulo semaforo autos
const byte LED_PEAT_R = 7, LED_PEAT_V = 8;                      // LEDs del peaton

// ---------- ajustes ----------
const bool SEMAFORO_INVERTIDO = false; // true si el modulo de autos prende al reves (anodo comun)
const bool ESPEJO_MATRIZ = false; // true si el muñeco sale al reves
int  UMBRAL_P = 20;   // cm: a esta distancia o menos hay un peaton
int  UMBRAL_V = 15;   // cm: a esta distancia o menos hay un vehiculo en el cruce
const unsigned long T_VERDE_MIN = 6000;  // verde minimo para autos antes de dar paso
const unsigned long T_AMARILLO  = 2500;
const unsigned long T_TODO_ROJO = 1000;
const unsigned long T_ESPERA_MAX = 8000; // si el auto no se va, igual se da paso al peaton
const unsigned long T_FIN = 1500;
const int VEL_ALERTA = 25;  // cm/s: un auto que se acerca mas rapido con luz roja = no va a frenar

// ---------- estados ----------
enum Fase { VERDE = 0, ESPERA_AUTO = 1, AMARILLO = 2, TODO_ROJO = 3, CRUCE = 4, FIN = 5, INTERMITENTE = 6 };
Fase fase = VERDE;
unsigned long tFase = 0;

LiquidCrystal_I2C *lcd = nullptr;
bool hayLcd = false;
String lcd1 = "", lcd2 = "";

int dP = 300, dV = 300;          // distancias en cm
int cuentaP = 0, cuentaV = 0;    // lecturas seguidas que confirman
bool peaton = false, vehiculo = false, pedido = false;
int cuenta = -1;                 // segundos que le quedan al peaton (-1 = no hay cruce)
float velocidad = 0;             // cm/s del vehiculo que se acerca
int dVant = -1; unsigned long tVant = 0;
unsigned int cruces = 0;
bool silencio = false;
bool alarmaDada = false;
unsigned long tSt = 0, tSensor = 0, tAnim = 0;
byte cuadro = 0;
bool turnoSensor = false;
String entrada = "";
unsigned long tEntrada = 0;   // ultima letra recibida (para apps que no mandan "enter")

// ---------- dibujos de la matriz ----------
const byte M_PARADO[8]  = {0x18,0x18,0x00,0x3C,0x5A,0x18,0x24,0x24};
const byte M_CAMINA1[8] = {0x0C,0x0C,0x00,0x1C,0x2E,0x0C,0x14,0x22};
const byte M_CAMINA2[8] = {0x0C,0x0C,0x00,0x1C,0x1C,0x0C,0x0C,0x14};
const byte M_ALERTA[8]  = {0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18};

// ================== MAX7219 (sin libreria) ==================
void mxEnviar(byte reg, byte val) {
  digitalWrite(MX_CS, LOW);
  shiftOut(MX_DIN, MX_CLK, MSBFIRST, reg);
  shiftOut(MX_DIN, MX_CLK, MSBFIRST, val);
  digitalWrite(MX_CS, HIGH);
}
byte invertirBits(byte b) {
  byte r = 0;
  for (byte i = 0; i < 8; i++) if (b & (1 << i)) r |= (1 << (7 - i));
  return r;
}
void mxDibujar(const byte *d) {
  for (byte f = 0; f < 8; f++) mxEnviar(f + 1, ESPEJO_MATRIZ ? invertirBits(d[f]) : d[f]);
}
void mxIniciar() {
  pinMode(MX_CS, OUTPUT); pinMode(MX_DIN, OUTPUT); pinMode(MX_CLK, OUTPUT);
  digitalWrite(MX_CS, HIGH);
  mxEnviar(0x0F, 0);  // sin prueba
  mxEnviar(0x09, 0);  // sin decodificar
  mxEnviar(0x0B, 7);  // 8 filas
  mxEnviar(0x0A, 4);  // brillo
  mxEnviar(0x0C, 1);  // encendida
  mxDibujar(M_PARADO);
}

// ================== LCD ==================
String a16(String s) { while (s.length() < 16) s += ' '; return s.substring(0, 16); }
void pantalla(String l1, String l2) {
  if (l1 == lcd1 && l2 == lcd2) return;
  lcd1 = l1; lcd2 = l2;
  if (hayLcd) {
    lcd->setCursor(0, 0); lcd->print(a16(l1));
    lcd->setCursor(0, 1); lcd->print(a16(l2));
  }
  Serial.print(F("LC,")); Serial.print(l1); Serial.print(','); Serial.println(l2);
}
bool existeI2C(byte dir) { Wire.beginTransmission(dir); return Wire.endTransmission() == 0; }

// ================== buzzer ==================
void pitar(byte veces, int on, int off, int frec) {
  Serial.print(F("BZ,")); Serial.print(veces); Serial.print(','); Serial.print(on); Serial.print(','); Serial.println(off);
  if (silencio) return;
  if (veces == 1) { tone(BUZZER, frec, on); return; }
  for (byte i = 0; i < veces; i++) { tone(BUZZER, frec, on); delay(on + off); }
}

// ================== sensores ==================
int medir(byte trig, byte echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 25000UL);
  if (us == 0) return 300;
  int cm = us / 58;
  return cm > 300 ? 300 : cm;
}

void leerSensores() {
  // un sensor por vez para que no se estorben los ecos
  turnoSensor = !turnoSensor;
  if (turnoSensor) {
    dP = medir(P_TRIG, P_ECHO);
    if (dP <= UMBRAL_P) { if (cuentaP < 5) cuentaP++; } else if (cuentaP > 0) cuentaP--;
    bool antes = peaton;
    if (cuentaP >= 3) peaton = true; else if (cuentaP == 0) peaton = false;
    if (peaton && !antes) {
      evento("Peaton detectado a " + String(dP) + " cm");
    }
  } else {
    dV = medir(V_TRIG, V_ECHO);
    unsigned long ahora = millis();
    if (dV < 300 && dVant >= 0 && dVant < 300 && ahora > tVant) {
      float v = (dVant - dV) * 1000.0 / (ahora - tVant);   // positivo = se acerca
      if (v < 0) v = 0;
      if (v > 300) v = 300;
      velocidad = velocidad * 0.6 + v * 0.4;
    } else if (dV >= 300) velocidad = 0;
    dVant = dV; tVant = ahora;
    if (dV <= UMBRAL_V) { if (cuentaV < 4) cuentaV++; } else if (cuentaV > 0) cuentaV--;
    bool antes = vehiculo;
    if (cuentaV >= 2) vehiculo = true; else if (cuentaV == 0) vehiculo = false;
    if (vehiculo && !antes) evento("Vehiculo detectado a " + String(dV) + " cm");
    if (!vehiculo && antes) evento("El vehiculo ya paso");
  }
}

// ================== luces (modulo de autos + LEDs del peaton) ==================
bool esModulo(byte pin) { return pin == LED_AUTO_R || pin == LED_AUTO_A || pin == LED_AUTO_V; }
void luz(byte pin, bool on) { digitalWrite(pin, (on != (SEMAFORO_INVERTIDO && esModulo(pin))) ? HIGH : LOW); }
bool L(byte pin) { return (digitalRead(pin) == HIGH) != (SEMAFORO_INVERTIDO && esModulo(pin)); }
void luces(bool aR, bool aA, bool aV, bool pR, bool pV) {
  luz(LED_AUTO_R, aR); luz(LED_AUTO_A, aA); luz(LED_AUTO_V, aV);
  luz(LED_PEAT_R, pR); luz(LED_PEAT_V, pV);
}

void evento(String t) { Serial.print(F("EV,")); Serial.println(t); }

void cambiar(Fase f) {
  fase = f; tFase = millis();
  switch (f) {
    case VERDE:
      luces(0, 0, 1, 1, 0); cuenta = -1; mxDibujar(M_PARADO);
      evento("Autos en verde"); break;
    case ESPERA_AUTO:
      evento("Peaton espera: hay un vehiculo en el cruce"); break;
    case AMARILLO:
      luces(0, 1, 0, 1, 0); evento("Autos en amarillo"); pitar(1, 150, 0, 1500); break;
    case TODO_ROJO:
      luces(1, 0, 0, 1, 0); evento("Autos en rojo"); break;
    case CRUCE:
      luces(1, 0, 0, 0, 1); cuenta = 9; alarmaDada = false;
      cruces++; evento("PUEDE CRUZAR (cruce #" + String(cruces) + ")"); pitar(1, 120, 0, 2400); break;
    case FIN:
      luces(1, 0, 0, 1, 0); cuenta = -1; mxDibujar(M_PARADO);
      pedido = false; evento("Fin del cruce"); pitar(2, 90, 90, 1200); break;
    case INTERMITENTE:
      luces(0, 0, 0, 0, 0); cuenta = -1; mxDibujar(M_ALERTA);
      pedido = false; evento("Modo intermitente (mantenimiento)"); break;
  }
}

void actualizarFase() {
  unsigned long t = millis() - tFase;
  switch (fase) {
    case VERDE:
      // mientras haya alguien frente al sensor, el paso queda pedido
      if (peaton && !pedido) { pedido = true; evento("Peaton pide paso"); pitar(1, 60, 0, 1800); }
      if (pedido && t >= T_VERDE_MIN) cambiar(vehiculo ? ESPERA_AUTO : AMARILLO);
      if (pedido) pantalla("  SMART VIAL", "Peaton: ESPERE");
      else        pantalla("  SMART VIAL", "Autos: SIGA");
      break;
    case ESPERA_AUTO:
      pantalla("VEHICULO CERCA", "Peaton: ESPERE");
      if (!vehiculo || t >= T_ESPERA_MAX) cambiar(AMARILLO);
      break;
    case AMARILLO:
      pantalla("AUTOS: CUIDADO", "Peaton: ESPERE");
      if (t >= T_AMARILLO) cambiar(TODO_ROJO);
      break;
    case TODO_ROJO:
      pantalla("AUTOS: ALTO", "Peaton: LISTO");
      if (t >= T_TODO_ROJO) cambiar(CRUCE);
      break;
    case CRUCE: {
      int n = 9 - (int)(t / 1000);
      if (n < 0) { cambiar(FIN); break; }
      if (n != cuenta) {
        cuenta = n;
        if (n <= 3) pitar(2, 60, 60, 2600); else pitar(1, 60, 0, 2400);
      }
      // ultimos 3 segundos: el verde del peaton parpadea
      luz(LED_PEAT_V, !(n <= 3 && (t % 500) >= 250));
      if (millis() - tAnim >= 300) { tAnim = millis(); cuadro ^= 1; mxDibujar(cuadro ? M_CAMINA1 : M_CAMINA2); }
      String l1 = "PUEDE CRUZAR " + String(n) + "s";
      if (vehiculo && velocidad > VEL_ALERTA) {
        pantalla(l1, "!AUTO NO FRENA!");
        if (!alarmaDada) { alarmaDada = true; evento("ALERTA: vehiculo acercandose con luz roja"); pitar(3, 80, 50, 3000); }
      } else pantalla(l1, n <= 3 ? "Apurese..." : "Autos: ALTO");
      break; }
    case FIN:
      pantalla("NO CRUCE", "Autos: SIGAN");
      if (t >= T_FIN) cambiar(VERDE);
      break;
    case INTERMITENTE: {
      bool on = (t % 1000) < 500;
      luz(LED_AUTO_A, on); luz(LED_PEAT_R, on);
      pantalla("MANTENIMIENTO", "Cruce con cuidado");
      break; }
  }
}

// ================== comunicacion ==================
void enviarEstado() {
  Serial.print(F("ST,"));
  Serial.print(fase); Serial.print(',');
  Serial.print(L(LED_AUTO_R)); Serial.print(','); Serial.print(L(LED_AUTO_A)); Serial.print(','); Serial.print(L(LED_AUTO_V)); Serial.print(',');
  Serial.print(L(LED_PEAT_R)); Serial.print(','); Serial.print(L(LED_PEAT_V)); Serial.print(',');
  Serial.print(cuenta); Serial.print(',');
  Serial.print(dP); Serial.print(','); Serial.print(dV); Serial.print(',');
  Serial.print((int)velocidad); Serial.print(',');
  Serial.print(peaton); Serial.print(','); Serial.print(vehiculo); Serial.print(',');
  Serial.print(pedido); Serial.print(',');
  Serial.print(cruces); Serial.print(','); Serial.print(silencio); Serial.print(',');
  Serial.print(UMBRAL_P); Serial.print(','); Serial.print(UMBRAL_V); Serial.print(',');
  Serial.println(hayLcd);
}

void comando(String c) {
  c.trim(); c.toUpperCase();
  // comandos de una letra para apps Bluetooth del celular
  if (c == "P") c = "PEATON";
  else if (c == "M") c = "MANT:1";
  else if (c == "N") c = "MANT:0";
  else if (c == "S") c = "SILENCIO:1";
  else if (c == "A") c = "SILENCIO:0";
  else if (c == "E") c = "ESTADO";
  if (c == "ESTADO") { lcd1 = ""; }
  else if (c == "PEATON") {
    if (fase == VERDE || fase == ESPERA_AUTO) { pedido = true; evento("Boton de peaton (app)"); pitar(1, 60, 0, 1800); }
    else evento("El cruce ya esta en curso");
  }
  else if (c == "PRUEBA_MX") {   // prende los 64 puntos 3 segundos
    mxIniciar(); mxEnviar(0x0F, 1); evento("Prueba de matriz: todo encendido");
    delay(3000); mxEnviar(0x0F, 0); mxDibujar(M_PARADO);
  }
  else if (c == "MANT:1") cambiar(INTERMITENTE);
  else if (c == "MANT:0") { if (fase == INTERMITENTE) cambiar(VERDE); }
  else if (c == "SILENCIO:1") { silencio = true; noTone(BUZZER); evento("Buzzer silenciado"); }
  else if (c == "SILENCIO:0") { silencio = false; evento("Buzzer activado"); }
  else if (c.startsWith("UMBRAL_P:")) { UMBRAL_P = constrain(c.substring(9).toInt(), 3, 200); evento("Umbral peaton = " + String(UMBRAL_P) + " cm"); }
  else if (c.startsWith("UMBRAL_V:")) { UMBRAL_V = constrain(c.substring(9).toInt(), 3, 200); evento("Umbral vehiculo = " + String(UMBRAL_V) + " cm"); }
  else if (c.length()) evento("Comando desconocido: " + c);
  enviarEstado();
}

void leerSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (entrada.length()) comando(entrada); entrada = ""; }
    else if (entrada.length() < 40) entrada += ch;
    tEntrada = millis();
  }
  // si la app no manda "enter", el comando se toma tras 150 ms sin letras nuevas
  if (entrada.length() && millis() - tEntrada > 150) { comando(entrada); entrada = ""; }
}

// ================== inicio ==================
void setup() {
  Serial.begin(9600);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT);
  pinMode(V_TRIG, OUTPUT); pinMode(V_ECHO, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_AUTO_R, OUTPUT); pinMode(LED_AUTO_A, OUTPUT); pinMode(LED_AUTO_V, OUTPUT);
  pinMode(LED_PEAT_R, OUTPUT); pinMode(LED_PEAT_V, OUTPUT);

  Serial.println(F("EV,Smart Vial iniciando (Bluetooth HC-06)"));
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // si la LCD esta mal conectada, no se traba
  byte dir = existeI2C(0x27) ? 0x27 : (existeI2C(0x3F) ? 0x3F : 0);
  if (dir) {
    lcd = new LiquidCrystal_I2C(dir, 16, 2);
    lcd->init(); lcd->backlight(); hayLcd = true;
  }
  mxIniciar();

  // prueba de luces al encender
  luces(1, 1, 1, 1, 1);   // prueba: prenden todas las luces
  pantalla("  SMART VIAL", "Transito intelig");
  pitar(1, 150, 0, 2000);
  delay(1200);
  evento(hayLcd ? "LCD encontrada" : "No se encontro la LCD (revisa SDA/SCL)");
  cambiar(VERDE);
}

void loop() {
  leerSerial();
  if (millis() - tSensor >= 60) { tSensor = millis(); leerSensores(); }
  actualizarFase();
  if (millis() - tSt >= 500) { tSt = millis(); enviarEstado(); }
  static unsigned long tMx = 0;
  if (millis() - tMx >= 5000) { tMx = millis(); mxEnviar(0x0F, 0); mxEnviar(0x09, 0); mxEnviar(0x0B, 7); mxEnviar(0x0A, 4); mxEnviar(0x0C, 1); }
}
