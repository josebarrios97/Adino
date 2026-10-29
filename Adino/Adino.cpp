/*
  CARRITO BLUETOOTH - Arduino UNO + 2 x DRV8871 + HC-05
  Maquina de estados (FSM) sin delay(), todo con millis().

  CABLEADO
    HC-05 TXD -> D2 (RX de SoftwareSerial)
    HC-05 RXD <- D3 (TX de SoftwareSerial) A TRAVES DE DIVISOR: D3 -[1k]- HC05_RXD -[2k]- GND
    HC-05 VCC -> 5V (del LM2596), GND -> GND
    DRV8871 IZQUIERDO: IN1 -> D9 (PWM)  IN2 -> D10 (PWM)   VM -> +VBAT  GND -> GND
    DRV8871 DERECHO  : IN1 -> D5 (PWM)  IN2 -> D6  (PWM)   VM -> +VBAT  GND -> GND
    El DRV8871 NO tiene pin enable: la velocidad se da por PWM en IN1/IN2.
    Arduino 5V y HC-05 VCC salen del LM2596 ajustado a 5.0 V (NO usar el pin VIN).

  PROTOCOLO (texto ASCII). MODO "MANTENER MIENTRAS SE PRESIONA":
    Cada boton manda una LETRA MAYUSCULA al presionar y la MINUSCULA al soltar.
    F / f  adelante      B / b  atras
    L / l  giro izquierda   R / r  giro derecha   (se pueden combinar con F o B)
    V / v  boost (presionar / soltar)
    S o s  freno activo inmediato (borra todo lo que estaba presionado)
    (Joystick opcional: J<x>,<y>;  con x,y entre -100 y 100)
    La orden se mantiene hasta soltar el boton: la app NO necesita repetir la letra.

  TELEMETRIA hacia la app: BOOST_NO (en cooldown), BOOST_ON, BOOST_END, BOOST_OK
*/

#include <SoftwareSerial.h>

// ---------- PINES ----------
const uint8_t BT_RX = 2;
const uint8_t BT_TX = 3;
const uint8_t L_IN1 = 9, L_IN2 = 10;         // DRV8871 izquierdo (D9, D10 con PWM)
const uint8_t R_IN1 = 5, R_IN2 = 6;          // DRV8871 derecho   (D5, D6 con PWM)
const bool INVERT_LEFT = false;             // pon true si un motor gira al reves
const bool INVERT_RIGHT = false;

// ---------- PARAMETROS (ajustables) ----------
const int PWM_CRUISE = 180;                  // ~7.5 V crucero (nunca 255)
const int PWM_BOOST = 218;                  // ~9.1 V boost
const unsigned long RAMP_MS = 400;       // rampa 0 -> crucero
const unsigned long BOOST_MS = 2500;      // duracion maxima del boost
const unsigned long COOLDOWN_MS = 6000;      // bloqueo tras cada boost
const unsigned long FAILSAFE_MS = 500;       // (solo si HOLD_MODE=false) sin datos -> PWM 0
const bool HOLD_MODE = true;                 // true: la orden se mantiene hasta soltar (la app manda 1 sola letra)
// false: exige datos cada FAILSAFE_MS (solo si la app repite el envio)
const bool USE_STATE_PIN = false;            // true: usa el pin STATE del HC-05 en D4 para detectar desconexion
const uint8_t STATE_PIN = 4;
const unsigned long STATE_LOST_MS = 300;     // STATE en LOW este tiempo -> se considera desconectado
const bool FAILSAFE_ACTIVE_BRAKE = false;    // false = PWM 0 (rueda libre), true = freno activo
const int DEADZONE = 8;                     // zona muerta del joystick (%)
const int TURN_SOFT = 40;                    // % del motor interior en botones L/R
const unsigned long LOOP_MS = 20;            // periodo de control
const int STEP_CALC = (PWM_CRUISE * LOOP_MS) / RAMP_MS;   // PWM por tick: 0->180 en 400 ms
const int STEP = (STEP_CALC < 1) ? 1 : STEP_CALC;
static_assert(PWM_CRUISE > 0 && PWM_BOOST >= PWM_CRUISE && PWM_BOOST <= 230, "PWM fuera de rango seguro");

// ---------- ESTADOS ----------
enum State { S_FAILSAFE, S_STOPPED, S_CRUISE, S_BOOST };
State state = S_FAILSAFE;

SoftwareSerial bt(BT_RX, BT_TX);

int joyL = 0, joyR = 0;          // pedido del piloto, -100..100
int curL = 0, curR = 0;          // PWM actual con signo (tras la rampa)
int ceilNow = PWM_CRUISE;        // techo de PWM actual (sube/baja con rampa)

bool cooldown = false;
bool boostTiming = false;
unsigned long cooldownStart = 0, boostStart = 0;
unsigned long lastRx = 0, lastBoostCmd = 0, lastTick = 0;

char buf[16];
uint8_t idx = 0;
bool inPacket = false;
bool hF = false, hB = false, hL = false, hR = false;   // botones mantenidos
unsigned long stateLowSince = 0;
bool stateLow = false;
unsigned long pktStart = 0;

// ---------- UTILIDADES ----------
int slew(int cur, int tgt, int step) {
    if (tgt > cur) return min(cur + step, tgt);
    return max(cur - step, tgt);
}

void notify(const __FlashStringHelper* m) { bt.println(m); }

// DRV8871: adelante = PWM en IN1 e IN2 en 0 | reversa = al reves
// freno = IN1 e IN2 en HIGH | rueda libre = ambas en LOW
void driveMotor(uint8_t in1, uint8_t in2, int v, bool brake) {
    if (v > 0) {
        analogWrite(in2, 0);
        analogWrite(in1, min(v, PWM_BOOST));
    }
    else if (v < 0) {
        analogWrite(in1, 0);
        analogWrite(in2, min(-v, PWM_BOOST));
    }
    else if (brake) {
        // Freno activo: ambas entradas en HIGH cortocircuitan el motor (no aplica voltaje de bateria)
        digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
    }
    else {
        digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    }
}

void setDrive(int l, int r) {
    joyL = constrain(l, -100, 100);
    joyR = constrain(r, -100, 100);
    if (state == S_STOPPED) state = S_CRUISE;
}

void endBoost() {
    state = S_CRUISE;
    boostTiming = false;
    cooldown = true;
    cooldownStart = millis();
    notify(F("BOOST_END"));
}

void requestBoost() {
    unsigned long now = millis();
    bool fresh = (now - lastBoostCmd) > 300;   // evita re-disparo si el boton sigue apretado
    lastBoostCmd = now;
    if (state == S_BOOST || !fresh) return;
    if (cooldown) { notify(F("BOOST_NO")); return; }
    state = S_BOOST;
    boostTiming = false;                       // el cronometro arranca al terminar la rampa
    notify(F("BOOST_ON"));
}

void stopNow() {                             // freno activo inmediato
    hF = hB = hL = hR = false;
    if (state == S_BOOST) endBoost();
    joyL = joyR = 0;
    curL = curR = 0;
    state = S_STOPPED;
}

// Combina los botones mantenidos (F/B/L/R) en una orden de movimiento.
void applyButtons() {
    int y = (hF ? 1 : 0) - (hB ? 1 : 0);
    int t = (hR ? 1 : 0) - (hL ? 1 : 0);
    if (y == 0 && t == 0)      setDrive(0, 0);                    // nada presionado: baja la rampa y frena
    else if (y >= 0 && t < 0)  setDrive(TURN_SOFT, 100);          // adelante (o solo giro) a la izquierda
    else if (y >= 0 && t > 0)  setDrive(100, TURN_SOFT);          // adelante (o solo giro) a la derecha
    else if (y > 0)            setDrive(100, 100);
    else if (t < 0)            setDrive(-TURN_SOFT, -100);        // reversa girando a la izquierda
    else if (t > 0)            setDrive(-100, -TURN_SOFT);        // reversa girando a la derecha
    else                       setDrive(-100, -100);
}

// ---------- PARSER ----------
void parseJoystick() {
    int x = 0, y = 0;
    if (sscanf(buf, "%d,%d", &x, &y) != 2) return;
    x = constrain(x, -100, 100);
    y = constrain(y, -100, 100);
    if (abs(x) < DEADZONE) x = 0;
    if (abs(y) < DEADZONE) y = 0;
    int l = y + x, r = y - x;
    int m = max(abs(l), abs(r));
    if (m > 100) { l = l * 100 / m; r = r * 100 / m; }
    setDrive(l, r);
}

void handleByte(char c) {
    unsigned long now = millis();
    lastRx = now;
    if (state == S_FAILSAFE) { state = S_STOPPED; joyL = joyR = 0; hF = hB = hL = hR = false; }  // reconexion: arranca en neutro

    if (inPacket) {
        if (c == ';' || c == '\n' || c == '#') { buf[idx] = 0; parseJoystick(); inPacket = false; }
        else if (idx < sizeof(buf) - 1) buf[idx++] = c;
        else inPacket = false;                   // paquete corrupto
        return;
    }
    switch (c) {
    case 'J': inPacket = true; idx = 0; pktStart = now; break;
    case 'F': hF = true;  applyButtons(); break;
    case 'f': hF = false; applyButtons(); break;
    case 'B': hB = true;  applyButtons(); break;
    case 'b': hB = false; applyButtons(); break;
    case 'L': hL = true;  applyButtons(); break;
    case 'l': hL = false; applyButtons(); break;
    case 'R': hR = true;  applyButtons(); break;
    case 'r': hR = false; applyButtons(); break;
    case 'S':
    case 's': stopNow(); break;            // freno (acepta mayuscula y minuscula)
    case 'V': requestBoost(); break;
    case 'v': if (state == S_BOOST) endBoost(); break;
    default: break;                          // cualquier otro byte solo cuenta como "latido"
    }
}

// ---------- CONTROL ----------
void controlTick(unsigned long now) {
    if (cooldown && now - cooldownStart >= COOLDOWN_MS) { cooldown = false; notify(F("BOOST_OK")); }

    if (state == S_BOOST) {
        if (!boostTiming && ceilNow >= PWM_BOOST) { boostTiming = true; boostStart = now; }
        if (boostTiming && now - boostStart >= BOOST_MS) endBoost();   // cronometro asesino
    }

    ceilNow = slew(ceilNow, (state == S_BOOST) ? PWM_BOOST : PWM_CRUISE, STEP);

    int tL = (int)((long)joyL * ceilNow / 100);
    int tR = (int)((long)joyR * ceilNow / 100);
    curL = slew(curL, tL, STEP);               // rampa de aceleracion (soft-start)
    curR = slew(curR, tR, STEP);

    bool brake = (state != S_FAILSAFE) || FAILSAFE_ACTIVE_BRAKE;
    driveMotor(L_IN1, L_IN2, INVERT_LEFT ? -curL : curL, brake);
    driveMotor(R_IN1, R_IN2, INVERT_RIGHT ? -curR : curR, brake);
}

void enterFailsafe() {
    if (state == S_BOOST) endBoost();
    state = S_FAILSAFE;
    hF = hB = hL = hR = false;
    joyL = joyR = 0;
    curL = curR = 0;
    ceilNow = PWM_CRUISE;
}

// ---------- ARDUINO ----------
void setup() {
    pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);   // arranca con motores libres
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    if (USE_STATE_PIN) pinMode(STATE_PIN, INPUT);
    bt.begin(9600);                            // velocidad de datos por defecto del HC-05
    lastRx = millis();
    lastTick = millis();
}

void loop() {
    unsigned long now = millis();

    while (bt.available()) handleByte((char)bt.read());

    if (inPacket && now - pktStart > 150) inPacket = false;   // paquete incompleto

    if (!HOLD_MODE && state != S_FAILSAFE && now - lastRx > FAILSAFE_MS) enterFailsafe();

    if (USE_STATE_PIN) {                       // STATE del HC-05: HIGH = celular conectado
        if (digitalRead(STATE_PIN) == LOW) {
            if (!stateLow) { stateLow = true; stateLowSince = now; }
            else if (state != S_FAILSAFE && now - stateLowSince >= STATE_LOST_MS) enterFailsafe();
        }
        else stateLow = false;
    }

    if (now - lastTick >= LOOP_MS) {
