/* ============================================================
   PROJETO: FUNDAMENTOS TECNOLÓGICOS II - PRÁTICAS DE EXTENSÃO
   AUTOR: Mikael Aurio Martins de Pula da Silva
   VERSÃO: 2.4 (Novembro/2025)

   -------------------- HISTÓRICO DAS VERSÕES --------------------

   V1.0 — Primeira Versão (1 sensor central)
   V2.1 — Evolução com 3 sensores
   V2.2 — Melhoria de controle e navegação
   V2.3 — Otimizações gerais, desvio lateral, ré inteligente

   V2.4 — Preferência Configurável de Direção (NOVO)
   - Adicionada variável global PREFERENCIA_GIRO (0=esq, 1=dir)
   - Lógica de desvio frontal agora respeita a preferência primeiro
   - Caso ambas as direções estejam ruins, continua usando o servo para decidir
   - Comentários atualizados e reorganizados

   ---------------------------------------------------------------
   Notas:
   - Mantido o mapeamento físico original dos motores.
   - Código preparado para ambientes estreitos e corredores.
   - Ajuste de limites recomendado após montagem física real.
=========================================================================== */

#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// ===== CONFIGURAÇÃO DE PREFERÊNCIA DE GIRO =====
// 0 → preferir virar ESQUERDA primeiro
// 1 → preferir virar DIREITA primeiro
int PREFERENCIA_GIRO = 0;   // <<< ALTERE AQUI SE NECESSÁRIO

// ===== MOTORES =====
AF_DCMotor motor1(1, MOTOR12_64KHZ); // frente direita
AF_DCMotor motor2(2, MOTOR12_64KHZ); // frente esquerda
AF_DCMotor motor3(3, MOTOR34_64KHZ); // traseira esquerda
AF_DCMotor motor4(4, MOTOR34_64KHZ); // traseira direita

// ===== SENSORES =====
#define TRIG_CENTRO  A1
#define ECHO_CENTRO  A4
#define TRIG_ESQ     A0
#define ECHO_ESQ     A3
#define TRIG_DIR     A2
#define ECHO_DIR     A5
#define MAX_DIST     250

NewPing sonarCentro(TRIG_CENTRO, ECHO_CENTRO, MAX_DIST);
NewPing sonarEsq(TRIG_ESQ, ECHO_ESQ, MAX_DIST);
NewPing sonarDir(TRIG_DIR, ECHO_DIR, MAX_DIST);

// ===== SERVO =====
Servo servoRadar;
const int SERVO_PIN = 10;
const int ANG_LEFT = 150;
const int ANG_CENTER = 90;
const int ANG_RIGHT = 30;

// ===== PARÂMETROS =====
const int BASE_SPEED_MIN = 120;
const int BASE_SPEED_MAX = 200;
const int SPEED_HOLD = 200;
const int LIMITE_FRENTE = 32;
const int CURVA_MAX = 70;
const int RAMP_STEP = 6;
const int RAMP_DELAY_MS = 10;

// ===== ESTADO =====
int dEsqScan = 0, dDirScan = 0, dCen = 0;

// ===== FUNÇÕES AUX =====
int medir(NewPing &s){
  delay(15);
  int d = s.ping_cm();
  return d == 0 ? 200 : d;
}

// ===== CONTROLE DE MOTORES =====
void setLeftSpeed(int s){ motor2.setSpeed(constrain(s,0,255)); motor3.setSpeed(constrain(s,0,255)); }
void setRightSpeed(int s){ motor1.setSpeed(constrain(s,0,255)); motor4.setSpeed(constrain(s,0,255)); }

void runLeft(uint8_t m){ motor2.run(m); motor3.run(m); }
void runRight(uint8_t m){ motor1.run(m); motor4.run(m); }

void pararTudo(){
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
}

void rampTo(int target){
  int start = BASE_SPEED_MIN;
  if(target < start) target = start;
  for(int v = start; v <= target; v += RAMP_STEP){
    setLeftSpeed(v);
    setRightSpeed(v);
    runLeft(BACKWARD);
    runRight(BACKWARD);
    delay(RAMP_DELAY_MS);
  }
}

void moverFrenteVel(int velLeft, int velRight){
  setLeftSpeed(velLeft);
  setRightSpeed(velRight);
  runLeft(BACKWARD);
  runRight(BACKWARD);
}

void marchaRe(int dur_ms = 250){
  setLeftSpeed(BASE_SPEED_MIN);
  setRightSpeed(BASE_SPEED_MIN);
  runLeft(FORWARD);
  runRight(FORWARD);
  delay(dur_ms);
  pararTudo();
}

void virarEsquerda(int dur_ms = 260){
  setLeftSpeed(SPEED_HOLD);
  setRightSpeed(SPEED_HOLD);
  runLeft(FORWARD);
  runRight(BACKWARD);
  delay(dur_ms);
  pararTudo();
}

void virarDireita(int dur_ms = 260){
  setLeftSpeed(SPEED_HOLD);
  setRightSpeed(SPEED_HOLD);
  runLeft(BACKWARD);
  runRight(FORWARD);
  delay(dur_ms);
  pararTudo();
}

// ===== SERVO SCAN =====
void scanComServo(){
  servoRadar.write(ANG_LEFT); delay(180); dEsqScan = medir(sonarCentro);
  servoRadar.write(ANG_RIGHT); delay(180); dDirScan = medir(sonarCentro);
  servoRadar.write(ANG_CENTER); delay(60); dCen = medir(sonarCentro);
}

// =============================================================
//               LÓGICA PRINCIPAL — VERSÃO 2.4
//        Preferência de giro implementada abaixo
// =============================================================
void loopLogica(){
  int front = medir(sonarCentro);
  int lateralEsq = medir(sonarEsq);
  int lateralDir = medir(sonarDir);

  // ===== DESVIO LATERAL =====
  if(lateralEsq < 20){ virarDireita(200); return; }
  if(lateralDir < 20){ virarEsquerda(200); return; }

  // ===== CAMINHO LIVRE =====
  if(front > LIMITE_FRENTE){
    rampTo(BASE_SPEED_MAX);

    int erro = lateralDir - lateralEsq;
    int absErro = abs(erro);
    int ganho = map(absErro, 0, 50, 0, CURVA_MAX);
    ganho = constrain(ganho, 0, CURVA_MAX);

    int velLeft = BASE_SPEED_MAX;
    int velRight = BASE_SPEED_MAX;

    if(erro > 3){
      velLeft  -= ganho;
      velRight += ganho;
    }
    else if(erro < -3){
      velLeft  += ganho;
      velRight -= ganho;
    }

    moverFrenteVel(constrain(velLeft,90,255), constrain(velRight,90,255));
    return;
  }

  // ===== OBSTÁCULO À FRENTE =====
  pararTudo();
  scanComServo();

  bool esquerdaBoa = dEsqScan > LIMITE_FRENTE;
  bool direitaBoa  = dDirScan > LIMITE_FRENTE;

  // ===== PRIMEIRO SEGUE A PREFERÊNCIA DE GIRO =====
  if(PREFERENCIA_GIRO == 0){   // Preferir ESQUERDA
    if(esquerdaBoa){
      virarEsquerda(280);
      return;
    }
    if(direitaBoa){
      virarDireita(280);
      return;
    }
  }
  else {                       // Preferir DIREITA
    if(direitaBoa){
      virarDireita(280);
      return;
    }
    if(esquerdaBoa){
      virarEsquerda(280);
      return;
    }
  }

  // ===== SE NENHUMA DAS DUAS ESTÁ BOA → RE AGE E DECIDE =====
  marchaRe(300);

  if(dEsqScan > dDirScan) virarEsquerda(300);
  else if(dDirScan > dEsqScan) virarDireita(300);
  else {
    if(PREFERENCIA_GIRO==0) virarEsquerda(300);
    else virarDireita(300);
  }

  if(medir(sonarCentro) < LIMITE_FRENTE){
    marchaRe(350);
  }
}

// ===== SETUP & LOOP =====
void setup(){
  Serial.begin(115200);
  servoRadar.attach(SERVO_PIN);
  servoRadar.write(ANG_CENTER);
  pararTudo();
  randomSeed(analogRead(A5));
}

void loop(){
  loopLogica();
  delay(20);
}
