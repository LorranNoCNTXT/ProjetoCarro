const int motor1_in1 = 2;
const int motor1_in2 = 4;
const int motor1_ena = 10;

const int motor2_in3 = 7;
const int motor2_in4 = 8;
const int motor2_enb = 11;

const int sensor1_pin = A0;
const int sensor2_pin = A1;
const int sensor3_pin = A2;
const int sensor4_pin = A3;
const int sensor5_pin = A4;

int velocidadeBase = 90;
int velocidadeAjusteLenta = 50;
int velocidadeCurvaForte = 120;

int velocidadeArrancada = 180;
int tempoArrancada = 70;

int velocidadeRe = 100;
int tempoReGrande = 400;

int compensacaoMotorEsquerdo = 5;

bool isMoving = false;

void motorEsquerdoFrente(int velocidade) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  analogWrite(motor1_ena, velocidade);
}

void motorEsquerdoTras(int velocidade) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  analogWrite(motor1_ena, velocidade);
}

void motorDireitoFrente(int velocidade) {
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  analogWrite(motor2_enb, velocidade);
}

void motorDireitoTras(int velocidade) {
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, HIGH);
  analogWrite(motor2_enb, velocidade);
}

void pararMotores() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, LOW);
  analogWrite(motor1_ena, 0);
  analogWrite(motor2_enb, 0);
}

void darReParaRecentralizar(int velocidade, int tempo) {
  motorEsquerdoTras(velocidade);
  motorDireitoTras(velocidade);
  delay(tempo);
  pararMotores();
  isMoving = false;
  Serial.println("Manobra de Ré para Recentralizar.");
}

void setup() {
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor1_ena, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(motor2_enb, OUTPUT);

  pinMode(sensor1_pin, INPUT);
  pinMode(sensor2_pin, INPUT);
  pinMode(sensor3_pin, INPUT);
  pinMode(sensor4_pin, INPUT);
  pinMode(sensor5_pin, INPUT);

  Serial.begin(9600);
  Serial.println("Iniciando seguidor de linha com lógica simples...");

  pararMotores();
  delay(1000);
}

void loop() {
  int s1 = digitalRead(sensor1_pin);
  int s2 = digitalRead(sensor2_pin);
  int s3 = digitalRead(sensor3_pin);
  int s4 = digitalRead(sensor4_pin);
  int s5 = digitalRead(sensor5_pin);

  int current_vel_esq = 0;
  int current_vel_dir = 0;

  bool seesLine = (s1 == LOW || s2 == LOW || s3 == LOW || s4 == LOW || s5 == LOW);

  if (!seesLine) {
    Serial.println("Perdeu a linha! Dando ré para recentralizar.");
    darReParaRecentralizar(velocidadeRe, tempoReGrande);
  } else {
    if (!isMoving) {
      motorEsquerdoFrente(velocidadeArrancada);
      motorDireitoFrente(velocidadeArrancada);
      delay(tempoArrancada);
      isMoving = true;
    }

    if (s1 == LOW) {
      motorEsquerdoFrente(0);
      motorDireitoFrente(velocidadeCurvaForte);
      current_vel_esq = 0;
      current_vel_dir = velocidadeCurvaForte;
    } else if (s2 == LOW) {
      motorEsquerdoFrente(velocidadeAjusteLenta);
      motorDireitoFrente(velocidadeBase);
      current_vel_esq = velocidadeAjusteLenta;
      current_vel_dir = velocidadeBase;
    } else if (s3 == LOW) {
      motorEsquerdoFrente(velocidadeBase);
      motorDireitoFrente(velocidadeBase);
      current_vel_esq = velocidadeBase;
      current_vel_dir = velocidadeBase;
    } else if (s4 == LOW) {
      motorEsquerdoFrente(velocidadeBase);
      motorDireitoFrente(velocidadeAjusteLenta);
      current_vel_esq = velocidadeBase;
      current_vel_dir = velocidadeAjusteLenta;
    } else if (s5 == LOW) {
      motorEsquerdoFrente(velocidadeCurvaForte);
      motorDireitoFrente(0);
      current_vel_esq = velocidadeCurvaForte;
      current_vel_dir = 0;
    } else {
      motorEsquerdoFrente(velocidadeBase);
      motorDireitoFrente(velocidadeBase);
      current_vel_esq = velocidadeBase;
      current_vel_dir = velocidadeBase;
    }

    current_vel_esq += compensacaoMotorEsquerdo;
    current_vel_esq = constrain(current_vel_esq, 0, 255);

    motorEsquerdoFrente(current_vel_esq);
    motorDireitoFrente(current_vel_dir);
  }

  Serial.print("S1:");
  Serial.print(s1);
  Serial.print(" S2:");
  Serial.print(s2);
  Serial.print(" S3:");
  Serial.print(s3);
  Serial.print(" S4:");
  Serial.print(s4);
  Serial.print(" S5:");
  Serial.print(s5);
  Serial.print(" | seesLine:");
  Serial.print(seesLine);
  Serial.print(" | isMoving:");
  Serial.print(isMoving);
  Serial.print(" | Vel Esq:");
  Serial.print(current_vel_esq);
  Serial.print(" | Vel Dir:");
  Serial.print(current_vel_dir);
  Serial.println();

  delay(10);
}