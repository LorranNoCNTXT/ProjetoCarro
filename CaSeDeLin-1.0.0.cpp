const int motor1_in1 = 2;
const int motor1_in2 = 4;
const int int motor1_ena = 10;

const int motor2_in3 = 7;
const int motor2_in4 = 8;
const int motor2_enb = 11;

const int sensor1_pin = A0;
const int sensor2_pin = A1;
const int sensor3_pin = A2;
const int sensor4_pin = A3;
const int sensor5_pin = A4;

int velocidadeBase = 100;
int velocidadeAjuste = 50;
int velocidadeCurva = 25;

void motor1Frente(int velocidade) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  analogWrite(motor1_ena, velocidade);
}

void motor1Tras(int velocidade) {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  analogWrite(motor1_ena, velocidade);
}

void motor2Frente(int velocidade) {
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  analogWrite(motor2_enb, velocidade);
}

void motor2Tras(int velocidade) {
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

void andarFrente(int velocidade) {
  motor1Frente(velocidade);
  motor2Frente(velocidade);
}

void virarEsquerda(int velocidade_m1, int velocidade_m2) {
  motor1Frente(velocidade_m1);
  motor2Frente(velocidade_m2);
}

void virarDireita(int velocidade_m1, int velocidade_m2) {
  motor1Frente(velocidade_m1);
  motor2Frente(velocidade_m2);
}

void setup() {
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(motor1_ena, OUTPUT);
  pinMode(motor2_enb, OUTPUT);

  pinMode(sensor1_pin, INPUT);
  pinMode(sensor2_pin, INPUT);
  pinMode(sensor3_pin, INPUT);
  pinMode(sensor4_pin, INPUT);
  pinMode(sensor5_pin, INPUT);

  Serial.begin(9600);
  Serial.println("Iniciando seguidor de linha branca...");

  analogWrite(motor1_ena, velocidadeBase);
  analogWrite(motor2_enb, velocidadeBase);

  pararMotores();
  delay(1000);
}

void loop() {
  int s1_state = digitalRead(sensor1_pin);
  int s2_state = digitalRead(sensor2_pin);
  int s3_state = digitalRead(sensor3_pin);
  int s4_state = digitalRead(sensor4_pin);
  int s5_state = digitalRead(sensor5_pin);

  Serial.print("S1:"); Serial.print(s1_state);
  Serial.print(" S2:"); Serial.print(s2_state);
  Serial.print(" S3:"); Serial.print(s3_state);
  Serial.print(" S4:"); Serial.print(s4_state);
  Serial.print(" S5:"); Serial.print(s5_state);
  Serial.println();

  if (s3_state == HIGH) {
    andarFrente(velocidadeBase);
  }
  else if (s2_state == LOW && s3_state == HIGH) {
    virarDireita(velocidadeAjuste, velocidadeBase);
  }
  else if (s1_state == LOW) {
    virarEsquerda(0, velocidadeCurva);
  }
  else if (s4_state == LOW && s3_state == HIGH) {
    virarEsquerda(velocidadeBase, velocidadeAjuste);
  }
  else if (s5_state == LOW) {
    virarDireita(velocidadeCurva, 0);
  }
  else {
    pararMotores();
  }

  delay(10);
}