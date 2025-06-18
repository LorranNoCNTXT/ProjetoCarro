// --- Definições de Pinos ---
// Pinos do Módulo Sensor TCRT5000 (Analógicos)
const int SENSOR_LEFT_EXTREME = A0;  // S0 na sua foto
const int SENSOR_LEFT = A1;          // S1 na sua foto (Corrigido: removido 'int' duplicado)
const int SENSOR_CENTER = A2;        // S2 na sua foto
const int SENSOR_RIGHT = A3;         // S3 na sua foto
const int SENSOR_RIGHT_EXTREME = A4; // S4 na sua foto

// Pinos da Ponte H L298N (Motores DC)
// Motor Esquerdo (Motor A)
const int ENA = 10; // Pino PWM para controle de velocidade do Motor A
const int IN1 = 2;  // Pino de direção 1 para Motor A
const int IN2 = 4;  // Pino de direção 2 para Motor A

// Motor Direito (Motor B)
const int ENB = 11; // Pino PWM para controle de velocidade do Motor B
const int IN3 = 7;  // Pino de direção 1 para Motor B
const int IN4 = 8;  // Pino de direção 2 para Motor B

// --- Constantes de Controle ---
// Valores PID (Ajuste estes valores para otimizar o desempenho do robô)
// Kp: Aumenta a agressividade da correção. Se muito alto, o robô oscila.
// Ki: Ajuda a eliminar erros persistentes. Se muito alto, pode causar sobre-ajuste lento.
// Kd: Amortece oscilações e previne sobre-ajuste. Se muito alto, pode causar instabilidade.
float Kp = 3.0; // Comece com Kp, Ki e Kd como 0 e ajuste Kp primeiro (ex: 0.5 a 2.0)
float Ki = 0.0; // Geralmente pequeno, ou 0 para começar
float Kd = 0.0; // Geralmente 0.5 a 1.5 vezes Kp

// Velocidade base dos motores quando o robô está na linha reta
const int BASE_SPEED = 150; // Velocidade base (0-255)

// Limiar de PWM mínimo para os motores começarem a andar
// Você observou que seus motores não andam abaixo de 85/255 a partir do repouso.
const int PWM_MIN_START = 85;

// Ponto de referência para a posição da linha (centro da matriz de sensores)
// Para 5 sensores com pesos 0, 1000, 2000, 3000, 4000, o centro é 2000.
const int SET_POINT = 2000;

// Limiar para detecção de perda de linha (soma total das leituras dos sensores)
// Se a soma for muito baixa, significa que todos os sensores estão no fundo preto.
// Este valor pode precisar de ajuste após a calibração.
const int LINE_LOST_THRESHOLD = 500; // Exemplo: se a soma total for menor que 500, considera-se linha perdida.

// Constantes para a recuperação de linha
const int REVERSE_SPEED = 100; // Velocidade para dar ré
const int REVERSE_DURATION = 300; // Duração da ré em milissegundos
const int SEARCH_PIVOT_SPEED = 80; // Velocidade para o padrão de busca
const int SEARCH_PIVOT_DURATION = 150; // Duração de cada pivô na busca
const int MAX_SEARCH_ATTEMPTS = 10; // Número máximo de tentativas de busca antes de parar ou tentar outra coisa

// --- Variáveis Globais ---
int sensorValues[2];   // Corrigido: Tamanho do array para 5
int calibratedMin[2];  // Corrigido: Tamanho do array para 5
int calibratedMax[2];  // Corrigido: Tamanho do array para 5

long weightedSum = 0; // Soma ponderada das leituras dos sensores
long sensorsSum = 0;  // Soma total das leituras dos sensores
int linePosition = 0; // Posição calculada da linha (0-4000)

float error = 0;       // Erro atual
float lastError = 0;   // Erro da iteração anterior (para termo derivativo)
float integral = 0;    // Soma acumulada dos erros (para termo integral)

float P_term, I_term, D_term; // Termos PID individuais
float PID_output;             // Saída total do controlador PID

int leftMotorSpeed;  // Velocidade calculada para o motor esquerdo
int rightMotorSpeed; // Velocidade calculada para o motor direito

bool isMoving = false; // Flag para rastrear se o robô está em movimento (para PWM_MIN_START)

// --- Funções de Controle de Motores ---
void setMotorDirection(int motor, int direction) {
  if (motor == 0) { // Motor Esquerdo
    if (direction == 1) { // Para frente
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (direction == -1) { // Para trás
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else { // Parar
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  } else if (motor == 1) { // Motor Direito
    if (direction == 1) { // Para frente
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (direction == -1) { // Para trás
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else { // Parar
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

void controlMotors(int leftPWM, int rightPWM, int direction) {
  // Aplica a lógica de PWM mínimo para arranque
  if (leftPWM > 0 && leftPWM < PWM_MIN_START &&!isMoving) {
    leftPWM = 0; // Se a velocidade desejada for muito baixa e o robô estiver parado, não move
  }
  if (rightPWM > 0 && rightPWM < PWM_MIN_START &&!isMoving) {
    rightPWM = 0; // Se a velocidade desejada for muito baixa e o robô estiver parado, não move
  }

  // Garante que as velocidades estejam dentro da faixa 0-255
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  // Define a direção dos motores
  setMotorDirection(0, direction); // Motor Esquerdo
  setMotorDirection(1, direction); // Motor Direito

  // Aplica os valores PWM
  analogWrite(ENA, 150);  // Corrigido: Usar leftPWM para o motor esquerdo
  analogWrite(ENB, 150); // Corrigido: Usar rightPWM para o motor direito

  // Atualiza a flag isMoving
  if (leftPWM > 0 
  || rightPWM > 0) {
    isMoving = true;
  } else {
    isMoving = false;
  }
}

// --- Funções de Sensor e Calibração ---
void calibrateSensors() {
  Serial.println("Calibrando sensores...");
  Serial.println("Mova o robô sobre a linha branca e o fundo preto por 5 segundos.");
  delay(1000); // Pequena pausa antes de começar

  // Inicializa min/max com valores extremos
  for (int i = 0; i < 5; i++) {
    calibratedMin[i] = 1023; // Valor máximo possível para leitura analógica
    calibratedMax[i] = 0;    // Valor mínimo possível para leitura analógica
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // Calibra por 5 segundos
    for (int i = 0; i < 5; i++) {
      int rawValue = analogRead(SENSOR_LEFT_EXTREME + i); // Lê o sensor
      if (rawValue < calibratedMin[i]) {
        calibratedMin[i] = rawValue;
      }
      if (rawValue > calibratedMax[i]) {
        calibratedMax[i] = rawValue;
      }
    }
    delay(20); // Pequeno atraso para leituras estáveis
  }

  Serial.println("Calibração Concluída:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min = ");
    Serial.print(calibratedMin[i]);
    Serial.print(", Max = ");
    Serial.println(calibratedMax[i]);
  }
  Serial.println("Pressione o botão Reset no Arduino para iniciar o robô.");
  while (true) {
    // Espera pelo reset para iniciar o programa principal
  }
}

void readAndNormalizeSensors() {
  weightedSum = 0;
  sensorsSum = 0;

  // Pesos para os sensores (0, 1000, 2000, 3000, 4000)
  // O sensor mais à esquerda tem peso 0, o central 2000, o mais à direita 4000.
  int sensorWeights[5] = {0, 1000, 2000, 3000, 4000};

  for (int i = 0; i < 5; i++) {
    int rawValue = analogRead(SENSOR_LEFT_EXTREME + i);

    // Normaliza a leitura do sensor para a faixa 0-1000
    // Para linha branca em fundo preto:
    // Valores altos (branco) devem ser mapeados para 1000 (ou próximo).
    // Valores baixos (preto) devem ser mapeados para 0 (ou próximo).
    // A função map() faz isso: map(value, fromLow, fromHigh, toLow, toHigh)
    // Se calibratedMin é preto (baixo) e calibratedMax é branco (alto),
    // então map(rawValue, calibratedMin[i], calibratedMax[i], 0, 1000) é o correto.
    sensorValues[i] = map(rawValue, calibratedMin[i], calibratedMax[i], 0, 1000);
    sensorValues[i] = constrain(sensorValues[i], 0, 1000); // Garante que esteja na faixa

    weightedSum += (long)sensorValues[i] * sensorWeights[i];
    sensorsSum += sensorValues[i];
  }

  if (sensorsSum == 0) {
    // Se a soma for zero, significa que todos os sensores estão no fundo preto.
    // Isso indica que a linha foi perdida.
    // A posição da linha pode ser definida para um valor que indique a última direção
    // ou um valor que acione a recuperação.
    // Para o PID, é melhor que o 'linePosition' mantenha a última direção de desvio.
    // No entanto, para a detecção de perda de linha, usaremos 'sensorsSum'.
    linePosition = SET_POINT; // Valor padrão para evitar divisão por zero, será tratado pela soma
  } else {
    linePosition = weightedSum / sensorsSum;
  }
}

// --- Funções PID ---
void pidCalculate() {
  error = SET_POINT - linePosition; // Calcula o erro: desvio do centro

  P_term = Kp * error;

  integral += error;
  // Limita o termo integral para evitar "wind-up"
  integral = constrain(integral, -2000, 2000); // Ajuste estes limites conforme necessário
  I_term = Ki * integral;

  D_term = Kd * (error - lastError);
  lastError = error;

  PID_output = P_term + I_term + D_term;

  // Limita a saída PID para um valor razoável
  PID_output = constrain(PID_output, -BASE_SPEED, BASE_SPEED); // Garante que a correção não exceda a velocidade base
}

// --- Estratégia de Recuperação de Linha Perdida ---
void recoverLostLine() {
  Serial.println("Linha Perdida! Iniciando recuperação...");

  // 1. Parar
  controlMotors(0, 0, 0); // Parar ambos os motores
  delay(500); // Pequena pausa

  // 2. Reverter
  Serial.println("Dando ré...");
  controlMotors(REVERSE_SPEED, REVERSE_SPEED, -1); // Dar ré
  delay(REVERSE_DURATION);
  controlMotors(0, 0, 0); // Parar novamente
  delay(200);

  // 3. Procurar/Recentrar (Padrão de busca oscilatório)
  Serial.println("Procurando a linha...");
  bool lineFound = false;
  int searchAttempts = 0;
  int currentPivotDirection = 1; // 1 para direita, -1 para esquerda

  while (!lineFound && searchAttempts < MAX_SEARCH_ATTEMPTS) {
    readAndNormalizeSensors();
    if (sensorsSum > LINE_LOST_THRESHOLD) { // Se a linha for encontrada
      lineFound = true;
      break;
    }

    if (currentPivotDirection == 1) { // Pivotar para a direita
      controlMotors(SEARCH_PIVOT_SPEED, -SEARCH_PIVOT_SPEED, 1); // Motor esquerdo para frente, direito para trás
    } else { // Pivotar para a esquerda
      controlMotors(-SEARCH_PIVOT_SPEED, SEARCH_PIVOT_SPEED, 1); // Motor esquerdo para trás, direito para frente
    }
    delay(SEARCH_PIVOT_DURATION);
    controlMotors(0, 0, 0); // Parar brevemente entre pivôs
    delay(50);

    currentPivotDirection *= -1; // Inverte a direção do pivô
    searchAttempts++;
  }

  if (lineFound) {
    Serial.println("Linha encontrada! Retornando ao seguimento.");
  } else {
    Serial.println("Não foi possível encontrar a linha. Robô parado.");
    // Opcional: Implementar uma estratégia de parada de emergência ou alarme
    controlMotors(0, 0, 0); // Parar completamente se a linha não for encontrada
    while(true); // Parar o programa
  }
}

// --- Função Setup ---
void setup() {
  Serial.begin(9600); // Inicia a comunicação serial para depuração

  // Configura os pinos dos motores como OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Garante que os motores estejam parados no início
  controlMotors(0, 0, 0);

  // Inicia o processo de calibração dos sensores
  calibrateSensors();
}

// --- Função Loop Principal ---
void loop() {
  readAndNormalizeSensors(); // Lê e normaliza os valores dos sensores

  // Verifica se a linha foi perdida
  if (sensorsSum < LINE_LOST_THRESHOLD) {
    recoverLostLine(); // Aciona a estratégia de recuperação
  } else {
    // Se a linha for detectada, calcula o PID e ajusta os motores
    pidCalculate();

    // Ajusta as velocidades dos motores com base na saída PID
    // Se PID_output for positivo, o robô precisa virar para a direita (diminuir motor direito, aumentar esquerdo)
    // Se PID_output for negativo, o robô precisa virar para a esquerda (aumentar motor direito, diminuir esquerdo)
    leftMotorSpeed = BASE_SPEED + PID_output;
    rightMotorSpeed = BASE_SPEED - PID_output;

    // Controla os motores com as velocidades calculadas (direção para frente)
    controlMotors(leftMotorSpeed, rightMotorSpeed, 1); // 1 para frente
  }

  // Pequeno atraso para estabilidade (pode ser ajustado ou removido para maior velocidade)
  // Um delay muito grande pode causar oscilações ou reações lentas.
  delay(10);
}