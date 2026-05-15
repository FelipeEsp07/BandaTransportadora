// ───────── Pines ─────────
const int trigPin = 4,  echoPin  = 2;     // Ultra 1 (banda 1)
const int trigPin2 = 8, echoPin2 = 3;     // Ultra 2 (banda 2)
const int IN3 = 6;                        // Motor 1 (canal B), un sentido (IN4 a GND)
const int IN1 = 12, IN2 = 13;             // Motor 2 (canal A)
const int PWM_MOTORES = 9;
const int ldrPin = A0, sensor = A1;
const int led = 11, bombillo = 7, ventilador = 5;

// ───────── Umbrales ─────────
const int UMBRAL_LDR = 250;
const int DISTANCIA_MIN_INICIO = 1,  DISTANCIA_MAX_INICIO = 25;
const int DISTANCIA_MIN_BANDA2 = 4,  DISTANCIA_MAX_BANDA2 = 25;
const int DISTANCIA_MAX_ARRANQUE_B2 = 27;   // rango de arranque de banda 2
const int TEMP_IDEAL_MIN = 70,       TEMP_IDEAL_MAX = 95;
const int velocidad = 170;

// ───────── Tiempos ─────────
const unsigned long TIEMPO_ESPERA            = 5000;
const unsigned long TIEMPO_REVERSA_MIN       = 1500;
const unsigned long TIEMPO_REVERSA_MAX       = 6000;
const unsigned long TIEMPO_CONFIRMAR_RETIRO  = 400;
const unsigned long TIEMPO_MAX_ADELANTE_BANDA2 = 6000;

// ───────── Filtrado por lecturas consecutivas ─────────
const int LECTURAS_CONFIRMACION_FIN_BANDA1 = 3;
const int LECTURAS_PAQUETE_FUERA_BANDA2    = 3;

// ───────── Estado ─────────
int sensorValue = 0, distancia = 0, distancia2 = 0;
bool tempIdeal = false, paqueteConfirmadoEnInicio = false;
bool recorridoIniciado = false, recorridoBanda1Completado = false;
int contadorFinBanda1 = 0, contadorPaqueteFueraBanda2 = 0;
unsigned long tiempoEspera = 0, tiempoReversa = 0;
unsigned long tiempoRetiroPaquete = 0, tiempoAdelanteBanda2 = 0;
bool primerPaqueteSalioInicio = false;
bool bloqueoDobleCaja = false;

enum EstadoBanda1 { BANDA1_DETENIDA, BANDA1_CORRIENDO };
enum EstadoBanda2 { DETENIDO, ADELANTE, ESPERANDO, REVERSA };
EstadoBanda1 estadoBanda1 = BANDA1_DETENIDA;
EstadoBanda2 estadoBanda2 = DETENIDO;

// ───────── Helpers ─────────
int leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long t = pulseIn(echo, HIGH, 30000);
  return t == 0 ? 0 : t * 0.034 / 2;
}

void motor2(int a, int b) { digitalWrite(IN1, a); digitalWrite(IN2, b); }
void apagarTermicos()     { digitalWrite(led, LOW); digitalWrite(bombillo, LOW); digitalWrite(ventilador, LOW); }

void reiniciarSistema() {
  digitalWrite(IN3, LOW);
  motor2(LOW, LOW);
  apagarTermicos();
  estadoBanda1 = BANDA1_DETENIDA;
  estadoBanda2 = DETENIDO;
  paqueteConfirmadoEnInicio = recorridoIniciado = recorridoBanda1Completado = tempIdeal = false;
  tiempoEspera = tiempoReversa = tiempoRetiroPaquete = tiempoAdelanteBanda2 = 0;
  contadorFinBanda1 = contadorPaqueteFueraBanda2 = 0;
  primerPaqueteSalioInicio = false;
  bloqueoDobleCaja = false;
  Serial.println("SISTEMA REINICIADO");
}

void leerTemperaturaSoloAntesDelRecorrido() {
  // Promedio de 10 lecturas para evitar parpadeo cerca del umbral.
  long suma = 0;
  for (int i = 0; i < 10; i++) suma += analogRead(sensor);
  sensorValue = suma / 10;

  if (sensorValue > TEMP_IDEAL_MIN && sensorValue < TEMP_IDEAL_MAX) {
    // Ideal: latch tempIdeal en true.
    tempIdeal = true;
    digitalWrite(led, HIGH); digitalWrite(bombillo, LOW); digitalWrite(ventilador, LOW);
  } else if (!tempIdeal) {
    // Solo actuamos si todavía no se alcanzó el ideal.
    digitalWrite(led, LOW);
    digitalWrite(bombillo,   sensorValue >= TEMP_IDEAL_MAX ? HIGH : LOW);
    digitalWrite(ventilador, sensorValue <= TEMP_IDEAL_MIN ? HIGH : LOW);
  }
}

// ───────── Setup ─────────
void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  pinMode(IN3, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(PWM_MOTORES, OUTPUT);
  pinMode(led, OUTPUT); pinMode(bombillo, OUTPUT); pinMode(ventilador, OUTPUT);

  digitalWrite(IN3, LOW);
  motor2(LOW, LOW);
  apagarTermicos();
  analogWrite(PWM_MOTORES, 0);
}

// ───────── Loop ─────────
void loop() {
  // --- Lectura de sensores ---
  int valorLDR = analogRead(ldrPin);
  bool laserDetectaPaquete = (valorLDR < UMBRAL_LDR);
  distancia  = leerDistancia(trigPin,  echoPin);
  distancia2 = leerDistancia(trigPin2, echoPin2);
  bool ultrasonicoInicioDetectaPaquete =
        (distancia >= DISTANCIA_MIN_INICIO && distancia <= DISTANCIA_MAX_INICIO);
  bool paqueteValidoEnInicio = laserDetectaPaquete && ultrasonicoInicioDetectaPaquete;

  analogWrite(PWM_MOTORES, velocidad);

  // --- Detección de DOS PAQUETES ---
  // Después de que banda 1 arrancó, marcamos cuando el primer paquete dejó
  // de ser detectado (salió del sensor). Si luego LDR + ultrasónico vuelven
  // a detectar paquete, es un segundo paquete: paramos todo.
  if (recorridoIniciado && !paqueteValidoEnInicio) {
    primerPaqueteSalioInicio = true;
  }
  if (!bloqueoDobleCaja && recorridoIniciado &&
      primerPaqueteSalioInicio && paqueteValidoEnInicio) {
    bloqueoDobleCaja = true;
    digitalWrite(IN3, LOW);
    motor2(LOW, LOW);
    apagarTermicos();
    Serial.println("DOS PAQUETES DETECTADOS - SISTEMA DETENIDO");
  }
  if (bloqueoDobleCaja) {
    digitalWrite(IN3, LOW);
    motor2(LOW, LOW);
    apagarTermicos();

    bool inicioLibre = !paqueteValidoEnInicio;
    bool banda2Libre = (distancia2 == 0 || distancia2 > DISTANCIA_MAX_BANDA2);

    if (inicioLibre && banda2Libre) {
      Serial.println("BANDAS LIBRES - REINICIO");
      reiniciarSistema();
    }

    delay(50);
    return;
  }

  // --- Paquete retirado antes de iniciar ---
  if (paqueteConfirmadoEnInicio && !paqueteValidoEnInicio &&
      !recorridoIniciado && !recorridoBanda1Completado) {
    if (tiempoRetiroPaquete == 0) tiempoRetiroPaquete = millis();
    if (millis() - tiempoRetiroPaquete >= TIEMPO_CONFIRMAR_RETIRO) {
      Serial.println("PAQUETE RETIRADO ANTES DEL RECORRIDO - REINICIO");
      reiniciarSistema(); delay(50); return;
    }
  } else {
    tiempoRetiroPaquete = 0;
  }

  // --- Detección inicial ---
  if (paqueteValidoEnInicio && !paqueteConfirmadoEnInicio &&
      !recorridoIniciado && !recorridoBanda1Completado) {
    paqueteConfirmadoEnInicio = true;
    tempIdeal = false;
    Serial.println("PAQUETE DETECTADO - EVALUANDO TEMPERATURA");
  }

  // --- Lectura temperatura (sin exigir paqueteValido para no perder la ventana ideal) ---
  if (paqueteConfirmadoEnInicio && !recorridoIniciado) leerTemperaturaSoloAntesDelRecorrido();

  // --- Monitor serial ---
  Serial.print("LDR:");  Serial.print(valorLDR);
  Serial.print(" L:");   Serial.print(laserDetectaPaquete);
  Serial.print(" D1:");  Serial.print(distancia);
  Serial.print(" U1:");  Serial.print(ultrasonicoInicioDetectaPaquete);
  Serial.print(" PV:");  Serial.print(paqueteValidoEnInicio);
  Serial.print(" PC:");  Serial.print(paqueteConfirmadoEnInicio);
  Serial.print(" T:");   Serial.print(sensorValue);
  Serial.print(" Id:");  Serial.print(tempIdeal);
  Serial.print(" RI:");  Serial.print(recorridoIniciado);
  Serial.print(" D2:");  Serial.print(distancia2);
  Serial.print(" B1c:"); Serial.print(recorridoBanda1Completado);
  Serial.print(" eB2:"); Serial.println(estadoBanda2);

  // --- BANDA 1 ---
  if (estadoBanda1 == BANDA1_DETENIDA) {
    digitalWrite(IN3, LOW);
    if (paqueteConfirmadoEnInicio && tempIdeal &&
        !recorridoIniciado && !recorridoBanda1Completado) {
      recorridoIniciado = true;
      // LED queda encendido todo el recorrido; bombillo y ventilador apagados.
      digitalWrite(led, HIGH); digitalWrite(bombillo, LOW); digitalWrite(ventilador, LOW);
      contadorFinBanda1 = 0;
      estadoBanda1 = BANDA1_CORRIENDO;
      Serial.println("TEMPERATURA IDEAL - INICIA BANDA 1");
    }
  } else { // BANDA1_CORRIENDO
    digitalWrite(IN3, HIGH);
    bool fueraDeRango = (distancia == 0) || (distancia > DISTANCIA_MAX_INICIO);
    contadorFinBanda1 = fueraDeRango ? contadorFinBanda1 + 1 : 0;
    if (contadorFinBanda1 >= LECTURAS_CONFIRMACION_FIN_BANDA1) {
      digitalWrite(IN3, LOW);
      contadorFinBanda1 = 0;
      estadoBanda1 = BANDA1_DETENIDA;
      recorridoBanda1Completado = true;
      Serial.println("BANDA 1 COMPLETADA - BANDA 2 HABILITADA");
    }
  }

  // --- BANDA 2 ---
  if (estadoBanda2 == DETENIDO) {
    motor2(LOW, LOW);
    if (recorridoBanda1Completado &&
        distancia2 > DISTANCIA_MIN_BANDA2 && distancia2 < DISTANCIA_MAX_ARRANQUE_B2) {
      estadoBanda2 = ADELANTE;
      tiempoAdelanteBanda2 = millis();
      Serial.println("BANDA 2 INICIA ADELANTE");
    }
  } else if (estadoBanda2 == ADELANTE) {
    motor2(HIGH, LOW);
    if (distancia2 > 0 && distancia2 <= DISTANCIA_MIN_BANDA2) {
      motor2(LOW, LOW);
      tiempoEspera = millis();
      estadoBanda2 = ESPERANDO;
      Serial.println("BANDA 2 LLEGA AL FINAL - ESPERANDO");
    } else if (millis() - tiempoAdelanteBanda2 >= TIEMPO_MAX_ADELANTE_BANDA2) {
      motor2(LOW, LOW);
      tiempoEspera = millis();
      estadoBanda2 = ESPERANDO;
      Serial.println("BANDA 2 ESPERANDO POR TIEMPO MAXIMO");
    }
  } else if (estadoBanda2 == ESPERANDO) {
    motor2(LOW, LOW);
    if (millis() - tiempoEspera >= TIEMPO_ESPERA) {
      // Voto por mayoría: 2 de 3 lecturas confirman presencia.
      int presencia = 0;
      for (int i = 0; i < 3; i++) {
        int d = leerDistancia(trigPin2, echoPin2);
        if (d > 0 && d <= DISTANCIA_MIN_BANDA2) presencia++;
        delay(15);
      }
      if (presencia >= 2) {
        tiempoReversa = millis();
        contadorPaqueteFueraBanda2 = 0;
        estadoBanda2 = REVERSA;
        Serial.println("PAQUETE PRESENTE - BANDA 2 EN REVERSA");
      } else {
        Serial.println("PAQUETE RETIRADO DURANTE ESPERA - REINICIO");
        reiniciarSistema(); delay(50); return;
      }
    }
  } else { // REVERSA
    motor2(LOW, HIGH);
    unsigned long tEnReversa = millis() - tiempoReversa;
    bool paqueteFueraDeBanda2 = (distancia2 == 0) || (distancia2 > DISTANCIA_MAX_BANDA2);
    contadorPaqueteFueraBanda2 = paqueteFueraDeBanda2 ? contadorPaqueteFueraBanda2 + 1 : 0;

    if (tEnReversa >= TIEMPO_REVERSA_MIN &&
        contadorPaqueteFueraBanda2 >= LECTURAS_PAQUETE_FUERA_BANDA2) {
      Serial.println("PAQUETE FUERA DE BANDA 2 - REINICIO");
      reiniciarSistema(); delay(50); return;
    }
  }

  delay(50);
}