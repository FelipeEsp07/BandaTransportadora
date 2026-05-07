// ───────── Pines ─────────
// Sensor ultrasónico 1
const int trigPin = 4;
const int echoPin = 2;

// Sensor ultrasónico 2
const int trigPin2 = 8;
const int echoPin2 = 3;

// Motor 1 (L298N canal B) - solo un sentido, IN4 atado a GND en hardware
const int IN3 = 6;

// Motor 2 (L298N canal A)
const int IN1 = 12;
const int IN2 = 13;

// PWM motores
const int PWM_MOTORES = 9;

// Láser + LDR
const int ldrPin = A0;

// Sensor temperatura
const int sensor = A1;

const int led = 11;
const int bombillo = 7;
const int ventilador = 5;

int sensorValue = 0;

// Velocidad
const int velocidad = 150;

// Distancias
int distancia = 0;
int distancia2 = 0;

// LDR
int valorLDR = 0;
bool laserDetectaPaquete = false;
bool ultrasonicoInicioDetectaPaquete = false;

// Temperatura
bool tempIdeal = false;

// Banderas de control
bool paqueteConfirmadoEnInicio = false;
bool recorridoIniciado = false;
bool recorridoBanda1Completado = false;

// ───────── UMBRALES ─────────
const int UMBRAL_LDR = 170;

const int DISTANCIA_MIN_INICIO = 1;
const int DISTANCIA_MAX_INICIO = 25;

const int DISTANCIA_MIN_BANDA2 = 7;
const int DISTANCIA_MAX_BANDA2 = 22;

// Temperatura ideal
const int TEMP_IDEAL_MIN = 70;
const int TEMP_IDEAL_MAX = 95;

// Tiempos
const unsigned long TIEMPO_ESPERA = 5000;

// Reversa de banda 2:
//  - MIN: tiempo mínimo de reversa antes de aceptar que el paquete salió
//         (evita reset inmediato por una lectura espurea del ultrasónico).
//  - MAX: seguridad por si el paquete se atasca y nunca despeja al sensor.
const unsigned long TIEMPO_REVERSA_MIN = 1500;
const unsigned long TIEMPO_REVERSA_MAX = 6000;

// Tiempo para confirmar que el paquete fue retirado
const unsigned long TIEMPO_CONFIRMAR_RETIRO = 400;

// Tiempo máximo avanzando en banda 2 por seguridad
const unsigned long TIEMPO_MAX_ADELANTE_BANDA2 = 6000;

// Lecturas consecutivas requeridas para confirmar fin de banda 1
// (filtra falsos positivos cuando pulseIn da timeout = 0)
const int LECTURAS_CONFIRMACION_FIN_BANDA1 = 3;
int contadorFinBanda1 = 0;

// Lecturas consecutivas para confirmar que el paquete salió de banda 2
// durante la reversa.
const int LECTURAS_PAQUETE_FUERA_BANDA2 = 3;
int contadorPaqueteFueraBanda2 = 0;

// Temporizadores
unsigned long tiempoEspera = 0;
unsigned long tiempoReversa = 0;
unsigned long tiempoRetiroPaquete = 0;
unsigned long tiempoAdelanteBanda2 = 0;

// ───────── ESTADOS BANDA 1 ─────────
enum EstadoBanda1 {
  BANDA1_DETENIDA,
  BANDA1_CORRIENDO
};

EstadoBanda1 estadoBanda1 = BANDA1_DETENIDA;

// ───────── ESTADOS BANDA 2 ─────────
enum EstadoBanda2 {
  DETENIDO,
  ADELANTE,
  ESPERANDO,
  REVERSA
};

EstadoBanda2 estadoBanda2 = DETENIDO;

// ───────── FUNCIÓN LEER ULTRASONIDO ─────────
int leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duracionPulso = pulseIn(echo, HIGH, 30000);

  if (duracionPulso == 0) {
    return 0;
  }

  int distanciaCalculada = duracionPulso * 0.034 / 2;
  return distanciaCalculada;
}

// ───────── FUNCIÓN REINICIAR SISTEMA ─────────
void reiniciarSistema() {
  // Apagar motores
  digitalWrite(IN3, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Apagar indicadores y actuadores térmicos
  digitalWrite(led, LOW);
  digitalWrite(bombillo, LOW);
  digitalWrite(ventilador, LOW);

  // Reiniciar estados
  estadoBanda1 = BANDA1_DETENIDA;
  estadoBanda2 = DETENIDO;

  // Reiniciar banderas
  paqueteConfirmadoEnInicio = false;
  recorridoIniciado = false;
  recorridoBanda1Completado = false;
  tempIdeal = false;

  // Reiniciar temporizadores y contadores
  tiempoEspera = 0;
  tiempoReversa = 0;
  tiempoRetiroPaquete = 0;
  tiempoAdelanteBanda2 = 0;
  contadorFinBanda1 = 0;
  contadorPaqueteFueraBanda2 = 0;

  Serial.println("SISTEMA REINICIADO");
}

// ───────── FUNCIÓN LEER TEMPERATURA ─────────
void leerTemperaturaSoloAntesDelRecorrido() {
  // Promedio de varias lecturas para evitar parpadeo de bombillo / ventilador
  // por ruido del ADC alrededor de los umbrales.
  const int N_LECTURAS_TEMP = 10;
  long suma = 0;
  for (int i = 0; i < N_LECTURAS_TEMP; i++) {
    suma += analogRead(sensor);
  }
  sensorValue = suma / N_LECTURAS_TEMP;

  if (sensorValue > TEMP_IDEAL_MIN && sensorValue < TEMP_IDEAL_MAX) {
    // Temperatura IDEAL: LED encendido, bombillo y ventilador apagados.
    // tempIdeal queda LATCHED en true: una vez que se alcanzó el rango
    // ideal, no se pierde aunque una lectura posterior caiga fuera por
    // ruido o porque el paquete se enfría/calienta rápido.
    tempIdeal = true;
    digitalWrite(led, HIGH);
    digitalWrite(bombillo, LOW);
    digitalWrite(ventilador, LOW);
  } else if (!tempIdeal) {
    // Solo aplicamos calefacción / enfriamiento si todavía no se ha
    // alcanzado el ideal. Si ya se alcanzó, dejamos el LED prendido y
    // los actuadores apagados a la espera de que arranque la banda.
    if (sensorValue <= TEMP_IDEAL_MIN) {
      // Lectura ADC <= 70: paquete CALIENTE -> ventilador para enfriar
      digitalWrite(led, LOW);
      digitalWrite(bombillo, LOW);
      digitalWrite(ventilador, HIGH);
    } else {
      // Lectura ADC >= 95: paquete FRÍO -> bombillo para calentar
      digitalWrite(led, LOW);
      digitalWrite(bombillo, HIGH);
      digitalWrite(ventilador, LOW);
    }
  }
}

// ───────── SETUP ─────────
void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(PWM_MOTORES, OUTPUT);

  pinMode(bombillo, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(ventilador, OUTPUT);

  // Motores apagados
  digitalWrite(IN3, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Indicadores y actuadores térmicos apagados al iniciar
  digitalWrite(led, LOW);
  digitalWrite(bombillo, LOW);
  digitalWrite(ventilador, LOW);

  analogWrite(PWM_MOTORES, 0);
}

// ───────── LOOP ─────────
void loop() {
  // ───────── LECTURA DE SENSORES ─────────
  valorLDR = analogRead(ldrPin);
  laserDetectaPaquete = (valorLDR < UMBRAL_LDR);

  distancia = leerDistancia(trigPin, echoPin);

  ultrasonicoInicioDetectaPaquete = (
    distancia >= DISTANCIA_MIN_INICIO &&
    distancia <= DISTANCIA_MAX_INICIO
  );

  distancia2 = leerDistancia(trigPin2, echoPin2);

  bool paqueteValidoEnInicio = (
    laserDetectaPaquete &&
    ultrasonicoInicioDetectaPaquete
  );

  analogWrite(PWM_MOTORES, velocidad);

  // ───────── PAQUETE RETIRADO ANTES DE INICIAR ─────────
  // Si el paquete fue confirmado, pero lo quitan antes de arrancar,
  // se espera un pequeño tiempo para evitar falsos cortes del sensor.
  if (
    paqueteConfirmadoEnInicio &&
    !paqueteValidoEnInicio &&
    !recorridoIniciado &&
    !recorridoBanda1Completado
  ) {
    if (tiempoRetiroPaquete == 0) {
      tiempoRetiroPaquete = millis();
    }

    if (millis() - tiempoRetiroPaquete >= TIEMPO_CONFIRMAR_RETIRO) {
      Serial.println("PAQUETE RETIRADO ANTES DEL RECORRIDO - REINICIO");
      reiniciarSistema();
      delay(50);
      return;
    }
  } else {
    tiempoRetiroPaquete = 0;
  }

  // ───────── DETECCIÓN INICIAL DE PAQUETE ─────────
  if (
    paqueteValidoEnInicio &&
    !paqueteConfirmadoEnInicio &&
    !recorridoIniciado &&
    !recorridoBanda1Completado
  ) {
    paqueteConfirmadoEnInicio = true;
    tempIdeal = false;

    // No se enciende ningún actuador aquí: la lectura de temperatura
    // del mismo loop decidirá bombillo / LED / ventilador.
    Serial.println("PAQUETE DETECTADO - EVALUANDO TEMPERATURA");
  }

  // ───────── LECTURA DE TEMPERATURA ─────────
  // Lee temperatura mientras el paquete esté confirmado y la banda no
  // haya iniciado. NO se exige paqueteValidoEnInicio aquí porque el
  // ventilador puede perturbar momentáneamente el láser o el ultrasónico
  // y hacernos perder la ventana en la que la temperatura cruza el rango
  // ideal. La rama de "paquete retirado" más arriba se sigue encargando
  // de detectar un retiro real (400 ms continuos sin paquete).
  if (
    paqueteConfirmadoEnInicio &&
    !recorridoIniciado
  ) {
    leerTemperaturaSoloAntesDelRecorrido();
  }

  // ───────── MONITOR SERIAL ─────────
  Serial.print("LDR: ");
  Serial.print(valorLDR);

  Serial.print(" | Laser: ");
  Serial.print(laserDetectaPaquete);

  Serial.print(" | D1: ");
  Serial.print(distancia);

  Serial.print(" | Ultra1: ");
  Serial.print(ultrasonicoInicioDetectaPaquete);

  Serial.print(" | Paquete valido: ");
  Serial.print(paqueteValidoEnInicio);

  Serial.print(" | Confirmado: ");
  Serial.print(paqueteConfirmadoEnInicio);

  Serial.print(" | Temp: ");
  Serial.print(sensorValue);

  Serial.print(" | Ideal: ");
  Serial.print(tempIdeal);

  Serial.print(" | Recorrido iniciado: ");
  Serial.print(recorridoIniciado);

  Serial.print(" | D2: ");
  Serial.print(distancia2);

  Serial.print(" | B1 completa: ");
  Serial.print(recorridoBanda1Completado);

  Serial.print(" | EstadoB2: ");
  switch (estadoBanda2) {
    case DETENIDO:  Serial.println("DETENIDO");  break;
    case ADELANTE:  Serial.println("ADELANTE");  break;
    case ESPERANDO: Serial.println("ESPERANDO"); break;
    case REVERSA:   Serial.println("REVERSA");   break;
  }

  // ───────── BANDA 1 ─────────
  switch (estadoBanda1) {
    case BANDA1_DETENIDA:
      digitalWrite(IN3, LOW);

      if (
        paqueteConfirmadoEnInicio &&
        tempIdeal &&
        !recorridoIniciado &&
        !recorridoBanda1Completado
      ) {
        recorridoIniciado = true;

        // Desde aquí ya NO se lee temperatura.
        // El LED permanece encendido durante todo el recorrido como
        // indicador de que el paquete fue procesado en condiciones ideales.
        // Solo bombillo y ventilador se apagan.
        digitalWrite(led, HIGH);
        digitalWrite(bombillo, LOW);
        digitalWrite(ventilador, LOW);

        contadorFinBanda1 = 0;
        estadoBanda1 = BANDA1_CORRIENDO;

        Serial.println("TEMPERATURA IDEAL - INICIA BANDA 1");
      }

      break;

    case BANDA1_CORRIENDO: {
      digitalWrite(IN3, HIGH);

      // Una lectura fuera de rango (incluyendo distancia==0 por timeout
      // del pulseIn) no basta: requerimos varias consecutivas para
      // evitar falsos positivos de un eco perdido.
      bool fueraDeRango = (distancia == 0) || (distancia > DISTANCIA_MAX_INICIO);

      if (fueraDeRango) {
        contadorFinBanda1++;
      } else {
        contadorFinBanda1 = 0;
      }

      if (contadorFinBanda1 >= LECTURAS_CONFIRMACION_FIN_BANDA1) {
        digitalWrite(IN3, LOW);

        contadorFinBanda1 = 0;
        estadoBanda1 = BANDA1_DETENIDA;
        recorridoBanda1Completado = true;

        Serial.println("BANDA 1 COMPLETADA - BANDA 2 HABILITADA");
      }

      break;
    }
  }

  // ───────── BANDA 2 ─────────
  switch (estadoBanda2) {
    case DETENIDO:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      // Solo arranca si banda 1 ya terminó y el paquete está dentro
      // del rango de "entrada" de banda 2 (entre 7 y 22 cm del sensor).
      if (
        recorridoBanda1Completado &&
        distancia2 > DISTANCIA_MIN_BANDA2 &&
        distancia2 < DISTANCIA_MAX_BANDA2
      ) {
        estadoBanda2 = ADELANTE;
        tiempoAdelanteBanda2 = millis();

        Serial.println("BANDA 2 INICIA ADELANTE");
      }

      break;

    case ADELANTE:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      // Caso normal: el paquete llega cerca del sensor de fin.
      if (distancia2 > 0 && distancia2 <= DISTANCIA_MIN_BANDA2) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        tiempoEspera = millis();
        estadoBanda2 = ESPERANDO;

        Serial.println("BANDA 2 LLEGA AL FINAL - ESPERANDO");
      }
      // Seguridad: si nunca llega a <= 7 cm en TIEMPO_MAX, también espera.
      // Usa else if para no setear tiempoEspera dos veces.
      else if (millis() - tiempoAdelanteBanda2 >= TIEMPO_MAX_ADELANTE_BANDA2) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        tiempoEspera = millis();
        estadoBanda2 = ESPERANDO;

        Serial.println("BANDA 2 ESPERANDO POR TIEMPO MAXIMO");
      }

      break;

    case ESPERANDO: {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      // Al cumplirse los 5 s: si el paquete SIGUE cerca del sensor de fin
      // se da reversa. Si alguien lo retiró durante la espera, se considera
      // recorrido finalizado y se reinicia el sistema directamente.
      if (millis() - tiempoEspera >= TIEMPO_ESPERA) {
        // 3 lecturas frescas con voto por mayoría para no decidir mal
        // por ruido de un solo eco.
        int presencia = 0;
        for (int i = 0; i < 3; i++) {
          int d = leerDistancia(trigPin2, echoPin2);
          if (d > 0 && d <= DISTANCIA_MIN_BANDA2) {
            presencia++;
          }
          delay(15);
        }

        if (presencia >= 2) {
          // Paquete sigue ahí: arrancar reversa.
          tiempoReversa = millis();
          contadorPaqueteFueraBanda2 = 0;
          estadoBanda2 = REVERSA;
          Serial.println("PAQUETE PRESENTE - BANDA 2 EN REVERSA");
        } else {
          // Paquete fue retirado durante la espera: recorrido completo.
          Serial.println("PAQUETE RETIRADO DURANTE ESPERA - REINICIO");
          reiniciarSistema();
          delay(50);
          return;
        }
      }

      break;
    }

    case REVERSA: {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      unsigned long tiempoEnReversa = millis() - tiempoReversa;

      // El paquete está fuera del alcance "cercano" del sensor 2 cuando
      // distancia2 supera el rango de banda 2 o no hay eco (==0).
      bool paqueteFueraDeBanda2 =
        (distancia2 == 0) || (distancia2 > DISTANCIA_MAX_BANDA2);

      if (paqueteFueraDeBanda2) {
        contadorPaqueteFueraBanda2++;
      } else {
        contadorPaqueteFueraBanda2 = 0;
      }

      // Reset normal: hay tiempo mínimo de reversa cumplido + N lecturas
      // consecutivas confirmando que el paquete salió de banda 2.
      if (
        tiempoEnReversa >= TIEMPO_REVERSA_MIN &&
        contadorPaqueteFueraBanda2 >= LECTURAS_PAQUETE_FUERA_BANDA2
      ) {
        Serial.println("PAQUETE FUERA DE BANDA 2 - REINICIO");
        reiniciarSistema();
        delay(50);
        return;
      }

      // Seguridad: si el paquete se atasca y nunca despeja al sensor,
      // forzamos el reinicio al cumplirse TIEMPO_REVERSA_MAX.
      if (tiempoEnReversa >= TIEMPO_REVERSA_MAX) {
        Serial.println("REVERSA POR TIEMPO MAXIMO - REINICIO DE SEGURIDAD");
        reiniciarSistema();
        delay(50);
        return;
      }

      break;
    }
  }

  delay(50);
}