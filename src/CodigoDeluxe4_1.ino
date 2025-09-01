
#include <Arduino.h>         // Librería principal de Arduino
#include <WiFi.h>            // Librería para conexión WiFi
#include <WebServer.h>
#include <Pixy2.h>           // Librería para Pixy2
#include <NewPing.h>         // Librería para sensores ultrasonicos
#include <ESP32Encoder.h>    // Librería para el manejo del encoder en ESP32
#include <ESP32Servo.h>      // Librería para el manejo de servos en ESP32


// --- Definiciones de Pixy2 ---
Pixy2 pixy;// Instancia de Pixy2

// --- Definiciones para el Motor ---
const int DIR_PIN_A = 16;                  // Pin de dirección A
const int DIR_PIN_B = 17;                  // Pin de dirección B
const int pinENB = 15;                      // Pin conectado al ENB del puente H

// --- Definiciones de Pines y Constantes para Sensores Ultrasonicos ---
#define TRIGGER_PIN_RIGHT 33                // Pin TRIGGER para el sensor ultrasonico Derecho
#define ECHO_PIN_RIGHT    25                // Pin ECHO para el sensor ultrasonico Derecho

#define TRIGGER_PIN_LEFT  27                // Pin TRIGGER para el sensor ultrasonico Izquierdo
#define ECHO_PIN_LEFT     26                // Pin ECHO para el sensor ultrasonico Izquierdo

#define TRIGGER_PIN_FRONT 13                // Pin TRIGGER para el sensor ultrasonico Frontal
#define ECHO_PIN_FRONT    14                // Pin ECHO para el sensor ultrasonico Frontal

#define TRIGGER_PIN_REAR  21                // Pin TRIGGER para el sensor ultrasonico Trasero
#define ECHO_PIN_REAR     34            // Pin ECHO para el sensor ultrasonico Trasero

Servo servo;                                // Instancia del servomotor
int servoPin = 2;                           // Pin del servomotor

int MAX_DISTANCE = 300;                     // Distancia máxima para los sensores ultrasónicos

int servoL = 115;                           // Determinar el estado basado en la posición del servomotor
int servoC = 80;                            // Determinar el estado basado en la posición del servomotor
int servoR = 50;                            // Determinar el estado basado en la posición del servomotor

int frontObject = 90;                       // Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int sense = 0;                              // Variable para el sentido de giro
int cantMuestras = 5;                       // Número de muestreos por cada sensor
int DELAY_ULTRASONICOS = 10;                // Intervalo entre pings de un mismo sensor (ms)
int wallfence = 30;                         // Estas muy cerca de la pared
int laps = 0;                               // Vueltas de cada esquina
int diameter = 66;                          // Diametro
int ppr = 1400;                             // Pulsos por revolución
int buttonState = 0;                        // Variable for reading the pushbutton status
int maximumCorrections = 0;                 // Correcciones maximas permitidas
int A = 0;
int B = 0;
unsigned long ultrasonicDisabledUntil = 1500;

// Objetos NewPing para los sensores ultrasónicos
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRear(TRIGGER_PIN_REAR, ECHO_PIN_REAR, MAX_DISTANCE);

unsigned int distanceRight, distanceLeft, distanceFront, distanceRear; // Distancia en cm

// --- Definiciones para el Encoder ---
ESP32Encoder encoder(true); // Instancia del encoder. 'true' habilita la detección de dirección por cuadratura
 // No pasamos la función de callback aquí, ya que la librería maneja el conteo.
long encoderTicks = 0;      // Variable para almacenar el conteo de ticks del encoder

// --- Variable para el estado del loop ---
String currentLoopState = "Iniciando..."; // Variable para almacenar el estado actual del loop
unsigned long lastStateChangeTime = 0;
const unsigned long STATE_CHANGE_INTERVAL = 2000; // Cambiar de estado cada 2 segundos para demostración

// --- Variables de Estado del Carro ---
enum CarState {
  START,
  SENSE,
  RIGHT,
  LEFT,
  STRAIGHT,
  STOPPED,
};
CarState currentState = START;

// --- Funciones de Pixy2 ---
String getSignatureColorName(int signature) {
  switch (signature) {
    case 1:
     Serial.print("Verde ");
      return "Verde";   // Asume que la firma 1 es Verde
    case 2:
     Serial.print("Rojo");
      return "Rojo";    // Asume que la firma 2 es Rojo
    // Añade más casos si tienes más firmas/colores entrenados
    default:
      return "Desconocido"; // Para cualquier otra firma
  }
}

// --- Funciones de Sensores Ultrasonicos ---

// Función para filtrar lecturas de ultrasonicos
unsigned int filtrarUltrasonicos(NewPing &sonar) {
    unsigned long sum = 0;
    unsigned int minVal = MAX_DISTANCE;
    unsigned int maxVal = 0;
    int validCount = 0;

    for (int i = 0; i < cantMuestras; i++) {
        unsigned int dist = sonar.ping_cm();

        if (dist == 0) {
            dist = MAX_DISTANCE;
        }

        sum += dist;

        if (dist < minVal) {
            minVal = dist;
        }
        if (dist > maxVal) {
            maxVal = dist;
        }
        validCount++;
        delay(DELAY_ULTRASONICOS);
    }

    if (validCount == 0) {
        return MAX_DISTANCE;
    }

    sum -= minVal;
    sum -= maxVal;

    int divisor = validCount;
    if (validCount >= 3) {
        divisor -= 2;
    }

    if (divisor == 0) return MAX_DISTANCE;

    return (unsigned int)(sum / divisor);
}

// Función para leer sensores ultrasónicos (solo lógica de lectura)
void readUltrasonicSensors() {
  distanceFront = filtrarUltrasonicos(sonarFront);
  distanceRight = filtrarUltrasonicos(sonarRight);
  distanceLeft = filtrarUltrasonicos(sonarLeft);
  distanceRear = filtrarUltrasonicos(sonarRear);

  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRear == 0) distanceRear = MAX_DISTANCE;
}


void servoRight() {   // Definimos nombre del void
  servo.write(servoR);   // Gira a la derecha
  delay(15);
}

void servoLeft() {   // Definimos nombre del void
  servo.write(servoL);   // Gira a la izquierda
  delay(15);
}

void servoCenter() {   // Definimos nombre del void
  servo.write(servoC);   // Derecho
  delay(15);
}



void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(DIR_PIN_A, LOW);
    digitalWrite(DIR_PIN_B, HIGH);
  } else {
    digitalWrite(DIR_PIN_A, HIGH);
    digitalWrite(DIR_PIN_B, LOW);
  }
}

void stopMotor() {
  // Stops the motor
  digitalWrite(DIR_PIN_A, LOW);
  digitalWrite(DIR_PIN_B, LOW);
  digitalWrite(pinENB, LOW); // Set pinENB to LOW to completely stop the motor
}

void MotorSpeed(bool direccion, int velocidad) {

  setMotorDirection(direccion); // Set the motor direction based on the boolean parameter
  digitalWrite(pinENB, velocidad); // Set the speed using digitalWrite
}

void EncoderForward(int encoderTicks) {

  encoder.clearCount();  

    while (encoder.getCount() < encoderTicks) {
  MotorSpeed(true, 30); // Move forward at speed 150
      delay(10); // Small pause to avoid blocking the main loop
  }
  stopMotor();
  encoder.clearCount(); 
  }

  void EncoderBackward(int encoderTicks) {

    encoder.clearCount(); 

    while (encoder.getCount() > encoderTicks) {
      MotorSpeed(false, 30); // Move backward at speed 150
      delay(10); // Small pause to avoid blocking the main loop
  }
  stopMotor();
  encoder.clearCount(); 
  }

  void Lbig() {
  // Serial.print("Lbig COMFIRMADO");
  EncoderBackward(-700);
  servoLeft();
  EncoderForward(1200);
  servoCenter();
  delay(10);
}

// -------- GIRO CORTO A LA DERECHA --------
void Rsmall() {
  EncoderBackward(-650);
  servoRight();
  EncoderForward(1100);
  servoCenter();
}

// -------- GIRO LARGO A LA DERECHA --------
void Rbig() {
  EncoderBackward(-700);
  servoRight();
  EncoderForward(1100);
  servoCenter();
}

// -------- GIRO CORTO A LA IZQUIERDA --------
void Lsmall() {
  EncoderBackward(-650);
  servoLeft();
  EncoderForward(1200);
  servoCenter();
}

void correctionL(){
  servoLeft();
  MotorSpeed(true, 120);
  delay(200);
  servoCenter();
  delay(15);
}


void correctionR(){
  servoRight();
  MotorSpeed(true, 120);
  delay(200);
  servoCenter();
  delay(15);
}


void Final() {
  stopMotor();
  servoCenter();
}


// --- Handlers del Servidor Web ---

// Handler para la página principal ("/")



void setup() {
  Serial.begin(115200);

  pixy.init(); // Inicializar la Pixy2
  Serial.println("Pixy2 inicializada.");

  pinMode(DIR_PIN_B, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(pinENB, OUTPUT); // Ensure pinENB is configured as an output

  servo.attach(servoPin, 500, 2500);


 // pixy.init();
  //Serial.println("Pixy2 inicializadaaaaaaaaaaaaaaaaa");

  // Configuración del encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Habilita pull-up internas
  encoder.attachSingleEdge(4, 22); // Conecta los pines del encoder (ej. Canal A en GPIO 4, Canal B en GPIO 5)
                                  // ¡Verifica estos pines con tu hardware!
  encoder.clearCount();           // Reinicia el contador del encoder
  Serial.println("Encoder inicializado.");


// ...existing code...

  // Inicializar el tiempo para el cambio de estado simulado
  lastStateChangeTime = millis();

}

void loop() {

  static unsigned long straightStartTime = 0;
  unsigned long elapsed = 0;
  unsigned long tiempo = millis();
 Serial.println(laps);

  switch (currentState){

    case START:
    
      Serial.println("¡SI!");
      servoCenter();
      readUltrasonicSensors();
      sense = 0;
      MotorSpeed(true, 150);
      delay(10);

      if ((distanceFront < frontObject) && (distanceFront != MAX_DISTANCE) && (sense == 0)) {
        stopMotor(); // Detener para la decisión
        currentState = SENSE;
      }
      break;

    case SENSE:

      if (distanceLeft < distanceRight) {
        sense = 2;
        currentState = RIGHT;
      } else if (distanceRight < distanceLeft) {
        sense = 1;
        currentState = LEFT;
      }
      break;

    case RIGHT:
      Serial.println("¡DERECHA!");

        if ((distanceLeft < 69) && (5 < distanceLeft)) {
          Rbig();
          delay(10);
          laps ++;
          maximumCorrections = 0;
          delay(15);
          currentState = STRAIGHT;
          break;
        } else if ((70 < distanceLeft) && (distanceLeft < 100)) {
          Rsmall();
          delay(15);
          laps ++;
          maximumCorrections = 0;
          delay(15);
          currentState = STRAIGHT;
          break;
        }
        
      break;

    case LEFT:
      Serial.println("¡IZQUIERDA!");

        if ((distanceRight < 69) && (5 < distanceRight)) {
          Lbig();
          laps ++;
          maximumCorrections = 0;
          delay(10);
          currentState = STRAIGHT;
        } else if ((70 < distanceRight) && (distanceRight < 100)) {
          Lsmall();
          laps ++;
          maximumCorrections = 0;
          delay(10);
          currentState = STRAIGHT;
        }
  
      break;

    case STRAIGHT:

if (straightStartTime == 0) {
  straightStartTime = millis();
  }

      Serial.println("recto!");
      servoCenter();
      delay(10);
      MotorSpeed(true, 150);
      readUltrasonicSensors();
      delay(10);
      Serial.println(tiempo);

      if (millis() > straightStartTime + 1200) {

              Serial.println("despues");

      if ((distanceFront < frontObject) && (sense == 1) && (50 < distanceFront)) {
          currentState = LEFT;
          straightStartTime = 0;
          delay(10);
        } 
        if ((distanceFront < frontObject) && (sense == 2) && (50 < distanceFront)) {
          currentState = RIGHT;
          straightStartTime = 0;
          delay(10);
        }
      } else {

              Serial.println("antes");


        if (11 <= laps) {
      EncoderForward(1200);
      currentState = STOPPED;
      break;
      }

        if (distanceFront < 15) {
          EncoderBackward(-1200);
        }

        if (maximumCorrections < 1) {
          if ((distanceLeft < wallfence) && (maximumCorrections < 2)) {
            maximumCorrections++;
            correctionR();
            delay(10);
          } else if ((distanceRight < wallfence) && (maximumCorrections < 2)) {
            maximumCorrections++;
            correctionL();
            delay(10);
          } else {
            servoCenter();
            delay(10);
          }
        }
      }

      break;

    case STOPPED:

      Final();
      break;

    default:

      currentState = STRAIGHT;
      delay(10);
      break;

  }
}
 





