
#include <Arduino.h>         // Librería principal de Arduino
#include <WiFi.h>            // Librería para conexión WiFi

#include <Pixy2.h>           // Librería para Pixy2
#include <NewPing.h>         // Librería para sensores ultrasonicos
#include <ESP32Encoder.h>    // Librería para el manejo del encoder en ESP32
#include <ESP32Servo.h>      // Librería para el manejo de servos en ESP32

// --- Definiciones de Pines y Constantes para WiFi y Servidor Web ---
// --- Definiciones de Pixy2 ---
Pixy2 pixy;                               // Instancia de Pixy2

// --- Definiciones para el Motor ---
const int DIR_PIN_A = 16;                  // Pin de dirección A
const int DIR_PIN_B = 17;                  // Pin de dirección B
const int pinENB = 5;                      // Pin conectado al ENB del puente H

// --- Definiciones de Pines y Constantes para Sensores Ultrasonicos ---
#define TRIGGER_PIN_RIGHT 33                // Pin TRIGGER para el sensor ultrasonico Derecho
#define ECHO_PIN_RIGHT    25                // Pin ECHO para el sensor ultrasonico Derecho

#define TRIGGER_PIN_LEFT  27                // Pin TRIGGER para el sensor ultrasonico Izquierdo
#define ECHO_PIN_LEFT     26                // Pin ECHO para el sensor ultrasonico Izquierdo

#define TRIGGER_PIN_FRONT 12                // Pin TRIGGER para el sensor ultrasonico Frontal
#define ECHO_PIN_FRONT    14                // Pin ECHO para el sensor ultrasonico Frontal

#define TRIGGER_PIN_REAR  22                // Pin TRIGGER para el sensor ultrasonico Trasero
#define ECHO_PIN_REAR     23                // Pin ECHO para el sensor ultrasonico Trasero

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
int wallfence = 20;                         // Estas muy cerca de la pared
int laps = 0;                               // Vueltas de cada esquina
int diameter = 66;                          // Diametro
int ppr = 1400;                             // Pulsos por revolución
int buttonState = 0;                        // Variable for reading the pushbutton status
int maximumCorrections = 0;                 // Correcciones maximas permitidas

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
  STOPPED,
  SENSE,
  RIGHT,
  LEFT,
  STRAIGHT,
};
CarState currentState = START;

// --- Funciones de Pixy2 ---
String getSignatureColorName(int signature) {
  switch (signature) {
    case 1:
      return "Verde";   // Asume que la firma 1 es Verde
    case 2:
      return "Rojo";    // Asume que la firma 2 es Rojo
    // Añade más casos si tienes más firmas/colores entrenados
    default:
      return "Desconocido"; // Para cualquier otra firma
  }
}

// Función para obtener los datos de la Pixy2 y formatearlos como HTML
/*
String getPixyDataAsHtml() {
  String dataHtml = "";
  pixy.ccc.getBlocks(); // Solicitar detección de bloques

  if (pixy.ccc.numBlocks) {
    dataHtml += "<h2>Bloques detectados: " + String(pixy.ccc.numBlocks) + "</h2>";
    dataHtml += "<table border='1'><tr><th>#</th><th>Signature</th><th>Color</th><th>X</th><th>Y</th><th>Width</th><th>Height</th><th>Area</th></tr>";
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;

      dataHtml += "<tr>";
      dataHtml += "<td>" + String(i) + "</td>";
      dataHtml += "<td>" + String(pixy.ccc.blocks[i].m_signature) + "</td>";
      dataHtml += "<td>" + getSignatureColorName(pixy.ccc.blocks[i].m_signature) + "</td>";
      dataHtml += "<td>" + String(pixy.ccc.blocks[i].m_x) + "</td>";
      dataHtml += "<td>" + String(pixy.ccc.blocks[i].m_y) + "</td>";
      dataHtml += "<td>" + String(pixy.ccc.blocks[i].m_width) + "</td>";
      dataHtml += "<td>" + String(pixy.ccc.blocks[i].m_height) + "</td>";
      dataHtml += "<td>" + String(area) + "</td>";
      dataHtml += "</tr>";
    }
    dataHtml += "</table>";
  } else {
    dataHtml += "<p>No se detectaron bloques.</p>";
  }
  return dataHtml;
}
*/
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

// Función para leer sensores ultrasónicos y formatearlos como HTML
/*
String getUltrasonicDataAsHtml() {
  readUltrasonicSensors(); // Actualizar las lecturas antes de mostrarlas
  String dataHtml = "";
  dataHtml += "<h2>Distancias de Ultrasonidos</h2>";
  dataHtml += "<table border='1'>";
  dataHtml += "<tr><th>Sensor</th><th>Distancia (cm)</th></tr>";
  dataHtml += "<tr><td>Frente</td><td>" + String(distanceFront) + "</td></tr>";
  dataHtml += "<tr><td>Derecha</td><td>" + String(distanceRight) + "</td></tr>";
  dataHtml += "<tr><td>Izquierda</td><td>" + String(distanceLeft) + "</td></tr>";
  dataHtml += "<tr><td>Trasera</td><td>" + String(distanceRear) + "</td></tr>";
  dataHtml += "</table>";
  return dataHtml;
}

// --- Función para obtener el estado del loop como HTML ---
String getLoopStateAsHtml() {
  String stateHtml = "";
  stateHtml += "<h2>Estado Actual del Loop</h2>";
  stateHtml += "<p><b>Estado:</b> <span id='current-state-text'>" + currentLoopState + "</span></p>";
  return stateHtml;
}

// --- Función para obtener los ticks del encoder como HTML ---
String getEncoderDataAsHtml() {
  String encoderHtml = "";
  encoderHtml += "<h2>Ticks del Encoder</h2>";
  encoderHtml += "<table border='1'>";
  encoderHtml += "<tr><th>Encoder Ticks</th></tr>";
  encoderHtml += "<tr><td>" + String(encoderTicks) + "</td></tr>";
  encoderHtml += "</table>";
  return encoderHtml;
}
*/
void servoRigth() {   // Definimos nombre del void
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
/*
String getServoStateAsHtml() {
  String servoHtml = "";
  servoHtml += "<h2>Estado del Servo</h2>";
  servoHtml += "<table border='1'>";
  servoHtml += "<tr><th>Función</th><th>Ángulo</th></tr>";
  
  // Determinar el estado actual basado en el ángulo del servo
  int currentAngle = servo.read();
  String currentState;

  if (currentAngle == servoR) {
    currentState = "Derecha";
  } else if (currentAngle == servoL) {
    currentState = "Izquierda";
  } else if (currentAngle == servoC) {
    currentState = "Centro";
  } else {
    currentState = ".";
  }

  servoHtml += "<tr><td>Posición Actual</td><td>" + String(currentAngle) + " (" + currentState + ")</td></tr>";
  servoHtml += "<tr><td>servoRight</td><td>" + String(servoR) + "</td></tr>";
  servoHtml += "<tr><td>servoLeft</td><td>" + String(servoL) + "</td></tr>";
  servoHtml += "<tr><td>servoCenter</td><td>" + String(servoC) + "</td></tr>";
  servoHtml += "</table>";
  return servoHtml;
}
  */


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
  MotorSpeed(true, 150); // Move forward at speed 150
      delay(10); // Small pause to avoid blocking the main loop
  }
  stopMotor();
  encoder.clearCount(); 
  }

  void EncoderBackward(int encoderTicks) {

    encoder.clearCount(); 

    while (encoder.getCount() > encoderTicks) {
      MotorSpeed(false, 150); // Move backward at speed 150
      delay(10); // Small pause to avoid blocking the main loop
  }
  stopMotor();
  encoder.clearCount(); 
  }

  void Lbig() {
  // Serial.print("Lbig COMFIRMADO");
  EncoderBackward(-700);
  servoLeft();
  EncoderForward(1300);
  servoCenter();
}

// -------- GIRO CORTO A LA DERECHA --------
void Rsmall() {
  EncoderBackward(-650);
  servoRigth();
  EncoderForward(1300);
  servoCenter();
}

// -------- GIRO LARGO A LA DERECHA --------
void Rbig() {
  EncoderBackward(-700);
  servoRigth();
  EncoderForward(1300);
  servoCenter();
}

// -------- GIRO CORTO A LA IZQUIERDA --------
void Lsmall() {
  EncoderBackward(-650);
  servoLeft();
  EncoderForward(1300);
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
  servoRigth();
  MotorSpeed(true, 120);
  delay(200);
  servoCenter();
  delay(15);
}


void backwardFinal() {
 if (sense == 1) {
  Lbig();
  } else if (sense == 2) {
  Rbig();
  }
}

// --- Handlers del Servidor Web ---

// Handler para la página principal ("/")



void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Conectando a ");



  pinMode(DIR_PIN_B, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(pinENB, OUTPUT); // Ensure pinENB is configured as an output

  servo.attach(servoPin, 500, 2500);


  pixy.init();
  Serial.println("Pixy2 inicializada.");

  // Configuración del encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Habilita pull-up internas
  encoder.attachSingleEdge(4, 5); // Conecta los pines del encoder (ej. Canal A en GPIO 4, Canal B en GPIO 5)
                                  // ¡Verifica estos pines con tu hardware!
  encoder.clearCount();           // Reinicia el contador del encoder
  Serial.println("Encoder inicializado.");


// ...existing code...

  // Inicializar el tiempo para el cambio de estado simulado
  lastStateChangeTime = millis();
}

void loop() {


  readUltrasonicSensors();
  encoderTicks = encoder.getCount();
  static unsigned long straightStartTime = 0;
  unsigned long elapsed;


  switch (currentState){

 case START:

  servoCenter();
  readUltrasonicSensors();
  sense = 0;
  MotorSpeed(true, 200);
  delay(10);
    
   if ((distanceFront < frontObject) && (distanceFront != MAX_DISTANCE) && (sense == 0)) {
      stopMotor(); // Detener para la decisión
      currentState = SENSE;
      break; // Salir del switch para procesar el nuevo estado
    }
    break;

   case SENSE:
  
     if (distanceLeft < distanceRight ){
    // Si la derecha está libre (distancia > pasilloAbierto) Y la izquierda está bloqueada
    // Serial.println("Decisión: Derecha abierta, Izquierda bloqueada -> Gira a la derecha.");
      sense = 2;
      currentState = RIGHT;
      break;
    }

    if ( distanceRight <  distanceLeft ){
    // Si la izquierda está libre (distancia > pasilloAbierto) Y la derecha está bloqueada
    // Serial.println("Decisión: Izquierda abierta, Derecha bloqueada -> Gira a la izquierda.");
      sense = 1;
      currentState = LEFT;
      break;
    }

case RIGHT:

if (laps < 11) {

   if ( (distanceLeft <  70) &&  (0 <  distanceLeft)){
       laps += 1;
       Rbig();
       delay(15);
       readUltrasonicSensors();
       maximumCorrections = 0; 
       delay(15);
       currentState = STRAIGHT;
      break;
    } else if ( (70 <  distanceLeft) && ( distanceLeft < 100)){
       laps += 1;
       Rsmall();
       delay(15);
       readUltrasonicSensors();
       maximumCorrections = 0;
       delay(15);
       currentState = STRAIGHT;
      break;
   } 
    } else if (laps >= 11){
    currentState = STOPPED;
    break;
    }
break;

case LEFT:

if (laps < 11) {

   if ( (distanceRight <  70) &&  (0 <  distanceRight)){
       laps += 1;
       Lbig();
       delay(15);
       readUltrasonicSensors();
       maximumCorrections = 0;
       delay(15);
       currentState = STRAIGHT;
      break;
    } else if (( 70 <  distanceRight) &&  (distanceRight < 100)){
       laps += 1;
       Lsmall();
       delay(15);
       readUltrasonicSensors();
       maximumCorrections = 0;
       delay(15);
       currentState = STRAIGHT;
      break; 
   }
    } else if (laps >= 11){
    currentState = STOPPED;
    delay(15);
    break;
    }
break;

case STRAIGHT:

    MotorSpeed(true, 70);
    //maximumCorrections = 0;
    readUltrasonicSensors();
    delay(15);


        if ((distanceFront < frontObject) && (MAX_DISTANCE != distanceFront) && (sense == 1) && (50 < distanceFront)) {
            currentState = LEFT;
            delay(15);
            break;
        }

        if ((distanceFront < frontObject) && (MAX_DISTANCE != distanceFront) && (sense == 2) && (50 < distanceFront)) {
            currentState = RIGHT;
            delay(15);
            break;
        }
        
        if ((distanceFront < 10) && (MAX_DISTANCE != distanceFront)) {
          laps -= 1;
          MotorSpeed(false, 200);
          delay(1500);
          stopMotor();
          delay(15);
          break;
        }


        // Prioridad 2: Corrección lateral para mantenerse centrado
            if ((distanceLeft < wallfence) && (maximumCorrections < 2)) {
                // Demasiado cerca de la izquierda, girar un poco a la derecha
                maximumCorrections += 1;
                correctionR();
                delay(15);
            }
            
            if ((distanceRight < wallfence) && (maximumCorrections < 2)) {
                // Demasiado lejos de la izquierda, girar un poco a la izquierda
                maximumCorrections += 1;
                correctionL();
                delay(15);
            }
        
    break;

case STOPPED:
backwardFinal();
stopMotor();
delay(50000);  // 
break; // Salir del switch para procesar el nuevo estado

// ------------------------- Caso Para Detener El Robot (CLOSE) -------------------------

default:
currentState = STRAIGHT;
delay(15);
break;

  }
}
