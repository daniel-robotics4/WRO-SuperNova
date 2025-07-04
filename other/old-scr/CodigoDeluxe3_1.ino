// --------------------------------------------- Configuración de Librerias ---------------------------------------------
#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <EEPROM.h>

// --------------------------------------------- Configuración de pines y parámetros ---------------------------------------------

// ----------------------------   PINES ----------------------------  

// ------- ULTRASONICOS ------- 
#define TRIGGER_PIN1  A3   // Pin para el sensor ultrasonico 2 de la Right
#define ECHO_PIN1     A2  // Pin para el sensor ultrasonico 2 de la Right
#define TRIGGER_PIN2  31  // Pin para el sensor ultrasonico 1 de la Left
#define ECHO_PIN2     30  // Pin para el sensor ultrasonico 1 de la Left
#define TRIGGER_PIN3  A4  // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     A5 // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  9   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     13   // Pin para el sensor ultrasonico 4 Rear


// ------- MAXIMA DISTANCIA ------- 
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)


// ------- MAXIMA DISTANCIA QUE PUEDEN LEER LOS SENSORES ULTRASONICOS ------- 
NewPing sonarRight(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar1
NewPing sonarLeft(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar2
NewPing sonarFront(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar3
NewPing sonarRear(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar4
unsigned int distanceRight, distanceLeft, distanceFront, distanceRear;   //Distancia en cm


// ------- ENCODER ------- 
#define ENCODER_PIN_A  3   // Pin para el encoder A
#define ENCODER_PIN_B  4   // Pin para el encoder B


// ------- SERVOMOTOR ------- 
#define SERVO_PIN     10  // Pin para el servo
Servo myServo;            // Objeto servo


// ------- MOTOR DC ------- 
AF_DCMotor motor(3);      // Motor conectado al puerto M3

// ----------------------------   VARIABLES ---------------------------- 

// ------- POSICIÓN DEL SERVOMOTOR ------- 
int servoL = 115;   // Determinar el estado basado en la posición del servomotor
int servoC = 80;   // Determinar el estado basado en la posición del servomotor
int servoR = 50;   // Determinar el estado basado en la posición del servomotor


// ------- PARÁMETROS DE NAVEGACIÓN Y DECISIÓN ------- 
int frontObject = 90; //Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int sense = 0;
int cantMuestras = 5; // Número de muestreos por cada sensor 
int delayUltrasonicos = 10; // Intervalo entre pings de un mismo sensor (ms)



// ------- ESTADOS DEL CARRO ------- 
enum CarState {
 START,   //
 STOPPED,   //
 SENSE,
 RIGHT,
 LEFT,
};
CarState currentState = START;



// --------------------------------------------- Configuración del programa ---------------------------------------------

void setup() {   //  función que solo se ejecutará una sola vez al inicio del programa

  Serial.begin(9600);   // Iniciando comunicación serial para depuración
  Serial.println("--- Iniciando Carrito Autonomo WRO Futuros Ingenieros ---");   // Imprimir en el serial

  // Adjuntar el servomotor al pin y centrarlo
  myServo.attach(SERVO_PIN);   // Adjuntar servo al pin 29
  myServo.write(servoC);   // Posición inicial del servo
  delay(100);

  // Configurar el motor de tracción (detenido al inicio)
  motor.setSpeed(0);    // Velocidad del motor (0-255)
  motor.run(RELEASE);   // Libera el moto para que pueda girar libremente si no está activo
  
}


// --------------------------------------------- Loop del programa ---------------------------------------------

void loop() { 

Serial.print("Estado Actual: ");
 switch (currentState) {
  case START: Serial.println("START"); break;
  case STOPPED: Serial.println("STOPPED"); break;
  case RIGHT: Serial.println("RIGHT"); break;
  case LEFT: Serial.println("LEFT"); break;
 }

switch (currentState){
 case START:
 
  servoCenter();
  readUltrasonicSensors();
  sense == 0;
  forward(150);
    
   if (distanceFront < frontObject && distanceFront != MAX_DISTANCE && sense == 0) {
      Serial.println("Pared frontal detectada. Pasando a DECISION_POINT.");
      stopped(); // Detener para la decisión
      currentState = SENSE;
      break; // Salir del switch para procesar el nuevo estado
    }
    break;

  case SENSE:
  
    if (distanceLeft < distanceRight ){
    // Si la derecha está libre (distancia > pasilloAbierto) Y la izquierda está bloqueada
      Serial.println("Decisión: Derecha abierta, Izquierda bloqueada -> Gira a la derecha.");
      sense = 1;
       currentState = RIGHT;
       break;
    }

    if ( distanceRight <  distanceLeft ){
    // Si la izquierda está libre (distancia > pasilloAbierto) Y la derecha está bloqueada
          Serial.println("Decisión: Izquierda abierta, Derecha bloqueada -> Gira a la izquierda.");
          sense = 2;
          currentState = LEFT;
          break;
    }

  case RIGHT:

  if ( distanceLeft <  69 &&  5 <  distanceLeft){
       Rbig();
       currentState = STOPPED;
    break;
    } else if ( 70 <  distanceLeft &&  distanceLeft < 100){
       Rsmall();
       currentState = STOPPED;
    break;
    }
  break;
 
 case LEFT:

  if ( distanceRight <  69 &&  5 <  distanceRight){
       Lbig();
       currentState = STOPPED;
    break;
    } else if ( 70 <  distanceRight &&  distanceRight < 100){
       Lsmall();
       currentState = STOPPED;
    break;
    }
  break;

    
  case STOPPED:
   stopped();
   break; // Salir del switch para procesar el nuevo estado
   
 }
}


// --------------------------------------------- Funciones ---------------------------------------------

// ------------------------- Basicas Del Movimiento -------------------------

// Función para mover el motor hacia delante
void forward(int speed) {   // Definimos nombre del void
 motor.setSpeed(speed);   // Establece la velocidad (0-255)
 motor.run(FORWARD);   // Establece la dirección hacia adelante
}

// Función para mover el motor hacia atrás
void backward(int speed) {   // Definimos nombre del void
  motor.setSpeed(speed);   // Establece la velocidad (0-255)
  motor.run(BACKWARD);   // Establece la dirección hacia atrás
}

// Función para parar
void stopped() {   // Definimos nombre del void
    motor.run(RELEASE);   // Se para
}

// Función para girar a la derecha
void servoRigth() {   // Definimos nombre del void
  myServo.write(servoR);   // Gira a la derecha
  sense == 1;
}

// Función para girar a la izquierda
void servoLeft() {   // Definimos nombre del void
  myServo.write(servoL);   // Gira a la izquierda
  sense == 2;
}


// Función para ir derecho
void servoCenter() {   // Definimos nombre del void
  myServo.write(servoC);   // Derecho
}

// ------------------------- LECTURA Y FILTRO DE LOS ULTRASONICOS -------------------------

void readUltrasonicSensors() {
  // NewPing tiene una función 'ping_cm()' que devuelve la distancia en cm o 0 si está fuera de rango.
  //distanceFront = sonarFront.ping_cm();
  distanceFront = filtrarUltrasonicos(sonarFront);
  delay(10); // Pequeña pausa entre pings para evitar ecos cruzados
  //distanceRight = sonarRight.ping_cm();
 distanceRight = filtrarUltrasonicos(sonarRight);

  delay(10);
  //distanceLeft = sonarLeft.ping_cm();
  distanceLeft = filtrarUltrasonicos(sonarLeft);
  delay(10);
  //distanceRear = sonarRear.ping_cm();
  distanceRear = filtrarUltrasonicos(sonarRear);
  delay(10);

  // NewPing devuelve 0 si no se detecta eco o si está fuera de rango.
  // Interpretamos 0 como la distancia máxima para que la lógica de navegación no se confunda
  // con un 0 real (que significaría estar pegado a una pared).
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRear == 0) distanceRear = MAX_DISTANCE;

  // Imprimir datos para depuración
  Serial.print("F:"); Serial.print(distanceFront);
  Serial.print("cm | R:"); Serial.print(distanceRight);
  Serial.print("cm | L:"); Serial.print(distanceLeft);
  Serial.print("cm | B:"); Serial.print(distanceRear);
  Serial.println(" | Estado: ");
}


unsigned int filtrarUltrasonicos(NewPing &sonar) {
  //#define CANT_MUESTRAS  5   // Número de muestreos por cada sensor 
  //#define DELAY_ULTRASONICOS 15
 unsigned long sum = 0;
 unsigned int minVal = MAX_DISTANCE; // Inicializar con el valor más alto posible
 unsigned int maxVal = 0; // Inicializar con el valor más bajo posible
 int validCount = 0;

 for (int i = 0; i < cantMuestras; i++) {
 unsigned int dist = sonar.ping_cm();
  
// Si la lectura es 0 (fuera de rango o error), la tratamos como MAX_DISTANCE.
// Esto es crucial para que no baje el promedio falsamente.
 if (dist == 0) {
 dist = MAX_DISTANCE;
 }
 
//Asegurarse de que la lectura sea razonable para incluirla en el min/max
// (ej. no incluir MAX_DISTANCE en minVal si de verdad estamos cerca)
// Sin embargo, para este algoritmo, incluimos todo para que el min/max actúe sobre el rango completo
 
 sum += dist;
 
 if (dist < minVal) {
 minVal = dist;
 }
 if (dist > maxVal) {
 maxVal = dist;
 }
 validCount++; // Contar que hemos tomado una lectura 
 delay(delayUltrasonicos); // Pequeña pausa entre cada ping del mismo sensor
 }

// Si no se tomó ninguna lectura válida (no debería pasar con este for)
 if (validCount == 0) {
 return MAX_DISTANCE;
 }

// Descartar la lectura más alta y la más baja
 sum -= minVal;
 sum -= maxVal;
 
 //Si NUM_SAMPLES_SIMPLE es 1 o 2, no se puede descartar min/max y promediar.
 //Por seguridad, si solo hay 1 o 2 muestras, el divisor es 'validCount'.
 //Si hay 3 o más, el divisor es 'validCount - 2'.
 int divisor = validCount;
 if (validCount >= 3) {
 divisor -= 2; // Descartamos la más alta y la más baja
 }
 
 // Evitar división por cero si por alguna razón divisor es 0
 if (divisor == 0) return MAX_DISTANCE;

 return (unsigned int)(sum / divisor);
}

// ------------------------- PASILLOS -------------------------


// -------- GIRO CORTO A LA DERECHA --------
void Rsmall() {
  Serial.print("Rsmall COMFIRMADO"); 
  backward(150);
  delay(700);
  servoRigth();
  forward(150);
  delay(2500);
  servoCenter();
}

// -------- GIRO LARGO A LA DERECHA --------
void Rbig() {
Serial.print("Rbig COMFIRMADO");
  backward(150);
  delay(1500);
  servoRigth();
  forward(150);
  delay(2500);
  servoCenter();
}

// -------- GIRO CORTO A LA IZQUIERDA --------
void Lsmall() {
  Serial.print("Rsmall COMFIRMADO"); 
  backward(150);
  delay(1000);
  servoLeft();
  forward(150);
  delay(2500);
  servoCenter();
}

// -------- GIRO LARGO A LA DERECHA --------
void Lbig() {
Serial.print("Rbig COMFIRMADO");
  backward(150);
  delay(1500);
  servoLeft();
  forward(150);
  delay(2500);
  servoCenter();
}
