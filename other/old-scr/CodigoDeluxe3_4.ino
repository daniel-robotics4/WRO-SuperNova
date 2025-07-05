// --------------------------------------------- Configuración de Librerias (OPEN) ---------------------------------------------
#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <move.h>
#include <QuadratureEncoder.h>

// --------------------------------------------- Configuración de Librerias (CLOSE) ---------------------------------------------

// --------------------------------------------- Configuración de pines y parámetros ---------------------------------------------

// ---------------------------- PINES (OPEN) ----------------------------

// ------- ULTRASONICOS (OPEN) ------- 
#define TRIGGER_PIN1  A3   // Pin para el sensor ultrasonico 2 de la Right
#define ECHO_PIN1     A2  // Pin para el sensor ultrasonico 2 de la Right
#define TRIGGER_PIN2  33  // Pin para el sensor ultrasonico 1 de la Left
#define ECHO_PIN2     32  // Pin para el sensor ultrasonico 1 de la Left
#define TRIGGER_PIN3  A4  // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     A5 // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  9   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     13   // Pin para el sensor ultrasonico 4 Rear
// ------- ULTRASONICOS (CLOSE) ------- 


// ------- ENCODER (OPEN) ------- 
#define ENCODER_PIN_A  20   // Pin para el encoder A
#define ENCODER_PIN_B  21  // Pin para el encoder B
Encoders encoder(ENCODER_PIN_A, ENCODER_PIN_B);  
//No eliminar este objeto de encoder, es necesario para la libreria
Encoders encoderIGNORAR(A14 , A12); // Encoder object name rightEncoder using analog pin A0 and A1
// ------- ENCODER (CLOSE) ------- 


// ------- SERVOMOTOR (OPEN) ------- 
#define SERVO_PIN     10  // Pin para el servo
Servo myServo;            // Objeto servo
// ------- SERVOMOTOR (CLOSE) ------- 


// ------- MOTOR DC (OPEN) ------- 
AF_DCMotor motor(3);      // Motor conectado al puerto M3 MOTOR HABIL
//DEFINICION DE OTROS MOTORES PARA LA LIBRERIA
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
// ------- MOTOR DC (CLOSE) ------- 


Move move(motor1, motor2, motor, motor4, encoder, encoderIGNORAR);
  

// ---------------------------- PINES (CLOSE) ----------------------------


// ----------------------------   VARIABLES (OPEN) ---------------------------- 

// ------- POSICIÓN DEL SERVOMOTOR (OPEN) ------- 
int servoL = 115;   // Determinar el estado basado en la posición del servomotor
int servoC = 80;   // Determinar el estado basado en la posición del servomotor
int servoR = 50;   // Determinar el estado basado en la posición del servomotor
// ------- POSICIÓN DEL SERVOMOTOR(CLOSE) ------- 


// ------- PARÁMETROS DE NAVEGACIÓN Y DECISIÓN (OPEN) ------- 
int frontObject = 90; //Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int sense = 0;
int cantMuestras = 5; // Número de muestreos por cada sensor 
int delayUltrasonicos = 10; // Intervalo entre pings de un mismo sensor (ms)
int wallfence = 25; // Estas muy cerca de la pared
int laps = 0; // 7
int diameter = 66; // Diametro
int ppr = 1400; //

// ------- PARÁMETROS DE NAVEGACIÓN Y DECISIÓN (CLOSE) ------- 


// ------- MAXIMA DISTANCIA (OPEN) ------- 
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)
// ------- MAXIMA DISTANCIA (CLOSE) ------- 


// ------- MAXIMA DISTANCIA QUE PUEDEN LEER LOS SENSORES ULTRASONICOS (OPEN) ------- 
NewPing sonarRight(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar1
NewPing sonarLeft(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar2
NewPing sonarFront(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar3
NewPing sonarRear(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar4
unsigned int distanceRight, distanceLeft, distanceFront, distanceRear;   //Distancia en cm
// ------- MAXIMA DISTANCIA QUE PUEDEN LEER LOS SENSORES ULTRASONICOS (CLOSE) ------- 


// ------- 


// ------- ESTADOS DEL CARRO (OPEN) ------- 
enum CarState {
 START,   //
 STOPPED,   //
 SENSE,
 RIGHT,
 LEFT,
 STRAIGHT,
};
CarState currentState = START;
// ------- ESTADOS DEL CARRO (CLOSE) ------- 


// ----------------------------   VARIABLES (CLOSE) ---------------------------- 


// --------------------------------------------- Configuración del programa (OPEN) ---------------------------------------------

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

// --------------------------------------------- Configuración del programa (CLOSE) ---------------------------------------------

// --------------------------------------------- Loop del programa (OPEN) ---------------------------------------------

void loop() { 

Serial.print("Estado Actual: ");
 switch (currentState) {
  case START: Serial.println("START"); break;
  case STOPPED: Serial.println("STOPPED"); break;
  case RIGHT: Serial.println("RIGHT"); break;
  case LEFT: Serial.println("LEFT"); break;
 }

switch (currentState){

   // ------------------------- Caso Para Elegir Sentido (OPEN) -------------------------

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
 // ------------------------- Caso Para Elegir Sentido (CLOSE) -------------------------

    
 // ------------------------- Caso Para Elegir Sentido (OPEN) -------------------------

  case SENSE:
  
    if (distanceLeft < distanceRight ){
    // Si la derecha está libre (distancia > pasilloAbierto) Y la izquierda está bloqueada
      Serial.println("Decisión: Derecha abierta, Izquierda bloqueada -> Gira a la derecha.");
      sense = 2;
       currentState = RIGHT;
       break;
    }

    if ( distanceRight <  distanceLeft ){
    // Si la izquierda está libre (distancia > pasilloAbierto) Y la derecha está bloqueada
          Serial.println("Decisión: Izquierda abierta, Derecha bloqueada -> Gira a la izquierda.");
          sense = 1;
          currentState = LEFT;
          break;
    }
 // ------------------------- Caso Para Elegir Sentido (CLOSE) -------------------------


 // ------------------------- Caso Para Gira A La Derecha En El Pasillo (OPEN) -------------------------

  case RIGHT:

if (laps != 12) {   
  
   if ( (distanceLeft <  69) &&  (5 <  distanceLeft)){
       Rbig();
       delay(15);
       laps += 1;
       readUltrasonicSensors(); // <-- Añade esto
       delay(15);
       currentState = STRAIGHT;
      break;
    } else if ( (70 <  distanceLeft) && ( distanceLeft < 100)){
       Rsmall();
       delay(15);
       laps += 1;
       readUltrasonicSensors(); // <-- Añade esto
       delay(15);
       currentState = STRAIGHT;
      break;
   } 
    } else if (laps == 12){
      forward(200);
      delay(125);
      currentState = STOPPED;
    }
break;
 // ------------------------- Caso Para Gira A La Derecha En El Pasillo (CLOSE) -------------------------


 // ------------------------- Caso Para Gira A La Izquierda En El Pasillo (OPEN) -------------------------
 
 case LEFT:

if (laps != 12) {   
 
   if ( (distanceRight <  69) &&  (5 <  distanceRight)){
//           currentState = STRAIGHT;
       Lbig();
       delay(15);
       laps += 1;
       readUltrasonicSensors(); // <-- Añade esto
       delay(15);
       currentState = STRAIGHT;
      break;
    } else if (( 70 <  distanceRight) &&  (distanceRight < 100)){
//             currentState = STRAIGHT;
       Lsmall();
       delay(15);
       laps += 1;
       readUltrasonicSensors(); // <-- Añade esto
       delay(15);
       currentState = STRAIGHT;
      break; 
   }
    } else if (laps == 12){
      forward(200);
      delay(125);
      currentState = STOPPED;
    }
break;
 // ------------------------- Caso Para Gira A La Izquierda En El Pasillo (CLOSE) -------------------------

  
 // ------------------------- Caso Para Que Avance El Robot (OPEN) -------------------------

  case STRAIGHT:

    forward(150);
    readUltrasonicSensors();
    delay(500);

   if ((distanceFront < frontObject) && (distanceFront != MAX_DISTANCE) && (sense == 1 && 40 < distanceFront)) {
      currentState = LEFT;
      break; // Salir del switch para procesar el nuevo estado
    }

    if ((distanceFront < frontObject) && (distanceFront != MAX_DISTANCE) && (sense == 2 && 40 < distanceFront)) {
      currentState = RIGHT;
      break; // Salir del switch para procesar el nuevo estado
    }

// Prioridad 2: Corrección lateral para mantenerse centrado
     if ((distanceRight != MAX_DISTANCE) && (distanceLeft != MAX_DISTANCE)) {
       if (distanceLeft < wallfence) {
         // Demasiado cerca de la izquierda, girar un poco a la derecha
         correctionR();
     } else if (distanceRight < wallfence) {
        // Demasiado lejos de la izquierda, girar un poco a la izquierda
         correctionL();
      } else {
        // Si un sensor lateral no detecta pared, puede ser una intersección grande o final de pasillo.
        //En este caso, seguir recto (centrado) por defecto o hasta que el frontal detecte.
        servoCenter();
      }
     }

// ------------------------- Caso Para Que Avance El Robot (CLOSE) -------------------------


// ------------------------- Caso Para Detener El Robot (OPEN) -------------------------
 
  case STOPPED:
   stopped();
   break; // Salir del switch para procesar el nuevo estado
   
 }
}
// ------------------------- Caso Para Detener El Robot (CLOSE) -------------------------


// --------------------------------------------- Funciones (OPEN) ---------------------------------------------

// ------------------------- Basicas Del Movimiento (OPEN) -------------------------

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
}

// Función para girar a la izquierda
void servoLeft() {   // Definimos nombre del void
  myServo.write(servoL);   // Gira a la izquierda
}


// Función para ir derecho
void servoCenter() {   // Definimos nombre del void
  myServo.write(servoC);   // Derecho
}
// ------------------------- Basicas Del Movimiento (CLOSE) -------------------------


// ------------------------- LECTURA Y FILTRO DE LOS ULTRASONICOS (OPEN) -------------------------

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
  //#define CANT_MUESTRAS  5   // Número de muestreos por cada sensor 
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
// ------------------------- LECTURA Y FILTRO DE LOS ULTRASONICOS (CLOSE) -------------------------


// ------------------------- PASILLOS (OPEN) -------------------------


// -------- GIRO CORTO A LA DERECHA (OPEN) --------
void Rsmall() {
  Serial.print("Rsmall COMFIRMADO"); 
  bool done = false;
  while (!done) {
    done = move.backward(move.mmToPulses(250, diameter, ppr));
    delay(5);
  }
  servoRigth();
  done = false;
  while (!done) {
    done = move.forward(move.mmToPulses(740, diameter, ppr));
    delay(5);
  }
  servoCenter();
}
// -------- GIRO CORTO A LA DERECHA (CLOSE)--------


// -------- GIRO LARGO A LA DERECHA (OPEN)--------
void Rbig() {
Serial.print("Rbig COMFIRMADO");
  bool done = false;
  while (!done) {
    done = move.backward(move.mmToPulses(350, diameter, ppr));
    delay(5);
  }
  servoRigth();
  done = false;
  while (!done) {
    done = move.forward(move.mmToPulses(740, diameter, ppr));
    delay(5);
  }
  servoCenter();
}
// -------- GIRO LARGO A LA DERECHA (CLOSE)--------


// -------- GIRO CORTO A LA IZQUIERDA (OPEN)--------
void Lsmall() {
  Serial.print("Rsmall COMFIRMADO"); 
  bool done = false;
  while (!done) {
    done = move.backward(move.mmToPulses(250, diameter, ppr));
    delay(5);
  }
  servoLeft();
  done = false;
  while (!done) {
    done = move.forward(move.mmToPulses(740, diameter, ppr));
    delay(5);
  }
  servoCenter();
}
// -------- GIRO CORTO A LA IZQUIERDA (CLOSE)--------


// -------- GIRO LARGO A LA DERECHA (OPEN)--------
void Lbig() {
Serial.print("Lbig COMFIRMADO");
  bool done = false;
  while (!done) {
    done = move.backward(move.mmToPulses(350, diameter, ppr));
    delay(5);
  }
  servoLeft();
  done = false;
  while (!done) {
    done = move.forward(move.mmToPulses(740, diameter, ppr));
    delay(5);
  }
  servoCenter();
}
// -------- GIRO LARGO A LA DERECHA (CLOSE)--------


// ------------------------- PASILLOS (CLOSE) -------------------------


// ------------------------- CORRECCIÓN (OPEN) -------------------------


// -------- CORRECCIÓN A LA IZQUIERDA (OPEN) --------
void correctionL(){
Serial.print("CORRECCIÓN DELTA IZQUIERD");
  servoLeft();
  forward(120);
  delay(300);
  servoCenter();
}
// -------- CORRECCIÓN A LA IZQUIERDA (CLOSE) --------


// -------- CORRECCIÓN A LA DERECHA (OPEN) --------
void correctionR(){
Serial.print("CORRECCIÓN DELTA DERECHA");
  servoRigth();
  forward(120);
  delay(300);
  servoCenter();
}
// -------- CORRECCIÓN A LA DERECHA (CLOSE) --------

// ------------------------- CORRECCIÓN (CLOSE) -------------------------
