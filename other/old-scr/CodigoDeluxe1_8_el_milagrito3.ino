#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <QuadratureEncoder.h>   // Libreria para el encoder

// Configuración de pines y parámetros ---------------------------------------------
#define TRIGGER_PIN1  A3   // Pin para el sensor ultrasonico 1 de la Right
#define ECHO_PIN1     A2  // Pin para el sensor ultrasonico 1 de la Right
#define TRIGGER_PIN2  A0  // Pin para el sensor ultrasonico 2 de la Left
#define ECHO_PIN2     A1  // Pin para el sensor ultrasonico 2 de la Left
#define TRIGGER_PIN3  A4  // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     A5 // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  9   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     13   // Pin para el sensor ultrasonico 4 Rear
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)

// Pin del botron
const int buttonPin = 9;  // the number of the pushbutton pin

// --- ENCODER DE CUADRATURA ---
// Pines del encoder proporcionados por el usuario (A8 y A9)
#define ENCODER_PIN_A  20   // Pin para el encoder A
#define ENCODER_PIN_B  21   // Pin para el encoder B

#define SERVO_PIN     10  // Pin para el servo


// Objeto Encoder usando la librería QuadratureEncoder.h
Encoders encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Parámetros del encoder y la rueda
volatile long encoderTicks = 0;   // Contador de pulsos del encoder (lectura directa de encoder.getEncoderCount())
float wheelDiameter = 0.07;       // Diámetro de la rueda en metros (ej. 7 cm = 0.07 m). ¡AJUSTA ESTO!
float wheelCircumference = wheelDiameter * PI; // Circunferencia de la rueda en metros
int ticksPerRevolution = 700;    // Pulsos del encoder por revolución de la rueda (¡MUY IMPORTANTE: CONSULTA EL DATASHEET DE TU ENCODER!)
long prevEncoderTicks = 0; // Ticks previos del encoder
float totalDistanceTravelledCm = 0.0; // Distancia total recorrida en cm


AF_DCMotor motor(3);      // Motor conectado al puerto M3
Servo myServo;            // Objeto servo

// Declarando ultrasonicos---------------------------------------
NewPing sonarRight(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar1
NewPing sonarLeft(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar2
NewPing sonarFront(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar3
NewPing sonarRear(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar4

unsigned int distanceRight, distanceLeft, distanceFront, distanceRear;   //Distancia en cm


//VARIABLES PARA EL MUESTREO DE LOS ULTRASONICOS
int cantMuestras=5; // Número de muestreos por cada sensor 
int delayUltrasonicos= 10; // Intervalo entre pings de un mismo sensor (ms)



enum CarState {
 INICIAL,   // configuracion inicial del carrito
 STRAIGHT,   // Conducción recto, intentando mantenerse centrado
 DECIDIR_SENTIDO, // ESTA FUNCION LA VAMOS A USAR PARA DECIDIR EN QUE SENTIDO VAMOS A GIRAR
 GIROLEFT,   // cuando encuentro el final del pasillo decido girar a la izquierda
 GIRORIGHT,    // cuando encuentro el final del pasillo decido girar a la derecha
 LEFT,   // Girando a la izquierda
 RIGHT,   // Girando a la derecha
 STOPPED,   // El carrito esta parado
 CORRECCION,    // Cuando halla una esquina gira
 SENTIDO
};
CarState currentState = INICIAL;   // Estado inicial del carro

// --- PARÁMETROS DE NAVEGACIÓN Y DECISIÓN ---
int objetoDelante = 95; //Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int paredCerca = 25; // Distancia para considerar que una pared lateral está "cerca"
int paredLejos = 40; //Distancia para considerar que una pared lateral está "lejos" o el pasillo "abierto"
int carritoCentrado = 50; //Distancia deseada a las paredes laterales para centrarse
int toleranciaPared = 0;  //Tolerancia para la corrección lateral
int pasilloAbierto = 100; // Variable para decidir si un pasillo esta abierto o no.
int pasilloCerrado = 80; // Variable para decidir si un pasillo esta cerrado o no.


// Variable para el control de vueltas
int vueltas = 0;

int servoIzq = 115;   // Determinar el estado basado en la posición del servomotor
int servoCen = 80;   // Determinar el estado basado en la posición del servomotor
int servoDer = 50;   // Determinar el estado basado en la posición del servomotor

int sentido = 0;   // Determinar la variable de la posición
int Switch = 0;   // Determinar variable del case

int buttonState = 0;  // variable for reading the pushbutton status


// Función para definir los datos
void setup() {   //  función que solo se ejecutará una sola vez al inicio del programa

  Serial.begin(9600);   // Iniciando comunicación serial para depuración
  Serial.println("--- Iniciando Carrito Autonomo WRO Futuros Ingenieros ---");   // Imprimir en el serial

  // Adjuntar el servomotor al pin y centrarlo
  myServo.attach(SERVO_PIN);   // Adjuntar servo al pin 29
  myServo.write(servoCen);   // Posición inicial del servo
  delay(100);

  // Configurar el motor de tracción (detenido al inicio)
  motor.setSpeed(0);    // Velocidad del motor (0-255)
  motor.run(RELEASE);   // Libera el moto para que pueda girar libremente si no está activo

  // Inicializar el contador del encoder a cero
  encoder.setEncoderCount(0); // Reinicia el contador del encoder a 0
  Serial.println("Sistema Configurado");   // Mensaje
  encoderTicks = 0;   // Variable para los Ticks
  prevEncoderTicks = 0;   // Variable para los Ticks
  totalDistanceTravelledCm = 0.0; // Reiniciar la distancia total también
  
}

// --- BUCLE PRINCIPAL DEL PROGRAMA ---
void loop() {   //  función que se ejecutará en bucle durante todo el programa
 //Actualizar lecturas de sensores y encoder en cada ciclo
readUltrasonicSensors();
updateEncoderDistance();
buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {

  // Lógica optimizada usando variable Switch y switch-case
switch (currentState){

  case INICIAL:
    centrado();
    readUltrasonicSensors();
    sentido == 0;

    avanza(170);

    //aqui quiero que el carrito comience a sensar las paredes y calcule el tamaño total del pasillo para luego ir ajustandose

    // Distancia izquierda + distancia derecha + tamaño de carrito = tamaño del pasillo. la mitad del pasillo estariamos centrados
    // Prioridad 1: Detección de pared frontal -> pasar a punto de decisión
   if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE && sentido == 0) {
      detenido(); // Detener para la decisión
      currentState = DECIDIR_SENTIDO;
      break; // Salir del switch para procesar el nuevo estado
    }



    //detenido();
    break;

  case STRAIGHT:
    avanza(220);
    delay(1000);
    //detenido();


   if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE && sentido != 0) {
      //detenido(); // Detener para la decisión
      currentState = SENTIDO;
      break; // Salir del switch para procesar el nuevo estado
    }
    // Prioridad 2: Corrección lateral para mantenerse centrado
     if (distanceRight != MAX_DISTANCE && distanceLeft != MAX_DISTANCE) {
       if (distanceLeft < paredCerca - toleranciaPared) {
         // Demasiado cerca de la izquierda, girar un poco a la derecha
         straightDer();
         } else if (distanceRight < paredCerca - toleranciaPared) {
        // Demasiado lejos de la izquierda, girar un poco a la izquierda
         straightIzq();
         } else {
          // Dentro de la tolerancia, ir recto
         centrado();
         }
       } else {
        // Si un sensor lateral no detecta pared, puede ser una intersección grande o final de pasillo.
        //En este caso, seguir recto (centrado) por defecto o hasta que el frontal detecte.
        centrado();
      }

    break;

  case DECIDIR_SENTIDO:


    if (distanceLeft < distanceRight ){
    // Si la derecha está libre (distancia > pasilloAbierto) Y la izquierda está bloqueada
      sentido = 1;
       currentState = GIRORIGHT;
       break;
    }

    if ( distanceRight <  distanceLeft ){
    // Si la izquierda está libre (distancia > pasilloAbierto) Y la derecha está bloqueada
          sentido = 2;
          currentState = GIROLEFT;
          break;
     }

    break;

//Ejecuta la secuencia de giro a la izquierda
  case GIROLEFT:    
   if (vueltas != 12) {    
    paredizq();
    vueltas += 1;
    //currentState = STRAIGHT; // Vuelve a navegar recto después del giro
    currentState = STRAIGHT;
    } else if (vueltas = 12){
      avanza(200);
      delay(125);
      currentState = STOPPED;
    }
    break;

// Ejecuta la secuencia de giro a la derecha
   case GIRORIGHT:
   if (vueltas != 12) {    
    paredder();
    vueltas += 1;
    currentState = STRAIGHT; 
    } else if (vueltas = 12){
      avanza(200);
      delay(125);
      currentState = STOPPED;
    }
    break;

   case SENTIDO:
   if (sentido == 1) {
      detenido();
      currentState = GIRORIGHT;
   }
    else if (sentido == 2) {
      detenido();
      currentState = GIROLEFT;
   }
   break;

  case STOPPED:
    detenido();


  default:
  detenido();
  break; 
}
}
}



 
  

// Función para mover el motor hacia delante
void avanza(int speed) {   // Definimos nombre del void
 motor.setSpeed(speed);   // Establece la velocidad (0-255)
 motor.run(FORWARD);   // Establece la dirección hacia adelante
}

// Función para mover el motor hacia atrás
void retrocede(int speed) {   // Definimos nombre del void
  motor.setSpeed(speed);   // Establece la velocidad (0-255)
  motor.run(BACKWARD);   // Establece la dirección hacia atrás
}

// Función para parar
void detenido() {   // Definimos nombre del void
    motor.run(RELEASE);   // Se para
}

// Función para girar a la derecha
void girarder() {   // Definimos nombre del void
  myServo.write(servoDer);   // Gira a la derecha
}

// Función para girar a la izquierda
void girarizq() {   // Definimos nombre del void
  myServo.write(servoIzq);   // Gira a la izquierda
}

// Función para ir derecho
void centrado() {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho

}

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







// Función para actualizar la distancia recorrida usando el encoder
void updateEncoderDistance() {
  // Leemos la posición actual del encoder. Esta función devuelve los ticks acumulados.
  long currentEncoderCount = encoder.getEncoderCount();
  long deltaTicks = currentEncoderCount - prevEncoderTicks; // Cambios desde la última lectura

  // Calculamos la distancia recorrida en este pequeño intervalo.
  // Usamos abs(deltaTicks) para que la distancia total siempre aumente,
  // independientemente de si el motor gira hacia adelante o hacia atrás.
  float distanceDeltaCm = (float)abs(deltaTicks) / ticksPerRevolution * wheelCircumference * 100; // * 100 para cm

  totalDistanceTravelledCm += distanceDeltaCm; // Sumar a la distancia total acumulada
  prevEncoderTicks = currentEncoderCount; // Guardar la lectura actual para el próximo cálculo
  encoderTicks = currentEncoderCount; // Actualizar la variable global de ticks para depuración
}

// Gira a la izquierda si no hay pared a la izquierda
void paredizq() {
  retrocede(200);
  delay(1000);
  girarizq();
  avanza(180);
  delay(1850);
  centrado();

  //updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Gira a la derecha si no hay pared a la derecha
void paredder() {
  retrocede(200);
  delay(1000);
  girarder();
  avanza(180);
  delay(1850);
  centrado();

  //updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Corrección derecha
void straightIzq() {
     myServo.write(115);
     avanza(120);
     delay(300);
     centrado();
    
    
}


// Corrección derecha
void straightDer() {
     Serial.print("CORRECCIÓN DELTA DERECHA");
    myServo.write(50);
    avanza(120);
    delay(300);
    centrado();

    
}
