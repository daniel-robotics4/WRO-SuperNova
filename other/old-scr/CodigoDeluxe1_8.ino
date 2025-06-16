#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <QuadratureEncoder.h>   // Libreria para el encoder

// Configuración de pines y parámetros ---------------------------------------------
#define TRIGGER_PIN1  19   // Pin para el sensor ultrasonico 1 de la Right
#define ECHO_PIN1     18  // Pin para el sensor ultrasonico 1 de la Right
#define TRIGGER_PIN2  31  // Pin para el sensor ultrasonico 2 de la Left
#define ECHO_PIN2     30  // Pin para el sensor ultrasonico 2 de la Left
#define TRIGGER_PIN3  15   // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     14  // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  20   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     21   // Pin para el sensor ultrasonico 4 Rear
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)

// --- ENCODER DE CUADRATURA ---
// Pines del encoder proporcionados por el usuario (A8 y A9)
#define ENCODER_PIN_A  A8   // Pin para el encoder A
#define ENCODER_PIN_B  A9   // Pin para el encoder B

#define SERVO_PIN     38  // Pin para el servo

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

enum CarState {
 INICIAL,   // configuracion inicial del carrito
 STRAIGHT,   // Conducción recto, intentando mantenerse centrado
 DECIDIR_SENTIDO, // ESTA FUNCION LA VAMOS A USAR PARA DECIDIR EN QUE SENTIDO VAMOS A GIRAR
 LEFT,   // Girando a la izquierda
 RIGHT,   // Girando a la derecha
 STOPPED,   // El carrito esta parado
 CORRECCION,    // Cuando halla una esquina gira
 GIROLEFT,   // Estas muy a la izquierda
 GIRORIGHT,    // Estas muy a la derecha
 PRUEBA,
};
CarState currentState = INICIAL;   // Estado inicial del carro

// --- PARÁMETROS DE NAVEGACIÓN Y DECISIÓN ---
int objetoDelante = 20; //Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int paredCerca = 25; // Distancia para considerar que una pared lateral está "cerca"
int paredLejos = 40; //Distancia para considerar que una pared lateral está "lejos" o el pasillo "abierto"
int carritoCentrado = 50; //Distancia deseada a las paredes laterales para centrarse
int toleranciaPared = 5;  //Tolerancia para la corrección lateral



// Variable para el control de vueltas
int vueltas = 0;

int servoIzq = 155;   // Determinar el estado basado en la posición del servomotor
int servoCen = 135;   // Determinar el estado basado en la posición del servomotor
int servoDer = 115;   // Determinar el estado basado en la posición del servomotor

int bandera = 0;   // Determinar la variable del servomotor
int Switch = 0;   // Determinar variable del case


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

// Después de leer sensores o donde prefieras...
Serial.print("Encoder ticks: ");
Serial.println(encoder.getEncoderCount());

 Serial.print("Estado Actual: ");
 switch (currentState) {
  case INICIAL: Serial.println("INICIAL"); break;
  case STRAIGHT: Serial.println("STRAIGHT"); break;
  case DECIDIR_SENTIDO: Serial.println("DECIDIR_SENTIDO"); break;
  //case EXECUTE_TURN_LEFT: Serial.println("EXECUTE_TURN_LEFT"); break;
  //case EXECUTE_TURN_RIGHT: Serial.println("EXECUTE_TURN_RIGHT"); break;
  case STOPPED: Serial.println("STOPPED"); break;
 }



  // Lógica optimizada usando variable Switch y switch-case
switch (currentState){

  case INICIAL:
    centrado();
    readUltrasonicSensors();
    

    //aqui quiero que el carrito comience a sensar las paredes y calcule el tamaño total del pasillo para luego ir ajustandose

    // Distancia izquierda + distancia derecha + tamaño de carrito = tamaño del pasillo. la mitad del pasillo estariamos centrados


    //detenido();
    Serial.println("Setup completo. Iniciando navegación...");
    currentState = STRAIGHT; // Pasa al estado de navegación
    break;

  case STRAIGHT:
    avanza(200);
    //detenido();
    // Prioridad 1: Detección de pared frontal -> pasar a punto de decisión
     if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE) {
      Serial.println("Pared frontal detectada. Pasando a DECISION_POINT.");
      //detenido(); // Detener para la decisión
      currentState = DECIDIR_SENTIDO;
      break; // Salir del switch para procesar el nuevo estado
    }

    // Prioridad 2: Corrección lateral para mantenerse centrado
     if (distanceRight != MAX_DISTANCE && distanceLeft != MAX_DISTANCE) {
       if (distanceLeft < paredCerca - toleranciaPared) {
        // Demasiado cerca de la izquierda, girar un poco a la derecha
         straightDer();
         Serial.println("Corrección: Demasiado cerca izquierda -> Derecha.");
         } else if (distanceLeft > paredLejos + toleranciaPared) {
        // Demasiado lejos de la izquierda, girar un poco a la izquierda
         straightIzq();
         Serial.println("Corrección: Demasiado lejos Izquierda -> izquierda.");
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
    //avanza(200);
    //delay(2000);
    detenido();
    break;

  case STOPPED:
    detenido();


  default:
  detenido();
  break; 
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
  bandera = 1;
}

// Función para girar a la izquierda
void girarizq() {   // Definimos nombre del void
  myServo.write(servoIzq);   // Gira a la izquierda
  bandera = 2;

}

// Función para ir derecho
void centrado() {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho
  bandera = 0;   // Determinar el estado basado en la posición del servomotor

}

void readUltrasonicSensors() {
  // NewPing tiene una función 'ping_cm()' que devuelve la distancia en cm o 0 si está fuera de rango.
  distanceFront = sonarFront.ping_cm();
  delay(10); // Pequeña pausa entre pings para evitar ecos cruzados
  distanceRight = sonarRight.ping_cm();
  delay(10);
  distanceLeft = sonarLeft.ping_cm();
  delay(10);
  distanceRear = sonarRear.ping_cm();
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
  Serial.print("IF Left COMFIRMADO");
  retrocede(150);
  delay(3400);
  avanza(200);
  girarizq();
  delay(1900);
  //updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Gira a la derecha si no hay pared a la derecha
void paredder() {
  Serial.print("IF Right COMFIRMADO");
  retrocede(150);
  delay(3000);
  avanza(200);
  girarder();
  delay(1900);
  //updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Corrección derecha
void straightDer() {
    Serial.print("CORRECCIÓN DELTA DERECHA");
    girarizq();
    delay(20);
    avanza(200);
    delay(50);
    girarder();
    delay(10);
    centrado();
    
    
}

// Corrección derecha
void straightIzq() {
    Serial.print("CORRECCIÓN DELTA IZQUIERDA");
    girarder();
    delay(20);
    avanza(200);
    delay(50);
    girarizq();
    delay(10);
    centrado();
    

}