#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <QuadratureEncoder.h>   // Libreria para el encoder

// Configuración de pines y parámetros ---------------------------------------------
#define TRIGGER_PIN1  31   // Pin para el sensor ultrasonico 1 de la Right
#define ECHO_PIN1     32   // Pin para el sensor ultrasonico 1 de la Right
#define TRIGGER_PIN2  19   // Pin para el sensor ultrasonico 2 de la Left
#define ECHO_PIN2     18   // Pin para el sensor ultrasonico 2 de la Left
#define TRIGGER_PIN3  15   // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     14  // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  20   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     21   // Pin para el sensor ultrasonico 4 Rear
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)

#define SERVO_PIN     29   // Pin para el servo

AF_DCMotor motor(3);      // Motor conectado al puerto M3
Servo myServo;            // Objeto servo

// Declarando ultrasonicos---------------------------------------
NewPing sonarRight(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar1
NewPing sonarLeft(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar2
NewPing sonarFront(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar3
NewPing sonarRear(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar4

unsigned int distanceRight, distanceLeft, distanceFront, distanceRear;   //Distancia en cm

enum CarState {
 STRAIGHT,   // Conducción recto, intentando mantenerse centrado
 LEFT,   // Ajustando la dirección ligeramente a la izquierda
 RIGHT,   // Ajustando la dirección ligeramente a la izquierda
 STOPPED   // El carrito esta parado
};
CarState currentState = STRAIGHT;   // Estado inicial del carro

// Variable para el control de vueltas
int vueltas = 0;

int servoIzq = 165;   // Determinar el estado basado en la posición del servomotor
int servoCen = 145;   // Determinar el estado basado en la posición del servomotor
int servoDer = 125;   // Determinar el estado basado en la posición del servomotor

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
  encoderTicks = 0;
  prevEncoderTicks = 0;
  totalDistanceTravelledCm = 0.0; // Reiniciar la distancia total también
  
}

// --- BUCLE PRINCIPAL DEL PROGRAMA ---
void loop() {   //  función que se ejecutará en bucle durante todo el programa

avanza(200);

// Leer los sensores ultrasónicos
readUltrasonicSensors();

// Imprimir datos para depuración (puedes comentar esto cuando todo funcione bien)
Serial.print("F:"); Serial.print(distanceFront);
Serial.print("cm | R:"); Serial.print(distanceRight);
Serial.print("cm | L:"); Serial.print(distanceLeft);
Serial.print("cm | B:"); Serial.print(distanceRear);
Serial.println(" | Estado: ");

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
  bandera = 1;   // Determinar el estado basado en la posición del servomotor
}

// Función para girar a la izquierda
void girarizq() {   // Definimos nombre del void
  myServo.write(servoIzq);   // Gira a la izquierda
    bandera = 2;   // Determinar el estado basado en la posición del servomotor
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
}