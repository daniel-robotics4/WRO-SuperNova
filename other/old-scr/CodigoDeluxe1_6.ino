#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <TimerOne.h>   //
#include <QuadratureEncoder.h>   // Libreria para el encoder

// Configuración de pines y parámetros ---------------------------------------------
#define TRIGGER_PIN1  31   // Pin para el sensor ultrasonico 1 de la Right
#define ECHO_PIN1     30   // Pin para el sensor ultrasonico 1 de la Right
#define TRIGGER_PIN2  19   // Pin para el sensor ultrasonico 2 de la Left
#define ECHO_PIN2     18   // Pin para el sensor ultrasonico 2 de la Left
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
  encoderTicks = 0;   // Variable para los Ticks
  prevEncoderTicks = 0;   // Variable para los Ticks
  totalDistanceTravelledCm = 0.0; // Reiniciar la distancia total también
  
  // ...existing code...
  Timer1.initialize(100000); // 100ms
  Timer1.attachInterrupt(handleUltrasonicEvents);
  // ...existing code...

}

// --- BUCLE PRINCIPAL DEL PROGRAMA ---
void loop() {   //  función que se ejecutará en bucle durante todo el programa

void loop() {
  // Leer los sensores ultrasónicos
  readUltrasonicSensors();

  // Imprimir datos para depuración
  Serial.print("F:"); Serial.print(distanceFront);
  Serial.print("cm | R:"); Serial.print(distanceRight);
  Serial.print("cm | L:"); Serial.print(distanceLeft);
  Serial.print("cm | B:"); Serial.print(distanceRear);
  Serial.println(" | Estado: ");

// Después de leer sensores o donde prefieras...
Serial.print("Encoder ticks: ");
Serial.println(encoder.getEncoderCount());

avanza(200);

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
  millis(10); // Pequeña pausa entre pings para evitar ecos cruzados
  distanceRight = sonarRight.ping_cm();
  millis(10);
  distanceLeft = sonarLeft.ping_cm();
  millis(10);
  distanceRear = sonarRear.ping_cm();
  millis(10);

  // NewPing devuelve 0 si no se detecta eco o si está fuera de rango.
  // Interpretamos 0 como la distancia máxima para que la lógica de navegación no se confunda
  // con un 0 real (que significaría estar pegado a una pared).
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRear == 0) distanceRear = MAX_DISTANCE;
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
  girarizq();
  millis(1000);
  centrado();
updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Gira a la derecha si no hay pared a la derecha
void paredder() {
  Serial.print("IF Right COMFIRMADO");
  girarder();
  millis(1000);
  centrado();
updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Corrección frontal bandera 1
void correccionFrontalBandera1() {  
      Serial.print("CORRECCIÓN Front BANDERA 1 COMFIRMADO");
      retrocede(100);
      delay(1000);
      girarizq();
updateEncoderDistance();
}

// Corrección frontal bandera 2
void correccionFrontalBandera2() {
      Serial.print("CORRECCIÓN Front BANDERA 2 COMFIRMADO");
      retrocede(100);
      delay(1000);
      girarder();
updateEncoderDistance();
}

// Corrección derecha
void correccionDerecha() {
      Serial.print("CORRECCIÓN Right COMFIRMADO");
      girarder();
updateEncoderDistance();
}

// Corrección izquierda
void correccionIzquierda() {
      Serial.print("CORRECCIÓN Left COMFIRMADO");
      girarizq();
updateEncoderDistance();
}

void handleUltrasonicEvents() {
  readUltrasonicSensors();

  if ((distanceRight > 100) && (distanceFront < 10)){
    paredder();
    return;
  }
  if ((distanceLeft > 100)  && (distanceFront < 10)){
    paredizq();
    return;
  }
  if (distanceRight < 5) {
    correccionIzquierda();
    return;
  }
  if (distanceLeft < 5) {
    correccionDerecha();
    return;
  }
  if ((distanceFront < 10) && bandera == 1) {
    correccionFrontalBandera1();
    return;
  }
  if ((distanceFront < 10) && bandera == 2) {
    correccionFrontalBandera2();
    return;
  }
  avanza(100); // Por defecto
}