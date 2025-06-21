#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <QuadratureEncoder.h>   // Libreria para el encoder
#include <Wire.h> 
#include <MPU6050.h> 

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

MPU6050 sensor; // Objeto del giroscopio

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
 LEFT,   // Girando a la izquierda
 RIGHT,   // Girando a la derecha
 STOPPED,   // El carrito esta parado
 INICIAL,   // Sentido del carro
 ESQUINA,    // Cuando halla una esquina gira
 CORREGIR_ANGULO,   // Estas muy a la izquierda o muy a la derecha
};
CarState currentState = INICIAL;   // Estado inicial del carro

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
  Wire.begin();
  sensor.initialize();

  // Adjuntar el servomotor al pin y centrarlo
  myServo.attach(SERVO_PIN);   // Adjuntar servo al pin 29
  myServo.write(servoCen);   // Posición inicial del servo
  delay(100);

  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
  }

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

// Después de leer sensores o donde prefieras...
Serial.print("Encoder ticks: ");
Serial.println(encoder.getEncoderCount());

// Mostrar el estado actual en el monitor serial para depuración
  if (currentState == STRAIGHT) Serial.println("STRAIGHT");
  else if (currentState == LEFT) Serial.println("LEFT");
  else if (currentState == RIGHT) Serial.println("RIGHT");
  else if (currentState == STOPPED) Serial.println("STOPPED");
  else if (currentState == INICIAL) Serial.println("INICIAL");
  else if (currentState == ESQUINA) Serial.println("ESQUINA");
  else if (currentState == CORREGIR_ANGULO) Serial.println("CORREGIR_ANGULO");

  // Lógica optimizada usando variable Switch y switch-case
switch (currentState){

  case INICIAL:
  readUltrasonicSensors();
  centrado();
  avanza(200);

  // Lógica optimizada usando variable Switch y switch-case
  if ((distanceRight > 100) && (distanceFront < 25)) {
    currentState = RIGHT;
  } else if ((distanceLeft > 100) && (distanceFront < 25)) {
    currentState = LEFT;
  }
  break;

  case RIGHT:
  paredder();
  currentState = STRAIGHT;
  break;

  case LEFT:
  paredizq();
  currentState = STRAIGHT;
  break;

  case STRAIGHT:
  readUltrasonicSensors();
  centrado();
  avanza(200);

  // Lógica optimizada usando variable Switch y switch-case
  if ((distanceRight > 100) && (distanceFront < 25)) {
    currentState = RIGHT;
  } else if ((distanceLeft > 100) && (distanceFront < 25)) {
    currentState = LEFT;
  }
  break;

 // ...existing code...
case CORREGIR_ANGULO: {
    int gx, gy, gz;
    sensor.getRotation(&gx, &gy, &gz);
    int angulo = gz; // Ajusta si usas otro eje

    if (angulo < 40) {
        girarder();
        delay(30); // Giro corto y suave
        // Si entra al rango, centra y cambia de estado
        sensor.getRotation(&gx, &gy, &gz);
        angulo = gz;
        if (angulo >= 40 && angulo <= 55) {
            currentState = STRAIGHT; // O el estado que corresponda
        }
    } else if (angulo > 55) {
        girarizq();
        delay(30); // Giro corto y suave
        // Si entra al rango, centra y cambia de estado
        sensor.getRotation(&gx, &gy, &gz);
        angulo = gz;
        if (angulo >= 40 && angulo <= 55) {
            currentState = STRAIGHT; // O el estado que corresponda
        }
      }
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
  delay(2100);
updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Gira a la derecha si no hay pared a la derecha
void paredder() {
  Serial.print("IF Right COMFIRMADO");
  retrocede(150);
  delay(3000);
  avanza(200);
  girarder();
  delay(2100);
  updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Corrección derecha
void straightDer() {
      Serial.print("CORRECCIÓN DELTA D COMFIRMADO");
      girarder();
      delay(200);
      avanza(200);
      delay(400);
      girarizq();
      delay(200);
      centrado();
}

// Corrección derecha
void straightIzq() {
      Serial.print("CORRECCIÓN DELTA I COMFIRMADO");
      girarizq();
      delay(200);
      avanza(200);
      delay(400);
      girarder();
      delay(200);
      centrado();
}

// Giroscopio
void giroscopio() {

   int gx, gy, gz;
  sensor.getRotation(&gx, &gy, &gz);

  Serial.print("Velocidad angular [x y z]:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
}