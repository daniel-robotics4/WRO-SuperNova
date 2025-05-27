Control software
====

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

// Crear el objeto del motor DC conectado al puerto M3
AF_DCMotor myMotor(3, MOTOR34_1KHZ); // Motor en M3 con frecuencia de 1kHz

// Crear el objeto del servo
Servo servoMotor;

// Pines de los sensores ultrasónicos
#define TRIG_FRONT 14
#define ECHO_FRONT 15
#define TRIG_RIGHT 16
#define ECHO_RIGHT 17
#define TRIG_LEFT 18
#define ECHO_LEFT 19
#define TRIG_BACK 20
#define ECHO_BACK 21

// Configuración de los sensores ultrasónicos
#define MAX_DISTANCE 200 // Máxima distancia en cm
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarBack(TRIG_BACK, ECHO_BACK, MAX_DISTANCE);

// Pines del encoder
#define ENCODER_A 22
#define ENCODER_B 23

// Variables globales
volatile int encoderRevolutions = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 50; // Intervalo para verificar sensores

// Función para manejar interrupciones del encoder
void encoderISR() {
  static bool lastA = LOW;
  bool currentA = digitalRead(ENCODER_A);
  if (lastA != currentA) {
    if (digitalRead(ENCODER_B) != currentA) {
      encoderRevolutions++;
    } else {
      encoderRevolutions--;
    }
  }
  lastA = currentA;
}

// Función para leer la distancia de un sensor ultrasónico
int readDistance(NewPing &sonar) {
  return sonar.ping_cm();
}

// Función para reiniciar el encoder
void resetEncoder() {
  noInterrupts();
  encoderRevolutions = 0;
  interrupts();
}

// Configuración inicial
void setup() {
  // Inicializar el motor
  myMotor.setSpeed(250); // Velocidad inicial del motor (0-255)
  myMotor.run(RELEASE);  // Detener el motor inicialmente

  // Configurar el servo
  servoMotor.attach(9); // Pin digital 9
  servoMotor.write(90); // Posición inicial

  // Configurar pines del encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Configurar comunicación serial
  Serial.begin(9600);
}

// Bucle principal
void loop() {
  unsigned long currentMillis = millis();

  // Verificar sensores periódicamente
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int distanceRight = readDistance(sonarRight);
    int distanceLeft = readDistance(sonarLeft);
    int distanceFront = readDistance(sonarFront);
    int distanceBack = readDistance(sonarBack);

    // Paso 2: Girar el servo según las distancias
    if (distanceRight > 100 || distanceRight == 0) {
      servoMotor.write(135);
      resetEncoder();
    } else if (distanceLeft > 100 || distanceLeft == 0) {
      servoMotor.write(45);
      resetEncoder();
    }

    // Paso 4: Mover el motor según las condiciones
    if ((distanceRight <= 20 && distanceRight > 0) || (distanceLeft <= 40 && distanceLeft > 0) ||
        (distanceFront > 100) || (distanceBack <= 40 && distanceBack > 0) || encoderRevolutions >= 15) {
      myMotor.run(RELEASE);
    } else {
      myMotor.run(FORWARD);
    }

    // Paso 5: Regresar el servo a 90°
    servoMotor.write(90);

    // Paso 6: Repetir el bucle hasta 12 veces
    static int loopCount = 0;
    if (loopCount >= 12) {
      resetEncoder();
      while (encoderRevolutions < 25) {
        myMotor.run(FORWARD);
      }
      myMotor.run(RELEASE);
      while (true); // Detener el programa
    }
    loopCount++;
  }
}
