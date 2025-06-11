#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

// Configuración de pines y parámetros
#define TRIGGER_PIN  12   // Pin trigger del sensor ultrasónico
#define ECHO_PIN     13   // Pin echo del sensor ultrasónico
#define MAX_DISTANCE 300  // Distancia máxima a medir (en cm)

AF_DCMotor motor(3);      // Motor conectado al puerto M3
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  motor.setSpeed(200);    // Velocidad del motor (0-255)
}

void loop() {
  delay(50); // Pequeño retardo para estabilidad del sensor
  unsigned int distancia = sonar.ping_cm();

  if (distancia > 200 && distancia < MAX_DISTANCE) {
    motor.run(RELEASE);   // Detener motor si detecta más de 200 cm
  } else {
    motor.run(FORWARD);   // Mantener motor encendido
  }
}