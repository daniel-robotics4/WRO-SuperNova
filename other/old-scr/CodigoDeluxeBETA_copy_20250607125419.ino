#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <NewPing.h>   // Libreria NewPing usada para sensores ultrasonicos corrige y mejora aspectos tecnicos de estos
#include <QuadratureEncoder.h>   // Libreria para el encoder

// Configuración de pines y parámetros
#define TRIGGER_PIN1  17   // Pin para el sensor ultrasonico 1 de la Right
#define ECHO_PIN1     16   // Pin para el sensor ultrasonico 1 de la Right
#define TRIGGER_PIN2  19   // Pin para el sensor ultrasonico 2 de la Left
#define ECHO_PIN2     18   // Pin para el sensor ultrasonico 2 de la Left
#define TRIGGER_PIN3  16   // Pin para el sensor ultrasonico 3 Right
#define ECHO_PIN3     17   // Pin para el sensor ultrasonico 3 Right
#define TRIGGER_PIN4  20   // Pin para el sensor ultrasonico 4 Rear
#define ECHO_PIN4     21   // Pin para el sensor ultrasonico 4 Rear
#define MAX_DISTANCE  300  // Distancia máxima a medir (en cm)

#define EncoderA      A8   // Pin para el encoder A
#define EncoderB      A9   // Pin para el encoder B

#define SERVO_PIN     23   // Pin para el servo

Encoders myEncoder(EncoderA, EncoderB);    // Crear un objeto para el encoder

AF_DCMotor motor(3);      // Motor conectado al puerto M3
Servo myServo;            // Objeto servo

// Declarando ultrasonicos
NewPing sonarRight(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar1
NewPing sonarLeft(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar2
NewPing sonarFront(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar3
NewPing sonarRear(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);   // Distancia máxima que le permitimos medir al senar4

unsigned int distanciaRight, distanciaLeft, distanciaFront, distanciaRear;   //Distancia en cm

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
double distance = 0;

// Función para definir los datos
void setup() {   //  función que solo se ejecutará una sola vez al inicio del programa

  Serial.begin(9600);   // Iniciando comunicación serial para depuración
  Serial.println("--- Iniciando Carrito Autonomo WRO Futuros Ingenieros ---");   // Imprimir en el serial

    pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIGGER_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
    pinMode(TRIGGER_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);
    pinMode(TRIGGER_PIN4, OUTPUT);
  pinMode(ECHO_PIN4, INPUT);

  // Adjuntar el servomotor al pin y centrarlo
  myServo.attach(SERVO_PIN);   // Adjuntar servo al pin 29
  myServo.write(servoCen);   // Posición inicial del servo
  delay(100);

  // Configurar el motor de tracción (detenido al inicio)
  motor.setSpeed(0);    // Velocidad del motor (0-255)
  motor.run(RELEASE);   // Libera el moto para que pueda girar libremente si no está activo

  // Inicializar el contador del encoder a cero
  // La creación del objeto Encoders ya maneja la configuración inicial
  myEncoder.setEncoderCount(0);   // Reinicia el contador del encoder a 0
  Serial.println("Sistema Configurado");   // Mensaje
  
}

// --- BUCLE PRINCIPAL DEL PROGRAMA ---
void loop() {   //  función que se ejecutará en bucle durante todo el programa

 // Leer las distancias de los sensores ultrasónicos
 // unsigned int distancia1 = sonar1.ping_cm();   // Distancia medida por el sensor 1 (derecha)
 // unsigned int distancia2 = sonar2.ping_cm();   // Distancia medida por el sensor 2 (izquierda)
 // unsigned int distancia3 = sonar3.ping_cm();   // Distancia medida por el sensor 3 (frontal)
 // unsigned int distancia4 = sonar4.ping_cm();   // Distancia medida por el sensor 4 (trasero)

distance = medirDistancia();

 avanza(200);
 delay(2000);
 girarizq("Izquierda");
 delay(1700);
 centrado("Centrarse");
 delay(100);
 avanza(200);
 delay(1000);
 girarizq("Izquierda");
 delay(1700);
 centrado("Centrarse");
 delay(100);
 avanza(200);
 delay(1000);
 girarizq("Izquierda");
 delay(1700);
 centrado("Centrarse");
 delay(100);
 detenido();
 delay(10000);

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
void girarder(String sentido) {   // Definimos nombre del void
  myServo.write(servoDer);   // Gira a la derecha
}

// Función para girar a la izquierda
void girarizq(String sentido) {   // Definimos nombre del void
  myServo.write(servoIzq);   // Gira a la izquierda
}

// Función para ir derecho
void centrado(String sentido) {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho

  // Función para ultrasonico derecho
void centrado(String sentido) {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho

   // Función para ultrasonico derecho
void ultrader(String sentido) {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho
}