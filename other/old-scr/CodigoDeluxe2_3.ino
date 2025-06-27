#include <Arduino.h>   // Libreria para codigo tipo c++ para placa arduino
#include <AFMotor.h>   // Libreria Adafruit Motor Shield V1 o en su defecto la V2 para controlar el shield
#include <Servo.h>     // Libreria Servo normalmente implementada de base para contrlar servomotores
#include <Adafruit_VL53L0X.h>   // Libreria VL53L0X usada para los sensores TOF
#include <VL53L0X.h>
#include <QuadratureEncoder.h>   // Libreria para el encoder
#include <move.h>   // Libreria para el encoder

// Configuración de pines y parámetros ---------------------------------------------

// Configuración de la posición de los sensores TOF
#define LOX1_ADDRESS 0x30   // TOF FRONTAL
#define LOX2_ADDRESS 0x31   // TOF IZQUIERDO
#define LOX3_ADDRESS 0x32   // TOF DERECHO
#define LOX4_ADDRESS 0x33   // TOF TRASERO

// objetos del vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();   // TOF FRONTAL
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();   // TOF IZQUIERDO  
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();   // TOF DERECHO
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();   // TOF TRASERO

// Pines para apagar el TOF
#define SHT_LOX1 24
#define SHT_LOX2 23
#define SHT_LOX3 25
#define SHT_LOX4 26


#define MAX_DISTANCE 3354 // Distancia máxima a medir (en mm)

// --- ENCODER DE CUADRATURA ---
// Pines del encoder proporcionados por el usuario (A8 y A9)
#define ENCODER_PIN_A  A8   // Pin para el encoder A
#define ENCODER_PIN_B  A9   // Pin para el encoder B

#define SERVO_PIN     10  // Pin para el servo

// Objeto Encoder usando la librería QuadratureEncoder.h
Encoders encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Parámetros del encoder y la rueda
volatile long encoderTicks = 0;   // Contador de pulsos del encoder (lectura directa de encoder.getEncoderCount())
float wheelDiameter = 0.068;       // Diámetro de la rueda en metros (ej. 7 cm = 0.07 m). ¡AJUSTA ESTO!
float wheelCircumference = wheelDiameter * PI; // Circunferencia de la rueda en metros
int ticksPerRevolution = 700;    // Pulsos del encoder por revolución de la rueda (¡MUY IMPORTANTE: CONSULTA EL DATASHEET DE TU ENCODER!)
long prevEncoderTicks = 0; // Ticks previos del encoder
float totalDistanceTravelledCm = 0.0; // Distancia total recorrida en cm


AF_DCMotor motor(3);      // Motor conectado al puerto M3
Servo myServo;            // Objeto servo

unsigned int distanceRight, distanceLeft, distanceFront, distanceRear;   //Distancia en cm

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
int objetoDelante = 900; //Si el sensor frontal detecta a menos de esto, es hora de decidir un giro
int paredCerca = 250; // Distancia para considerar que una pared lateral está "cerca"
int paredLejos = 400; //Distancia para considerar que una pared lateral está "lejos" o el pasillo "abierto"
int carritoCentrado = 500; //Distancia deseada a las paredes laterales para centrarse
int toleranciaPared = 0;  //Tolerancia para la corrección lateral
int pasilloAbierto = 1000; // Variable para decidir si un pasillo esta abierto o no.
int pasilloCerrado = 800; // Variable para decidir si un pasillo esta cerrado o no.

    // Estado inicial del carro

// --- PARÁMETROS para el TOF ---
int TOF;

// Variable para el control de vueltas
int vueltas = 0;

int servoIzq = 125;   // Determinar el estado basado en la posición del servomotor
int eservoIzq = 100;   // Determinar el estado basado en la posición del servomotor 
int servoCen = 90;    // Determinar el estado basado en la posición del servomotor
int eservoDer = 75;    // Determinar el estado basado en la posición del servomotor
int servoDer = 55;    // Determinar el estado basado en la posición del servomotor

int sentido = 0;   // Determinar la variable de la posición
int Switch = 0;   // Determinar variable del case

// Declaración de variables para las mediciones TOF ------------------------------
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;


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

    // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);


  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();

  

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
read_TOF();
updateEncoderDistance();

// Después de leer sensores o donde prefieras...
Serial.print("Encoder ticks: ");
Serial.println(encoder.getEncoderCount());



  // Lógica optimizada usando variable Switch y switch-case
switch (currentState){

  case INICIAL:
    centrado();
    sentido == 0;
    eAvanza(100);

    //aqui quiero que el carrito comience a sensar las paredes y calcule el tamaño total del pasillo para luego ir ajustandose

    // Distancia izquierda + distancia derecha + tamaño de carrito = tamaño del pasillo. la mitad del pasillo estariamos centrados


    //detenido();
    Serial.println("Setup completo. Iniciando navegación...");
    currentState = STRAIGHT; // Pasa al estado de navegación
    break;

  case STRAIGHT:

    read_TOF();

    // Prioridad 1: Detección de pared frontal -> pasar a punto de decisión
     if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE && sentido == 0 && 500 < distanceFront) {
      Serial.println("Pared frontal detectada. Pasando a DECISION_POINT.");
      currentState = DECIDIR_SENTIDO;
      break; // Salir del switch para procesar el nuevo estado
    } else {
    detenido();
    }

 if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE && sentido != 0) {
      Serial.println("Pared frontal detectada. Pasando a DECISION_POINT.");
      //detenido(); // Detener para la decisión
      currentState = SENTIDO;
      break; // Salir del switch para procesar el nuevo estado
    }
    // Prioridad 2: Corrección lateral para mantenerse centrado
     if (distanceRight != MAX_DISTANCE && distanceLeft != MAX_DISTANCE) {
       if (distanceLeft < paredCerca - toleranciaPared) {
         // Demasiado cerca de la izquierda, girar un poco a la derecha
         straightDer();
         Serial.println("Corrección: Demasiado cerca izquierda -> Derecha.");
         } else if (distanceRight < paredCerca - toleranciaPared && distanceRight < distanceLeft) {
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

      read_TOF();

    // Si la derecha está libre (distancia > pasilloAbierto) Y la izquierda está bloqueada
    if (distanceLeft < pasilloCerrado)   {
      Serial.println("Decisión: Derecha abierta, Izquierda bloqueada -> Gira a la derecha.");
      sentido = 1;
       currentState = GIRORIGHT;
       break;
    }
    // Si la izquierda está libre (distancia > pasilloAbierto) Y la derecha está bloqueada
    if (distanceRight < pasilloCerrado) {
    Serial.println("Decisión: Izquierda abierta, Derecha bloqueada -> Gira a la izquierda.");
    sentido = 2;
    currentState = GIROLEFT;
    break;
   }
    break;

//Ejecuta la secuencia de giro a la izquierda
  case GIROLEFT:    
    paredizq();
    //currentState = STRAIGHT; // Vuelve a navegar recto después del giro
    currentState = STRAIGHT;
    break;

// Ejecuta la secuencia de giro a la derecha
   case GIRORIGHT:
    paredder();
    currentState = STRAIGHT; 
    break;

   case SENTIDO:
   if (sentido == 1) {
      currentState = GIRORIGHT;
   }
    else if (sentido == 2) {
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

void eAvanza(int cm) {
  // Calcula la cantidad de ticks necesarios para avanzar los cm pedidos
  float distanciaEnCm = (float)cm;
  float vueltasNecesarias = distanciaEnCm / (wheelCircumference * 100); // wheelCircumference está en metros
  long ticksObjetivo = (long)(vueltasNecesarias * ticksPerRevolution);

  long ticksIniciales = encoder.getEncoderCount();

  avanza(200);
  while ((encoder.getEncoderCount() - ticksIniciales) < ticksObjetivo) {
    // Puedes llamar a updateEncoderDistance() aquí si quieres actualizar la distancia total
    updateEncoderDistance();
    read_TOF();
    // Pequeña pausa para no saturar el loop
    delay(5);
  }
  detenido();
}

// ...existing code...

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
  sentido == 1;
}

// Función para girar a la izquierda
void girarizq() {   // Definimos nombre del void
  myServo.write(servoIzq);   // Gira a la izquierda
  sentido == 2;
}

// Función para ir derecho
void centrado() {   // Definimos nombre del void
  myServo.write(servoCen);   // Derecho

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
  eAvanza(100);
}

// Gira a la derecha si no hay pared a la derecha
void paredder() {
  Serial.print("IF Right COMFIRMADO");
  girarder();
  eAvanza(100);
  //updateEncoderDistance();   // Actualizar la distancia recorrida
}

// Corrección derecha
void straightIzq() {
    Serial.print("CORRECCIÓN DELTA DERECHA");
    myServo.write(eservoIzq);   // Gira a la izquierda
    eAvanza(30);
    centrado();
    
}

// Corrección derecha
void straightDer() {
    Serial.print("CORRECCIÓN DELTA IZQUIERDA");
    myServo.write(eservoDer);   // Gira a la derecha
    eAvanza(30);
    centrado();
    

}

void setID() {
  // Apaga todos los sensores
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);

  // Inicializa LOX1
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("El primer sensor Tof fallo VL53L0X"));
    while(1);
  }

  // Inicializa LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("El segundo sensor Tof fallo VL53L0X"));
    //while(1);
  }

  // Inicializa LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("El tercer sensor Tof fallo VL53L0X"));
    //while(1);
  }

  // Inicializa LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("El cuarto sensor Tof fallo VL53L0X"));
    //while(1);
  }
}

void read_TOF() {

  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);

  // Sensor 1
  if(measure1.RangeStatus != 4) {
    distanceFront = measure1.RangeMilliMeter;
  } else {
    distanceFront = MAX_DISTANCE;
  }

  // Sensor 2
  if(measure2.RangeStatus != 4) {
    distanceLeft = measure2.RangeMilliMeter;
  } else {
    distanceLeft = MAX_DISTANCE;
  }

  // Sensor 3
  if(measure3.RangeStatus != 4) {
    distanceRight = measure3.RangeMilliMeter;
  } else {
    distanceRight = MAX_DISTANCE;
  }

  // Sensor 4
  if(measure4.RangeStatus != 4) {
    distanceRear = measure4.RangeMilliMeter;
  } else {
    distanceRear = MAX_DISTANCE;
  }

  // Imprime todas las distancias en una sola línea
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" | Left: "); Serial.print(distanceLeft);
  Serial.print(" | Right: "); Serial.print(distanceRight);
  Serial.print(" | Rear: "); Serial.println(distanceRear);
}