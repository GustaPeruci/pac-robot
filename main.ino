#include <SoftwareSerial.h>
#include <Ultrasonic.h>

/*DECLARAÇÃO DE VARIAVEIS*/
#define MotorA_sentido1 3
#define MotorA_sentido2 4
#define MotorB_sentido1 5
#define MotorB_sentido2 6
#define MotorA_PWM 2  
#define MotorB_PWM 7
#define pinoRX 8;
#define pinoTX 9;
#define pino_trigger 10
#define pino_echo 11

int dadoBluetooth = 0;
Ultrasonic ultrasonic(pino_trigger, pino_echo);
SoftwareSerial bluetooth(pinoRX, pinoTX);

#define veloc0 0
#define veloc1 80
#define veloc2 180
#define veloc3 255

#define Sensor_direita 6
#define Sensor_esquerda 7

bool direita, esquerda;

void setup() {
  Serial.begin(9600);
  pinMode(MotorA_sentido1, OUTPUT);
  pinMode(MotorA_sentido2, OUTPUT);
  pinMode(MotorB_sentido1, OUTPUT);
  pinMode(MotorB_sentido2, OUTPUT);
  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);
  pinMode(Sensor_direita, INPUT);
  pinMode(Sensor_esquerda, INPUT);
  delay(100);
}

void loop() {
   //Define o sentido de rotação dos motores
  digitalWrite(MotorA_sentido1, LOW);
  digitalWrite(MotorA_sentido2, HIGH);
  digitalWrite(MotorB_sentido1, HIGH);
  digitalWrite(MotorB_sentido2, LOW);
  
  //Leituras dos Sensores das faixas
  direita = digitalRead(Sensor_direita);
  esquerda = digitalRead(Sensor_esquerda);
  Serial.print(direita);
  Serial.print(" || ");
  Serial.println(esquerda);

  //Leitura dos Sensores de distância
  float cmMsec,
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);

  //Se o Bluetooh estiver disponível o robô pode ser controlado por ele
  if(bluetooth.available()){
    dadoBluetooth = bluetooth.read(); 
     //Se o dado do bluetooh for verdadeiro ele liga o sistema
     if(dadoBluetooth == '1'){ 
       Serial.println("SISTEMA LIGADO");
      
       if(direita == false && esquerda == false){
       analogWrite(MotorA_PWM, veloc2);
       analogWrite(MotorB_PWM, veloc2);
       } else if(direita == false && esquerda == true){
       delay(400);
       analogWrite(MotorA_PWM, veloc2);
       analogWrite(MotorB_PWM, veloc1);
       delay(400);
       }else if(direita == true && esquerda == false){
       delay(400);
       analogWrite(MotorA_PWM, veloc1);
       analogWrite(MotorB_PWM, veloc2);
       delay(400);
       
       }else if(direita == true && esquerda == true){
       analogWrite(MotorA_PWM, veloc0);
       analogWrite(MotorB_PWM, veloc0);
       }

       if(cmMsec = 1.5){
        digitalWrite(MotorA_sentido1, HIGH);
        digitalWrite(MotorA_sentido2, LOW);
        digitalWrite(MotorB_sentido1, LOW);
        digitalWrite(MotorB_sentido2, HIGH);
       }
     }
     //Se o dado do bluetooh não for verdadeiro ele desliga o sistema
     if(dadoBluetooth == '0'){ 
        Serial.println("SISTEMA DESLIGADO"); 
      }
}
 
