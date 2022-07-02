/// Referencias: http://tixplicando.blogspot.com/2015/10/criando-um-robo-para-derrubar-obstaculos.html
  //               https://github.com/JacksonDuvan/Machine-Learning-/blob/master/laberinto1.ino
#include "Wire.h"
#define slaveAdress 0x08
// Variaves para controle do motor
int URTRIG = 5; 
int Frente = 0;  
int Tras  = 1;  
int MotorEsq = 1;  
int MotorDir = 2; 
int URPWM = 3; 
uint8_t EnPwmCmd[4] = {0x44, 0x22, 0xbb, 0x01};
int STBY = 10; 
 //Motor A  
int PWMA = 6; 
int AIN1 = 9; 
int AIN2 = 8;
 //Motor B  
int PWMB = 11;
int BIN1 = 12;
int BIN2 = 2; 
//vetor de envio da localização
char locVet [2];
char loc[2];
unsigned long previusMillis = 0;
// Variaves para controle do sensor ultrassônico
const int EchoPIN1 = 4; 
const int TrigPIN1 = 7; 
#define EchoPIN2  A2
#define TrigPIN2 A3 
#define EchoPIN3  A0
#define TrigPIN3  A1
long distancia_cm1 = 0;
long distancia_cm2 = 0;
long distancia_cm3 = 0;
long Duration1; 
long Duration2; 
long Duration3; 
int status_sentido;
float RightDistance = 0;
float LeftDistance = 0;
float MiddletDistance = 0;
int MaxX; //Armazena a distância X maxima no mapa em centimetros
int MaxY; //Armazena a distância Y maxima no mapa em centimetros

//Ditancia em cm do sensor direito
int Distance_Right (void) {                   
  digitalWrite (TrigPIN2, LOW);       
  delayMicroseconds (2);              
  digitalWrite (TrigPIN2, HIGH);      
  delayMicroseconds (20);             
  digitalWrite (TrigPIN2, LOW);       
  Duration2 = pulseIn (EchoPIN2, HIGH); 
  distancia_cm2 = (Duration2/29/2);  
  return distancia_cm2;  
} 
  
//Ditancia em cm do sensor esquerdo
int Distance_Left (void) {                    
  digitalWrite (TrigPIN3, LOW);       
  delayMicroseconds (2);              
  digitalWrite (TrigPIN3, HIGH);      
  delayMicroseconds (20);             
  digitalWrite (TrigPIN3, LOW);       
  Duration3 = pulseIn (EchoPIN3, HIGH); 
  distancia_cm3 = (Duration3/29/2);  
  return distancia_cm3;  
}   

//Ditancia em cm do sensor do meio
int Distance_Middle (void) {                      
  digitalWrite (TrigPIN1, LOW);       
  delayMicroseconds (2);              
  digitalWrite (TrigPIN1, HIGH);      
  delayMicroseconds (20);             
  digitalWrite (TrigPIN1, LOW);       
  Duration1 = pulseIn (EchoPIN1, HIGH); 
  distancia_cm1 = (Duration1/29/2);  
  return distancia_cm1;  
}   

void setup() {
   pinMode(EchoPIN1, INPUT); 
   pinMode(TrigPIN1, OUTPUT); 
   pinMode(EchoPIN2, INPUT); 
   pinMode(TrigPIN2, OUTPUT); 
   pinMode(EchoPIN3, INPUT); 
   pinMode(TrigPIN3, OUTPUT); 
   Serial.begin(9600);
   Wire.begin();
   pinMode(STBY, OUTPUT);  
   pinMode(PWMA, OUTPUT);  
   pinMode(AIN1, OUTPUT);  
   pinMode(AIN2, OUTPUT);  
   pinMode(PWMB, OUTPUT);  
   pinMode(BIN1, OUTPUT);  
   pinMode(BIN2, OUTPUT);  
   pinMode(A5 ,OUTPUT);  

 

}

void loop() {
  LeftDistance = Distance_Left();
  delay(10);
  RightDistance = Distance_Right();
  delay(10);
  MiddletDistance = Distance_Middle();
  delay(10);
 // Serial.println("Left_Distance\tMiddle_Distance\tRight_Distance\tStatus\n");
//  Serial.print(LeftDistance);
//  Serial.print("cm\t\t");
//  Serial.print(MiddletDistance);
//  Serial.print("cm\t\t");
//  Serial.print(RightDistance);
//  Serial.print("cm\t\t");
//  ContarAnguloPos();
 // Serial.print(contadorX);
 // Serial.print(",");
//  Serial.println(contadorY);

  if(MiddletDistance<=10){
  //  ChangePath ();
  //  delay(3000);
    if(RightDistance > LeftDistance ){
         TurnRight();
     }else if(RightDistance < LeftDistance){
         TurnLeft();
     }else{
         TurnAround();
     }
   }else if(RightDistance<=5){
       TurnLeft();
   }else if(LeftDistance<=5){
       TurnRight();
   }else{
        Forward();
   }
  }
 
void ChangePath () {         
  Stop ();                  
  Backward ();               
  Stop ();                     
}

void Forward () {   
 status_sentido = 4;
 enviarDadosSerial(status_sentido);
 move(MotorEsq, 145, Frente); 
 move(MotorDir, 150, Frente);  
}

void Backward () { 
 status_sentido = 3;   
 enviarDadosSerial(status_sentido);
move(MotorEsq, 145, Tras);   
move(MotorDir, 150, Tras); 
delay (1000);      
}

void TurnRight () {   
 status_sentido = 1; 
 enviarDadosSerial(status_sentido);       
move(MotorEsq, 150, Frente);  
move(MotorDir, 145, Tras); 
delay (300);                
}

void TurnLeft () {  
 status_sentido = 2;     
 enviarDadosSerial(status_sentido);     
 move(MotorEsq, 145, Tras); 
 move(MotorDir, 150, Frente);    
 delay (300);               
}

void TurnAround () {  
 move(MotorEsq, 50, Tras); 
 move(MotorDir, 55, Frente);        
 delay (500);              
}
void Stop () {      
  stop();
  delay (100);      
}

void move(int motor, int speed, int direction) {  
  digitalWrite(STBY, HIGH);
  boolean inPin1 = LOW;  
  boolean inPin2 = HIGH;  
  if (direction == 1) {  
   inPin1 = HIGH;  
   inPin2 = LOW;  
  }  
  if (motor == 1) {  
   digitalWrite(AIN1, inPin1);  
   digitalWrite(AIN2, inPin2);  
   analogWrite(PWMA, speed);  
  } else if (motor == 2) {  
   digitalWrite(BIN1, inPin1);  
   digitalWrite(BIN2, inPin2);  
   analogWrite(PWMB, speed);  
  }  
 }  
      
 void stop() {    
  digitalWrite(STBY, LOW);  
 }  
 
 void PWM_Mode_Setup() {  
  pinMode(URTRIG, OUTPUT);             
  digitalWrite(URTRIG, HIGH);            
  pinMode(URPWM, INPUT);                
  for (int i = 0; i < 4; i++) {  
   Serial.write(EnPwmCmd[i]);  
  }  
 }

 void enviarDadosSerial(int sentido){
  if(millis() - previusMillis > 500){
    // incia a transmissao para o endereco 0x08 (slaveAdress)
    Wire.beginTransmission(slaveAdress);
    Wire.write(sentido); // envia um byte contendo o estado do LED
    Wire.endTransmission(); // encerra a transmissao
   }
 }
 //void enviarDadosSerial(uint16_t x, uint16_t y){
   //locVet[0] = (uint8_t)x;
   //locVet[1] = (uint8_t)y;
  //if(millis() - previusMillis > 3000){
    //Serial.write(locVet,2);
    //previusMillis = millis();
  //}
  //}
 
