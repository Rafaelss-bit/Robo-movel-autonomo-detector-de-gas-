


#include <SPI.h>
#include "Wire.h"
#include <RH_RF95.h>

RH_RF95 rf95; // driver utilizado para envio e recebimento de dados LoRa 
#define myAdress 0x08


byte bGlobalErr;//variavel para controle de erros no envio e recebimento de dados
char sensLoc_dat[2]; //Vetor para Armazenar dados do sensor de gás
char BytesX[4];
char BytesY[4];
char node_id[3] = {1,1,1}; //ID do nó final LoRa usado para sabermos identificar que nó está enviando dados
float frequency = 915.0; //frequencia de operação do LoRa, no Brasil usamos padrão australiano (915 Mhz)
unsigned int count = 1;
//char locSerial[4];
char vetorDados [2];
int sentido = 0;
uint16_t leitura;
//Variaveis Odometria
float angulo;    //Armazena angulo em radiandos do giro do robô
float contadorX; //Contador usado para calculo de distância X
float contadorY; //Contador usado para calculo de distância Y
int contadorr;   //Contador usado para calculo da angulação em radianos
int status_sentido; //Marca o sentido do robô

void setup() {
  Wire.begin(myAdress);
  //Registra um evento para ser chamado quando chegar algum dado via I2C
  Wire.onReceive(receiveEvent);    
  pinMode(A0, INPUT);
  pinMode(3, INPUT); // Primeiro pino de interrupção
  attachInterrupt(1, ContarAnguloPos, RISING); // 
  Serial.begin(9600);
      if (!rf95.init()){
       Serial.println("falha na inicialização do driver");
    }
    //Configurando a frequencia de operação
    rf95.setFrequency(frequency);

    // Potência de configuração, dBm
    rf95.setTxPower(13);
    rf95.setSpreadingFactor(7);
    Serial.println("Nó Final LoRa"); 
    Serial.println("Sensor de gás MQ-6");
    Serial.print("ID do nó final LoRa:: ");
    for(int i = 0;i < 3; i++){
        Serial.print(node_id[i],HEX);
    }
    Serial.println();
  angulo=0;      //Angulo inicial
  contadorX=50;  //Começa em uma posição X determinada no mapa
  contadorY=50;  //Começa em uma posição Y determinada no mapa
}

void ReadSensorMQ(){
bGlobalErr=0;
leitura = analogRead(A0);
byte i;
 // for(i =0; i<2; i++){
    sensLoc_dat[0] = (uint8_t)((leitura>>8)&0xFF); //byte mais significativo do sinal
    sensLoc_dat[1] = (uint8_t)leitura; //byte menos significativo do sinal
 // }
}

uint16_t calcByte(uint16_t crc, uint8_t b){
  uint32_t i;
  crc = crc ^ (uint32_t)b << 8;
    for ( i = 0; i < 8; i++){
       if ((crc & 0x8000) == 0x8000){
            crc = crc << 1 ^ 0x1021;
       }else{
            crc = crc << 1;
       }
    }
 return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer,uint32_t length){
   uint16_t wCRC16=0;
   uint32_t i;
    if (( pBuffer==0 )||( length==0 )){
      return 0;
    }
    for ( i = 0; i < length; i++){ 
      wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}
void loop() {
 //  if(Serial.available()>0){
 //   sentido = Serial.read();
    // for(int i =0; i<2; i++){
     //  vetorDados[i]= Serial.read();
     //  vetorDados[i+1]=Serial.peek();
    // }
//   }

    Serial.print("###########    ");
    Serial.print("COUNT=");
    Serial.print(count);
    Serial.println("    ###########");
    count++;

    ReadSensorMQ();
    char data[50] = {0} ;
    int dataLength = 13; // tamanho do pacote

    // Use data[0], data[1],data[2] como id do nó
    data[0] = node_id[0] ;
    data[1] = node_id[1] ;
    data[2] = node_id[2] ;
    data[3] = sensLoc_dat[0]; //byte mais significativo da leitura do sensor
    data[4] = sensLoc_dat[1]; //byte menos significaivo da leitura do sensor
    //Bytes posição X 
    data[5] = BytesX[0];
    data[6] = BytesX[1];
    data[7] = BytesX[2];
    data[8] = BytesX[3];
    //Bytes posição Y
    data[9] = BytesY[0];
    data[10] = BytesY[1];
    data[11] = BytesY[2];
    data[12] = BytesY[3];

    uint16_t crcData = CRC16((unsigned char*)data,dataLength);//Obter CRC
      Serial.print("Dados a serem enviados(sem CRC): ");
    int i;
    for(i = 0;i < dataLength; i++){
        Serial.print(data[i],HEX);
        Serial.print(" ");
    }
    Serial.println();

    unsigned char sendBuf[50]={0};
    for(i = 0;i < dataLength;i++){
        sendBuf[i] = data[i] ;
    }
    // Adicionar CRC aos dados LoRa
    sendBuf[dataLength] = (unsigned char)crcData; 
    sendBuf[dataLength+1] = (unsigned char)(crcData>>8);

    Serial.print("Dados a serem enviados com CRC    ");
    for(i = 0;i < (dataLength +2); i++){
        Serial.print(sendBuf[i],HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    rf95.send(sendBuf, dataLength+2);//envio de dos dados LoRa
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//Matriz de dados de resposta
    uint8_t len = sizeof(buf);//comprimento dos dados de resposta

    if (rf95.waitAvailableTimeout(3000)){ //Verifique se há resposta em 3 segundos
    //Deve ser uma mensagem de resposta para nós agora 
      if (rf95.recv(buf, &len)){ //verifique se a mensagem de resposta está correta
          if(buf[0] == node_id[0] && buf[1] == node_id[2] && buf[2] == node_id[2] ){ // Verifique se a mensagem de resposte tem o ID do nó
              pinMode(4, OUTPUT);
              digitalWrite(4, HIGH);
              Serial.print("Resposta do Gateway: ");//imprime a resposta
              Serial.println((char*)buf);
              delay(400);
              digitalWrite(4, LOW); 
          }    
      }else{
         Serial.println("Falha na Recepção");//
         rf95.send(sendBuf, strlen((char*)sendBuf));//reenviar se não houver resposta
      }
    }else{
      Serial.println("Sem resposta, o gateway LoRa está em execução ?");//Sem resposta do sinal
      rf95.send(sendBuf, strlen((char*)sendBuf));//reenviar dados
   }
   delay(3000); // Envie dados do sensor a cada 3 segundos
   Serial.println("");
      
}

void receiveEvent(int leitura) 
{  
  sentido = Wire.read();   
}


 void ContarAnguloPos(){ // Função para valor de Angulo e Posição 
  static unsigned long delayest; // delay falso para retornar um estado sem "tremida"
  switch(sentido){
    case 1: //direita
     if(millis()-delayest>1){
      contadorr= contadorr+1;
      angulo=contadorr*3.1415/200;
      if(angulo>=6.283){
        contadorr=0;
      }
     }
      delayest=millis();
      break; 
    case 2 : //esquerda
     if(millis()-delayest>1){
      contadorr= contadorr-1;
      angulo=contadorr*3.1415/200;
      if(angulo>=6.283){
        contadorr=0;
      }
     }
     delayest=millis();
    break;
    case 3: //re
      if(millis()-delayest>1){
        contadorX= contadorX-1*cos(angulo); 
        contadorY= contadorY-1*sin(angulo);
      }
      delayest=millis();
    break;
    case 4: //frente
       if(millis()-delayest>1){
         contadorX= contadorX+1*cos(angulo); 
         contadorY= contadorY+1*sin(angulo);
       }
       delayest=millis();
    break;
  }
  bytesLocX(contadorX);
  bytesLocY(contadorY);
}

void bytesLocX(float locX){
  union{
     float valX;
     unsigned char bvalX[4];
  }floatAsBytes;
  floatAsBytes.valX = locX;
  BytesX[0] = floatAsBytes.bvalX[0];
  BytesX[1] = floatAsBytes.bvalX[1];
  BytesX[2] = floatAsBytes.bvalX[2];
  BytesX[3] = floatAsBytes.bvalX[3];
 }

 void bytesLocY(float locY){
  union{
     float valY;
     unsigned char bvalY[4];
  }floatAsBytes;
  floatAsBytes.valY = locY;
  BytesY[0] = floatAsBytes.bvalY[0];
  BytesY[1] = floatAsBytes.bvalY[1];
  BytesY[2] = floatAsBytes.bvalY[2];
  BytesY[3] = floatAsBytes.bvalY[3];
 }



  
