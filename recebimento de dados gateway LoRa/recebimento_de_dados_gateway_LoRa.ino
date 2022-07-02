
#include <SPI.h>
#include <RH_RF95.h>
#include <Console.h>
#include <Process.h>

RH_RF95 rf95;

String myWriteAPIString = "7FE8C3DTIALJCIWG";
uint16_t crcdata = 0;
uint16_t recCRCData = 0;
float frequency = 915.0;
String dataString = "";
char BytesX[4];
char BytesY[4];
float locX;
float locY;

void uploadData(); // Carregar dados para o ThingSpeak.

void setup() {
   Bridge.begin(115200);
   Console.begin();
   if (!rf95.init()){
    Console.println("Falha na inicialização");
  }
  // Configurar frequência ISM
    rf95.setFrequency(frequency);
  // Potência de instalação, dBm
    rf95.setTxPower(13);
   // rf95.setSyncWord(0x34);
}

uint16_t calcByte(uint16_t crc, uint8_t b){
   uint32_t i;
  crc = crc ^ (uint32_t)b << 8;
  for ( i = 0; i < 8; i++) {
   if ((crc & 0x8000) == 0x8000)
   crc = crc << 1 ^ 0x1021;
   else
   crc = crc << 1;
 }
  return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer, uint32_t length){
 uint16_t wCRC16 = 0;
 uint32_t i;

if (( pBuffer == 0 ) || ( length == 0 )) {
  return 0;
}
for ( i = 0; i < length; i++){
   wCRC16 = calcByte(wCRC16, pBuffer[i]);
 }
return wCRC16;
}

uint16_t recdata( unsigned char* recbuf, int Length){
    crcdata = CRC16(recbuf, Length - 2); //Obter código CRC
    recCRCData = recbuf[Length - 1]; //Calcular dados CRC
    recCRCData = recCRCData << 8; //
    recCRCData |= recbuf[Length - 2];
}


void loop(){
  if (rf95.waitAvailableTimeout(3000)){ //Ouvir dados do nó LoRa
       uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//receber buffer de dados
       uint8_t len = sizeof(buf);//comprimento do buffer de dados
        if (rf95.recv(buf, &len)){ //Verifique se há dados recebidos
            recdata( buf, len);
            Console.print("Obtenha o pacote LoRa: ");
            for (int i = 0; i < len; i++){
                Console.print(buf[i],HEX);
                Console.print(" ");
            }
            Console.println();
            if(crcdata == recCRCData){ //Verifique se o CRC está correto
             if(buf[0] == 1 && buf[1] == 1 && buf[2] ==1){ //Verifique se o ID corresponde ao LoRa Node ID
                   uint8_t data[] = "Servidor ACK";//Resposta (confirmação de recebimento de dados)
                   data[0] = buf[0];
                   data[1] = buf[1];
                   data[2] = buf[2];
                   rf95.send(data, sizeof(data));//Enviar resposta para o nó LoRa
                   rf95.waitPacketSent();
                 int newData[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //Armazenar dados do sensor aqui
                   for (int i = 0; i < 10; i++){
                      newData[i] = buf[i + 3];
                   }
                    uint8_t b1 = newData[0];
                    uint8_t b2 = newData[1];
                    BytesX[0] = newData[2];
                    BytesX[1] = newData[3];
                    BytesX[2] = newData[4];
                    BytesX[3] = newData[5];
                    BytesY[0] = newData[6];
                    BytesY[1] = newData[7];
                    BytesY[2] = newData[8];
                    BytesY[3] = newData[9];
                    uint16_t leiSen = b1 << 8 | b2;
                    locX = floatLocX(BytesX);
                    locY = floatLocY(BytesY);
                    Console.print("leitura do sensor: ");
                    Console.println(leiSen);
                    Console.print("LOCX: ");
                    Console.println(locX);
                    Console.print("LOCY: ");
                    Console.println(locY);
                    dataString ="field1=";
                    dataString += leiSen;
                    dataString +="&field5=";
                    dataString += locX;
                    dataString +="&field6=";
                    dataString += locY;
                    uploadData(); // 
                    dataString="";
    }
  }else{
    Console.println(" CRC Fail");
  }     
}else{
    Console.println("recv failed");          
}
}
}

void uploadData() {//Carregar dados para o ThingSpeak
   // forma a sequência do parâmetro do cabeçalho da API:
  // forme a sequência do parâmetro URL, tenha cuidado com os "
  String upload_url = "https://api.thingspeak.com/update?api_key=";
  upload_url += myWriteAPIString;
  upload_url += "&";
  upload_url += dataString;
  Console.println("Chamar o comando Linux para enviar dados");
  Process p;    //Crie um processo e chame-o de "p", este processo executará um comando curl do Linux
  p.begin("curl");
  p.addParameter("-k");
  p.addParameter(upload_url);
  p.run();    // Execute o processo e aguarde seu término
  Console.print("Comentários do Linux: ");
  //Se houver saída do Linux,
 //envie para o Console:
  while (p.available()>0){
    char c = p.read();
    Console.write(c);
  }
 Console.println("");
 Console.println("Call Finished");
 Console.println("####################################");
 Console.println("");
}

float floatLocX(char teste[4]){
   union{
     float valX;
     unsigned char bvalX[4];
  }floatAsBytes;
  floatAsBytes.bvalX[0] =teste[0];
  floatAsBytes.bvalX[1] = teste[1];
  floatAsBytes.bvalX[2] = teste[2];
  floatAsBytes.bvalX[3] = teste[3];

  return floatAsBytes.valX;
  }

 float floatLocY(char teste[4]){
   union{
     float valY;
     unsigned char bvalY[4];
  }floatAsBytes;
  floatAsBytes.bvalY[0] =teste[0];
  floatAsBytes.bvalY[1] = teste[1];
  floatAsBytes.bvalY[2] = teste[2];
  floatAsBytes.bvalY[3] = teste[3];

  return floatAsBytes.valY;
  }
