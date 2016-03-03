/***************************************************************
/                                                               *
/                      Codigo implementado no barco             *
/                                                               *
/   Instituto tecnologico de aeronautica - ITA                  *
/   Criado por: David Issa Mattos                               *
/   Orientadores: Cairo Lucio Nascimento Junior                 *
/                 Douglas Soares dos Santos                     *
/                                                               *
/    Data de criacao: 03/09/2014                                *
/    Ultima modificacao: 18/08/2015                             * 
/***************************************************************/


/*Log de versoes
  - 03/09/2014 - Criacao do sketch inicial do programa
  - 04/09/2014 - Criacao da biblioteca do compasso da IMU
  - 05/09/2014 - Implementacao da biblioteca TinyGPS, SoftwareSerial e das funcoes basicas com strings
  - 09/09/2014 - Primeiros testes com o hardware no barco
               - Arduino queimou o pino TX -> nao grava mais =(
               - Cuidado extra com os contatos dos pinos na placa desenvolvida
               - Cuidado com cargas eletrostaticas
  - 10/10/2014 - Geracao das strings que serao transmitidas
  - 11/10/2014 - Melhoria de erros
               - Controle do fluxo de dados com a serial
               - Correcao dos pinos do gps
               - Troca do codigo do gps -> usando um while igual no livro Arduino Cookbook
               - Testes com a placa na versao 3.0
               - Usando um arduino uno Rev3
               - Funcionando os motores e o gps
  - 15/09/2014 - Adicionando uma funcao e biblioteca para converter float para string
               - stdlib.h para avr
               - Bibliotecas float para o gps
  - 29/09/2014 - Adicionando um contador para verificar a recepcao de sinal. Se nao houver sinal de comunicacao o barco para.
  - 03/11/2014 - Acrescentando configuracoes iniciais para eliminar o static navigation do modulo de GPS 
  - 04/05/2015 - Acrescentando o acelerometro na biblioteca e fazendo sua leitura
  - 05/05/2015 - Acrescentando o gyroscopio na biblioteca e fazendo sua leitura
  - 07/05/2015 - Ajustes na biblioteca. Acrescentado as rotinas para pegar os dados da IMU e enviar
  - 17/06/2015 - Enviando os tempos entre aquisicao de dados da IMU
  - 18/08/2015 - Alterando os sinais do giroscopio e do acelerometro para ficar compativel com o MOOS-IvP
               - IMU envia dados em dps(degree per second). Convertendo para rad/s. Correcoes na biblioteca
               - Acelerometro envia dados em . Convertendo para m/s2. COrrecoes na biblioteca
               - Corrigindo a orientacao dos eixos do acelerometro e do giroscopio. Correcoes no sketch
*/

/*
Tarefas a serem realizadas:
  1- Inicializacao do sistema
    a) Inicializacao do GPS -> string
    b) Inicializacao da IMU
    c) Inicializacao dos canais de comunicacao
    d) Incializacao dos pinos
  2- Ler os dados recebidos pelo radio modem na UART 
  3- Implementar o controle do barco, comandando os motores M1 e M2
  4- Ler os dados do GPS atraves de uma serial por software
  5- Processar os sinais do GPS
  6- Ler os dados da IMU utilizando a biblioteca GY80IMU.h
  7- Enviar string com os dados da IMU e GPS
  8- Verificacao do nivel do sinal do enlace  
*/

/* Inclusao das bibliotecas necessarias
  1 - SoftwareSerial -> para o GPS
  2 - Wire -> implementacao I2C
  3 - TinyGPS -> extracao de dados da string NMEA
  4 - GY80IMU -> biblioteca propria, baseada em outras e no datasheet que cria um objeto para o sensor GY80
  5 - stdlib -> utilizacao e conversao de floats
 */
 
 #include <Wire.h>
 #include <SoftwareSerial.h>
 #include <TinyGPS.h>
 #include <GY80IMU.h>
 #include <stdlib.h>
 
 
 /* Definicao de algumas constantes, enderecos do I2C e pinos */
 //Pinos GPS
   #define GPS_RX 2
   #define GPS_TX 3
   #define GPS_1PPS 4
 
 //Pinos Modem
   #define MODEM_RX 0
   #define MODEM_TX 1
 
 //Pinos Motores
   #define M2_F 8
   #define M2_T 9
   #define M1_F 10
   #define M1_T 11
   
 //constantes
  #define TRUE 1
  #define FALSE 0
  float PI_CTE=3.141592;
  
 /* Inicializando as variaveis e de objetos */
 //Objeto para verificar recepcao de sinal.
 float tempo_recep_ant;
 float tempo_recep_atual;
 int status_recep;//0 -> sem sinal. 1 -> com sinal
 
 
 //Objetos para o GPS
   TinyGPS gps;
   SoftwareSerial gpsSerial(GPS_RX,GPS_TX);
   float lat, lon;
   float speed, course;
   unsigned long fix_age, time, date;
   unsigned long chars;
   unsigned short sentences, failed_checksum;
 
 //Objetos para a IMU
   compassData compassDados;
   accelData accelDados;
   gyroData gyroDados;
   GY80IMU imu;
   float headingDegrees;
   float heading;
   float declinationAngle;
   unsigned long atimeNow, atimeBefore, adeltaTime;
   unsigned long gtimeNow, gtimeBefore, gdeltaTime;
   
 //Objetos para recepcao de dados
   String inputString = "";         // a string to hold incoming data
   String stringM1="";
   String stringM1_header="";
   String stringM2="";
   String stringM2_header="";
   boolean stringComplete = false;  // whether the string is complete
   
 //Objetos para transmissao de dados
   String IMU_HEADING = "";
   String GPS_OUT = "";
   char inter[10];
   
   long time_serial;
   
   
 ////////////////////////   SETUP   ////////////////////////////////////////////
 void setup()
 {
       atimeNow = 0;
       atimeBefore = 0;
       adeltaTime = 0;
       gtimeNow = 0;
       gtimeBefore = 0;
       gdeltaTime = 0;
 /*Inicializando as portas de comunicacao*/
     //Inicializnado o I2C
       Wire.begin();
     //Inicializacao da comunicacao do radio modem
       Serial.begin(9600);
       
 /*Inicializando os pinos*/
   //Configurando os pinos dos motores
       pinMode(M1_F,OUTPUT);
       pinMode(M1_T,OUTPUT);
       pinMode(M2_F,OUTPUT);
       pinMode(M2_T,OUTPUT);
       delay(1000);
   //barco comeca parado
       digitalWrite(M2_T,LOW);
       digitalWrite(M2_F,LOW);
       digitalWrite(M1_T,LOW);
       digitalWrite(M1_F,LOW);

 /*Configurando a IMU*/
   //Configurando a bussola. Se for mudar a escala (gauss) deve-se fazer antes de configurar
      imu.configCOMP(2);
   //Configurando o acelerometro
      imu.configACCEL();//O offset ser definido por software
   //Configurando o giroscopio
     imu.configGYRO(250);//Full-scale de +-250dps
   
 /*Configurando as strings dos dados recebidos*/    
     inputString.reserve(20);
     stringM1_header.reserve(5);
     stringM1.reserve(5);
     stringM2_header.reserve(5);
     stringM2.reserve(5);
     
     /*Inicializando o GPS*/
   //Desabilitando o static navigation
       gpsSerial.begin(4800);
       gpsSerial.println("$PSRF100,0,9600,8,1,0*0C"); //this sets it to binary mode
       delay(3000);
       gpsSerial.begin(9600);
       
       //Sem o static navigation
       gpsSerial.write((byte)0xA0);
       gpsSerial.write((byte)0xA2);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x02);
       gpsSerial.write((byte)0x8F);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x8F);
       gpsSerial.write((byte)0xB0);
       gpsSerial.write((byte)0xB3);
       delay(100);
       
      //Voltando ao normal NMEA
       gpsSerial.write((byte)0xA0);
       gpsSerial.write((byte)0xA2);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x18);
       gpsSerial.write((byte)0x81);
       gpsSerial.write((byte)0x02);
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01);
       delay(100);
       gpsSerial.write((byte)0x00); 
       gpsSerial.write((byte)0x01); 
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01); 
       gpsSerial.write((byte)0x01); 
       gpsSerial.write((byte)0x01); 
       gpsSerial.write((byte)0x00); 
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01);              
       delay(100);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x00);
       gpsSerial.write((byte)0x01);
       
       gpsSerial.write((byte)0x12);//4800bps
       gpsSerial.write((byte)0xC0);
       
       gpsSerial.write((byte)0x01);
       gpsSerial.write((byte)0x61);
       delay(100);
       gpsSerial.write((byte)0xB0);
       gpsSerial.write((byte)0xB3);
       delay(100);
       gpsSerial.begin(4800);
   
   //Frases para desabilitar algumas sentencas do GPS que sobrecarregam o arduino
       delay(100);
       gpsSerial.println("$PSRF103,01,00,00,01*25");//Desabilita o GLL
       delay(100);
       gpsSerial.println("$PSRF103,02,00,00,01*26");//Desabilita o GSA
       delay(100);
       gpsSerial.println("$PSRF103,03,00,00,01*27");//Desabilita o GSV
       delay(100);
       gpsSerial.println("$PSRF103,05,00,00,01*21");//Desabilita o VTG
   
 ///fim do setup
 }



 ////////////////////////   LOOP   ////////////////////////////////////////////

 
 void loop()
 {
 
/*****************    Primeira parte: Leitura dos dados do motor via radio modem  *************/
  //Se ocorreu uma leitura correta dos dados
  //Os eventos sao lidos pela funcao
  //Valores possiveis para M1 ou para M2
      //"D" : motor desligado
      //"F" : motor para frente
      //"T" : motor para tras
  if (stringComplete) 
 {
  
    if(stringM1_header !="M1=" || stringM2_header !="M2=")
      {
        inputString = "";
        stringComplete = false;
        Serial.println("TransmittingError");
      }
     else
     { 
       //salva quando foi recebido a recepcao de sinal
        tempo_recep_atual=millis();
        if(stringM1=="F")
          { 

            digitalWrite(M1_F,HIGH);
            digitalWrite(M1_T,LOW);
          }
        if(stringM1=="T")
          { 
            digitalWrite(M1_T,HIGH);
            digitalWrite(M1_F,LOW);  
          }
         if(stringM1=="D")
          { 
            digitalWrite(M1_T,LOW);
            digitalWrite(M1_F,LOW);  
          }
        if(stringM2=="F")
          { 
            digitalWrite(M2_F,HIGH);
            digitalWrite(M2_T,LOW);        
          }
        if(stringM2=="T")
          { 
            digitalWrite(M2_T,HIGH);
            digitalWrite(M2_F,LOW);
          }
        if(stringM2=="D")
          { 
            digitalWrite(M2_T,LOW);
            digitalWrite(M2_F,LOW);
          }
         // Serial.println("OkTransmitting");
     }
    // clear the string:
    inputString = "";
    stringComplete = false;
 }


/*****************    Segunda parte: Leitura e processamento do GPS  *************/
   
   while (gpsSerial.available())
    {
      int c = gpsSerial.read();
      //Serial.write(c);//Display NMEA data for debug 
      if (gps.encode(c))
      {
        //Serial.println("OK");
        //Debug
           //unsigned long chars;
          //unsigned short sentences, failed_checksum;
          //gps.stats(&chars, &sentences, &failed_checksum);
          // retrieves +/- lat/long in 100000ths of a degree
          gps.f_get_position(&lat, &lon, &fix_age);
          // time in hhmmsscc, date in ddmmyy
          gps.get_datetime(&date, &time, &fix_age);
          // returns speed in meters per second
          speed = gps.f_speed_mps();
          // course in 100ths of a degree
          course = gps.f_course();
      }
   }
         GPS_OUT = "";
       //GPS String 
         GPS_OUT += "GPS_LAT=";
         dtostrf(lat,10,8,inter);
         GPS_OUT += inter;
         
         GPS_OUT += ",GPS_LON=";
         dtostrf(lon,10,8,inter);
         GPS_OUT += inter;
         
         GPS_OUT +=",GPS_TIME=";
         GPS_OUT += time;
         
         GPS_OUT +=",GPS_SPEED=";
         dtostrf(speed,10,8,inter);
         GPS_OUT += inter;
         
         GPS_OUT += ";";
        Serial.println(GPS_OUT); 
  //"GPS_LAT=valor,GPS_LON=valor,GPS_SPEED=valor,GPS_HEADING=valor,GPS_TIME=valor"
 
 /*****************    Terceira parte: Processamento dos dados da IMU  *************/
 //Na presenca de todos os dados possiveis com integracao o heading devera ser compensado para mudancas de inclinacao em X,Y e Z
 //Bussola - Leitura
      compassDados = imu.readCOMP();
            //Pequeno processamento dos dados
              heading = atan2(compassDados.YAxis, compassDados.XAxis);//devido a orientacao
            //Descontando a declinacao magnetica //Declinacao magnetica no ITA -22o21'
              declinationAngle = -(22*PI/180 + (21/60)*PI/180);
              heading = heading - (declinationAngle);
            // Correcao do angulo de guinada
              if(heading < 0) heading += 2*PI;
              if(heading > 2*PI) heading -= 2*PI;
            //Conversao para graus
              headingDegrees =360 - heading * 180/PI;
 
 //Acelerometro - Leitura 
       atimeBefore = atimeNow;
       accelDados = imu.readACCEL();
       atimeNow = millis();
       adeltaTime = atimeNow - atimeBefore;

 //Gyroscopio - Leitura
       gtimeBefore = gtimeNow;
       gyroDados = imu.readGYRO();  
       gtimeNow = millis();
       gdeltaTime = gtimeNow - gtimeBefore;
 // Impressao dos dados adquiridos             
         //Heading
         IMU_HEADING = "";
         IMU_HEADING += "IMU_HEADING=";
         dtostrf(headingDegrees,6,2,inter); 
         IMU_HEADING += inter;
         IMU_HEADING += ";";
         Serial.println(IMU_HEADING);
         
         //ACCEL - Corrigindo a orientacao das medidas do acelerometro para bater com a orientacao do barco
        Serial.print("IMU_AX=");
        dtostrf((accelDados.XAxis),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_AY=");
        dtostrf((accelDados.YAxis*(-1.0)),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_AZ=");
        dtostrf((accelDados.ZAxis*(-1.0)),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_ATIME=");
        dtostrf(adeltaTime,8,4,inter);
        Serial.print(inter);
        Serial.println(";");
        
        //GYRO -> Corrigindo a orientacao do eixo Z apenas do gyroscopio
        // para bater com a orientacao do barco
        Serial.print("IMU_WX=");
        dtostrf((gyroDados.XAxis),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_WY=");
        dtostrf((gyroDados.YAxis),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_WZ=");
        dtostrf((gyroDados.ZAxis*(-1.0)),8,4,inter);
        Serial.print(inter);
        Serial.print(",");
        Serial.print("IMU_WTIME=");
        dtostrf(gdeltaTime,8,4,inter);
        Serial.print(inter);
        Serial.println(";");
 

 
 /*****************    Quarta parte: Verificacao de enlace  *************/
 
 
  /*****************    Fim do loop  *************/
//End void loop()
 }
 
 
 
////////////////////////   Serial Event - Leitura dos dados do Modem - Lida automaticamente com as interrupcoes   ////////////////////////////////////////////
 
 
 
void serialEvent() 
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') 
    {
      stringComplete = true;
    }  
  }
  //Get string datas
  stringM1=inputString.substring(3,4);
  stringM2=inputString.substring(8,9);
  stringM1_header=inputString.substring(0,3);
  stringM2_header=inputString.substring(5,8);
}
