#include <QMC5883LCompass.h>
#include <Wire.h>
#include <Ticker.h>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
Ticker Timer1;
Ticker Timer2;
Ticker Timer3;
Ticker Timer4;
Ticker Timer5;
Ticker Timer6;
#define pi 3.1416
//*************OTA***************
//const char* ssid    = "ERICK";
//const char* password = "29082908";
const char* ssid    = "HUAWEIE";
const char* password = "12345678";
//*******************************
const int D23 = 23;             // PWM
const int D4 = 4;             // SIGNO CW CCW
const int interruptPinA = 18; //D5 CANAL A
const int interruptPinB = 19;
bool PinA, PinB;
float t, s, ms, SPo=0.0, SPoO, RPM, w;
const int frecuencia = 25000;
const int canal = 0;      //0-15 16canales
const int resolucion = 8; //0-255
//int DutyCicle,DutyCicleInverso;
float DutyCicle,DutyCicleInverso;
//float Kp = 0.2, Ki = 1.0, Kd = 0.00000;
//float Kp = 0.15, Ki = 0.1, Kd = 0.009; //con protoboards este era bueno
float Kp = 0.06, Ki = 0.1, Kd = 0.001;
float u=0.0, e=0.0, up, ui, ud, e_1, ui_1, u_abs;
volatile long EncoderCount = 0;
//**********PID Orientación***********
float KpO = 40, KiO = 0.0, KdO = 0.9; //ki= 1.5 se malogra, kp=40 decente, antes del filtro se tenia Kp=50     0.23     70   0.025
float uO = 0.0, eO = 0.0, upO, uiO, udO, e_1O, ui_1O, u_absO;
//************************************
//FILTRO PASABAJAS 6HZ Fc VELOCIDAD
float M1 , M2, rk, rk1, rk2, Out;
float b0 = 0.00299103;//0.00249377;//0.00299103;
float b1 = 0.00299103;//0.00249377;//0.00299103;
float a0 = 1.0;//1.0;
float a1 = -0.99401795;//-0.99501247;//-0.99401795;
//************************************
//FILTRO PASABAJAS 10HZ Fc ORIENTACIÓN
float MO1 , MO2, rOk, rOk1, rOk2, OutO;
float bO0 = 0.0;
float bO1 = 0.0198013266932447;
float aO0 = 1.0;
float aO1 = -0.980198673306755;
//************************************
//**************MRAC**********************************************************************************************
float eM=0.0,BP,BI,BD,uP,uI,uD;
float GammaP=-0.000000001;//-0.0000001;
float GammaI=-0.00000004,GammaD=0.0000000000000001;
float OutM,OutP,OutI,OutD;
//PROBAR CON 8 SEG DE ESTABLECIMIENTO
float TiempoEst = 8.0;
float Amortiguamiento = 0.7, alpha = 0.1;
float FrecNat = 4.0/Amortiguamiento/TiempoEst ;
// MODELO DE REFERENCIA
float M1M , M2M, rkM, rk1M, rk2M;
float b0M = 0.00;
float b1M = 0.00246702830614498;//6.271909515344530e-04;//1.581141871640755e-04;//1.62830358489683e-05;//4.07619314183456e-06;//1.01972805775648e-06;//2.55017017215136e-07;//0.000100205025180565;
float b2M = 0.00238614186238073;//6.168241502112873e-04;//1.568020359555196e-04;//1.62396722372427e-05;//4.07076183848479e-06;//1.01904846558111e-06;//2.54932025707953e-07;//-0.0000996950761376422;
float a0M = 1.0;
float a1M = -1.89998424786743;//-1.949985409398968;//-1.974994995805213;//-1.99199939212897;//-1.99599984238901;//-1.99799995989081;//-1.99899998988433;
float a2M = 0.904837418035959;//0.951229424500714;//0.975309912028333;//0.992031914837061;//0.996007989343992;//0.998001998667333;//0.999000499833375;
//*******************************
// KP
float M1P , M2P, rkP, rk1P, rk2P;
float b0P = 0.00;
float b1P = 0.000999500081633501;
float b2P = -0.000999500081633501;
float a0P = 1.0;
float a1P = -1.99899998988433;
float a2P = 0.999000499833376;
//******************************
// KI
float M1I , M2I, rkI, rk1I, rk2I;
float b0I = 0.00;
float b1I = 4.99833353741666e-07;
float b2I = 4.99666770387588e-07;
float a0I = 1.0;
float a1I = -1.99899998988433;
float a2I = 0.999000499833375;
//****************************
// KD
float M1D , M2D, rkD, rk1D, rk2D;
float b0D = 0.00;
float b1D = -1.99999974498298;
float b2D = 0.999999744982983;
float a0D = 1.0;
float a1D = -1.99899998988433;
float a2D = 0.999000499833375;
//****************
// Integrador P
float OutAP, rkAP, rk1AP;
float b0AP = 0.00;
float b1AP = 0.001;
float a1AP = -1.00;
//****************************
// Integrador I
float OutAI, rkAI, rk1AI;
//***************************
// Integrador D
float OutAD, rkAD, rk1AD;
//**************************
// DERIVADOR DE RPM
float OutRPM, Out_1RPM;
//*************************
// Integrador ERROR
float OutE, rkE, rk1E;
//**************************
//************************************
//************Bluetooth***************
BluetoothSerial SerialBT;
const int sizedata = 50;
uint8_t send_data[sizedata];
String MACadd = "EC:62:60:9E:33:E6";//Write Drone side MAC address
uint8_t address[6]  = {0xEC, 0x62, 0x60, 0x9E, 0x33, 0xE6};//Write Drone side MAC address in HEX
bool connected;
int SPo_d,SPo_f,SPo_s,SPo_2p,SPo_2u;
int e_d,e_f,e_s,e_2p,e_2u;
int Out_d,Out_f,Out_s,Out_2p,Out_2u;
int RPM_d,RPM_f,RPM_s,RPM_2p,RPM_2u;
int Ori_d,Ori_f,Ori_s,Ori_2p,Ori_2u;

int SPoO_d,SPoO_f,SPoO_s,SPoO_2p,SPoO_2u;
int eO_d,eO_f,eO_s,eO_2p,eO_2u;
int uO_d,uO_f,uO_s,uO_2p,uO_2u;

int OutAP_d,OutAP_f,OutAP_s,OutAP_2p,OutAP_2u;
int OutAI_d,OutAI_f,OutAI_s,OutAI_2p,OutAI_2u;
int OutAD_d,OutAD_f,OutAD_s,OutAD_2p,OutAD_2u;

int OutM_d,OutM_f,OutM_s,OutM_2p,OutM_2u;
//************************************
//****Magnetómetro********
/*
MechaQMC5883 qmc;
int x,y,z;
float acimut,geografico;
float declination = -6.20;
*/
float u_pos,e_p,Geo_fix;
QMC5883LCompass compass;
double azimut;
double anguloConvertido;
//************************
void ICACHE_RAM_ATTR onTimer1() {
  ms = ms + 1;
  if (ms == 1000.00) {ms = 0.0; s = s + 1;}
  t    = s + ms / 1000.00;

  w = w + 1.0;
  if (w == 100.00) {
    RPM = EncoderCount * 1.71;
    EncoderCount = 0.0;
    w = 0.0;
  }

  /*
  if (t < 5.0) SPo = 0.0;                                                      
  if ((t >= 5.00) && (t < 70.0)) SPo = 200.0;                                  
  if ((t >= 70.0) && (t < 150.0)) SPo = ((3000.0 - 200.0) / (150.0 - 70.0)) * (t - 70.0) + 200.0;
  if ((t >= 150.0) && (t < 170.0)) SPo = 3000.0;
  if ((t >= 170.0) && (t < 190.0)) SPo = ((1000.0 - 3000.0) / (190.0 - 170.0)) * (t - 170.0) + 3000.0;
  if ((t >= 190.0) && (t < 210.0)) SPo = 1000.0;
  if ((t >= 210.0) && (t < 250.0)) SPo = ((-200 - 1000.0) / (250.0 - 210.0)) * (t - 210.0) + 1000.0;
  if ((t >= 250.0) && (t < 280.0)) SPo = -200;
  if ((t >= 280.0) && (t < 320.0)) SPo = ((-1000 + 200.0) / (320.0 - 280.0)) * (t - 280.0) - 200.0;
  if ((t >= 320.0) && (t < 360.0)) SPo = -1000;
  if ((t >= 360.0) && (t < 400.0)) SPo = ((0 + 1000.0) / (400.0 - 360.0)) * (t - 360.0) - 1000.0;
  if (t >= 400.0) {t = 0.0;ms = 0, s = 0;}
  */

 if (t < 60.0) SPoO = 80.0;
 if ((t >= 60.0) && (t < 120.0)) SPoO = 100.0;
// if ((t >= 80.0) && (t < 150.0)) SPoO = 120.0;

//  if ((t >= 50.0) && (t < 70.0)) SPoO = ((200.0 - 50.0) / (70.0 - 50.0))*(t - 50.0) + 50.0;
 // if ((t >= 70.0) && (t < 90.0)) SPoO = 200.0;
 // if ((t >= 90.0) && (t < 110.0)) SPoO = ((280.0 - 200.0) / (110.0 - 90.0))*(t - 90.0) + 200.0;
 // if ((t >= 110.0) && (t < 130.0)) SPoO = 280.0;
 // if ((t >= 130.0) && (t < 150.0)) SPoO = ((350.0 - 280.0) / (150.0 - 130.0))*(t - 130.0) + 280.0;
 // if ((t >= 150.0) && (t < 170.0)) SPoO = 350;
 // if ((t >= 170.0) && (t < 190.0)) SPoO = ((20.0 - 350.0) / (190.0 - 170.0))*(t - 170.0) + 350.0;
 // if ((t >= 190.0) && (t < 210.0)) SPoO = 20;
  if (t >= 120.0) {t = 0.0;ms = 0, s = 0;}
  //***************CONTROL DE ORIENTACIÓN*************
  //eO = SPoO - geografico;
  eO =  anguloConvertido - SPoO;
  //Control Proporcional Orientación
  upO = KpO * eO;
  //upO = constrain (upO, -3000.0, 3000.0);
  //Control Integral Orientación
  uiO = eO * 0.001 + ui_1O;
  ui_1O = uiO;
  uiO = KiO * uiO;
  uiO = constrain(uiO, -3000.0, 3000.0);
  //Control Derivativo Orientación
  udO = (eO - e_1O) / 0.001;
  e_1O = eO;
  //udO = constrain(udO, -3000.0, 3000.0);
  udO = KdO * udO;
  uO = upO + uiO + udO;
  uO = constrain(uO, -3000.0, 3000.0);
  //***************CONTROL DE VELOCIDAD***************
  //********************
  Out = constrain(Out, -3000.0, 3000.0);

  e = uO - Out;
  //e = OutO - Out;
  //e = SPo - Out;
  eM = Out - OutM;
  BP = OutP*eM*GammaP;
  BI = OutI*eM*GammaI;
  BD = OutD*eM*GammaD;
  if (OutAP<0) {OutAP=0;}
  else {OutAP = OutAP;}
  uP = OutAP*e;
  uP = constrain(uP, -255.0, 255.0);
  if (OutAI<0) {OutAI=0;}
  else {OutAI = OutAI;}
  uI = OutAI*OutE;
  uI = constrain(uI, -255.0, 255.0);
  //if (OutAD<0) {OutAD=0;}
  //else {OutAD = OutAD;}
  uD = OutAD*OutRPM;
  uD = constrain(uD, -255.0, 255.0);
  u = uP + uI + uD;
  //********************
  //Signo
  if (u >= 0) {
    digitalWrite(D4, HIGH);
  }
  else {
    digitalWrite(D4, LOW);
  }
  u_abs = abs(u);
  if (u_abs >= 255) {
    u_abs = 255;
  }
  else {
    u_abs = u_abs;
  }
  u_abs = constrain(u_abs, 0, 255); //174 de límite quizas arregle el problema de que aveces el motor se detiene
  DutyCicle = u_abs; // 248 se detiene
  DutyCicleInverso = 255 - DutyCicle;
}

void ICACHE_RAM_ATTR onTimer2() {
  Out = rk * b0 + M2;
  rk  = RPM - M1;
  //rk2 = rk1;
  rk1 = rk;
  M1  = rk1 * a1; //+rk2*a2;
  M2  = rk1 * b1; //+rk2*b2;
  Out = constrain(Out, -3000.0, 3000.0);

  OutO = rOk * bO0 + MO2;
  rOk  = uO - MO1;
  //rk2 = rk1;
  rOk1 = rOk;
  MO1  = rOk1 * aO1; //+rk2*a2;
  MO2  = rOk1 * bO1; //+rk2*b2;
  OutO = constrain(OutO, -3000.0, 3000.0);
}

void ICACHE_RAM_ATTR onTimer3() {
  SerialBT.write(send_data, sizedata);
}

void ICACHE_RAM_ATTR onTimer4() {
  
  //KP
  rkP = e - M1P;
  OutP = rkP*b0P + M2P;
  rk2P = rk1P;
  rk1P = rkP;
  M1P = rk1P*a1P + rk2P*a2P;
  M2P = rk1P*b1P + rk2P*b2P;
  //***********************
  //KI
  rkI = e - M1I;
  OutI = rkI*b0I + M2I;
  rk2I = rk1I;
  rk1I = rkI;
  M1I = rk1I*a1I + rk2I*a2I;
  M2I = rk1I*b1I + rk2I*b2I;
  //***********************
  //KD
  rkD = Out - M1D;
  OutD = rkD*b0D + M2D;
  rk2D = rk1D;
  rk1D = rkD;
  M1D = rk1D*a1D + rk2D*a2D;
  M2D = rk1D*b1D + rk2D*b2D;
  //***********************
  //INTEGRADOR P
  OutAP = rkAP*b0AP + rk1AP*b1AP;
  rkAP = BP - rk1AP*a1AP;
  rk1AP = rkAP;
  //INTEGRADOR I
  OutAI = rkAI*b0AP + rk1AI*b1AP;
  rkAI = BI - rk1AI*a1AP;
  rk1AI = rkAI;
  if (OutAI<0) {OutAI=0;}
  else {OutAI = OutAI;}
  //INTEGRADOR D
  OutAD = rkAD*b0AP + rk1AD*b1AP;
  rkAD = BD - rk1AD*a1AP;
  rk1AD = rkAD;
  //if (OutAD<0) {OutAD=0;}
  //else {OutAD = OutAD;}
  //************************
  //DERIVADOR RPM
  OutRPM = (Out - Out_1RPM)/0.001;
  Out_1RPM = Out;
  //***********************
  //INTEGRADOR ERROR
  OutE = rkE*b0AP + rk1E*b1AP;
  rkE = e - rk1E*a1AP;
  rk1E = rkE;
  //************************
}

void ICACHE_RAM_ATTR onTimer5() {
  //MODELO DE REFERENCIA
  //rkM = SPo - M1M;
  rkM = uO - M1M;
  //rkM = OutO - M1M;
  OutM = rkM*b0M + M2M;
  rk2M = rk1M;
  rk1M = rkM;
  M1M = rk1M*a1M + rk2M*a2M;
  M2M = rk1M*b1M + rk2M*b2M;
  //********************
}

void ICACHE_RAM_ATTR onTimer6() {
  //Magentómetro**********
  compass.read();
  azimut = compass.getAzimuth();
  anguloConvertido = convertirRango(azimut);
}

void ICACHE_RAM_ATTR ISR_EncoderA() {
  PinB = digitalRead(interruptPinB);
  PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }
}

void ICACHE_RAM_ATTR ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }
}

void setup() {
  Serial.begin(9600);
  //**************OTA*******************
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando...");
  while (WiFi.status()!= WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname("Cubesat");
  ArduinoOTA.setPassword("Cubesat");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    Serial.println("Iniciando Programación " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nTerminando");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("Listo");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
  //************************************
  //**************Bluetooth*************
  SerialBT.begin("ESP32test", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  connected = SerialBT.connect(address);
  if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
//*************************************************
  Wire.begin(21,22);
  //qmc.init();
  compass.init();
  ledcSetup(canal,frecuencia,resolucion);
  ledcAttachPin(D23,canal);
  Timer1.attach_ms(1, onTimer1);
  Timer2.attach_ms(1, onTimer2);
  Timer3.attach_ms(100, onTimer3);
  Timer4.attach_ms(1, onTimer4);
  Timer5.attach_ms(25, onTimer5);
  Timer6.attach_ms(40, onTimer6);
  pinMode(D4, OUTPUT);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  //Serial.println("SPo,RPM,e,u");
}

uint8_t calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];

  checksum |= 0b10100000 & data[5];
  checksum |= 0b01010000 & data[6];
  checksum |= 0b00101000 & data[7];
  checksum |= 0b00010100 & data[8];

  checksum |= 0b10010000 & data[9];
  checksum |= 0b01001000 & data[10];
  checksum |= 0b00100100 & data[11];
  checksum |= 0b00010010 & data[12];
  checksum |= 0b00001001 & data[13];

  checksum |= 0b10001000 & data[14];
  checksum |= 0b01000100 & data[15];
  checksum |= 0b00100010 & data[16];
  checksum |= 0b00010001 & data[17];
  checksum |= 0b11100000 & data[18];
  checksum |= 0b01110000 & data[19];
  checksum |= 0b00111000 & data[20];

  checksum |= 0b00011100 & data[21];
  checksum |= 0b00001110 & data[22];
  checksum |= 0b00000111 & data[23];
  checksum |= 0b11110000 & data[24];

  checksum |= 0b01111000 & data[25];
  checksum |= 0b00111100 & data[26];
  checksum |= 0b00011110 & data[27];
  checksum |= 0b00001111 & data[28];
  checksum |= 0b11111000 & data[29];
  checksum |= 0b01111100 & data[30];
  checksum |= 0b00111110 & data[31];
  checksum |= 0b00011111 & data[32];
  //********************************
  checksum |= 0b10101000 & data[33];
  checksum |= 0b01010100 & data[34];
  checksum |= 0b00101010 & data[35];
  checksum |= 0b00010101 & data[36];

  checksum |= 0b01100000 & data[37];
  checksum |= 0b00011000 & data[38];
  checksum |= 0b00000110 & data[39];
  checksum |= 0b10101010 & data[40];

  checksum |= 0b01010101 & data[41];
  checksum |= 0b11000001 & data[42];
  checksum |= 0b11100001 & data[43];
  checksum |= 0b11110001 & data[44];

  checksum |= 0b10000001 & data[45];
  checksum |= 0b10000011 & data[46];
  checksum |= 0b10000111 & data[47];
  checksum |= 0b10001111 & data[48];

  return checksum;
}
/*
int convertirRango(int angulo) {
  // Asegurar que el ángulo esté en el rango de -180 a 180
  angulo = angulo % 360;
  // Convertir el rango de -180 a 180 a 0 a 360
  if (angulo < 0) {angulo += 360;}
  return angulo;
}
*/
double convertirRango(double angulo) {
  // Asegurar que el ángulo esté en el rango de -180 a 180
  angulo = fmod(angulo, 360.0d);
  // Convertir el rango de -180 a 180 a 0 a 360
  if (angulo < 0) {angulo += 360.0d;}
  return angulo;
}

void obtenerPartes(float numero, int &parteEntera, int &parteFraccionaria, int &signo,int &parteSuperior, int &parteInferior) {
  parteEntera = int(numero);                 // Obtener la parte entera
  parteFraccionaria = abs(int((numero-parteEntera) * 100)); // Obtener la parte fraccionaria (abs para asegurarse de que sea positiva)
  signo = (numero >= 0) ? 1 : 0;            // Obtener el signo (1 para positivo, 0 para negativo)
  parteInferior = abs(parteEntera % 100); // Obtener los dos dígitos menos significativos
  parteSuperior = abs((parteEntera / 100) % 100); // Obtener los dos dígitos más significativos
}

void obtenerPartesDEC(float numero, int &parteEntera, int &parteFraccionaria, int &signo,int &parteSuperior, int &parteInferior) {
  parteEntera = int(numero);                 // Obtener la parte entera
  parteFraccionaria = abs(int((numero-parteEntera) * 10000)); // Obtener la parte fraccionaria (abs para asegurarse de que sea positiva)
  signo = (numero >= 0) ? 1 : 0;            // Obtener el signo (1 para positivo, 0 para negativo)
  parteInferior = abs(parteFraccionaria % 100); // Obtener los dos dígitos menos significativos
  parteSuperior = abs((parteFraccionaria / 100) % 100); // Obtener los dos dígitos más significativos
}

void loop() {
  ArduinoOTA.handle();
  
  ledcWrite(canal,DutyCicleInverso);
  //ledcWrite(canal,240);
  obtenerPartes(SPo,SPo_d,SPo_f,SPo_s,SPo_2p,SPo_2u);
  obtenerPartes(Out,Out_d,Out_f,Out_s,Out_2p,Out_2u);
  obtenerPartes(e,e_d,e_f,e_s,e_2p,e_2u);
  //obtenerPartes(DutyCicleInverso,DutyCicleInverso_d,DutyCicleInverso_f,DutyCicleInverso_s);
  obtenerPartes(RPM,RPM_d,RPM_f,RPM_s,RPM_2p,RPM_2u);
  obtenerPartes(anguloConvertido,Ori_d,Ori_f,Ori_s,Ori_2p,Ori_2u);
  obtenerPartes(SPoO,SPoO_d,SPoO_f,SPoO_s,SPoO_2p,SPoO_2u);
  obtenerPartes(eO,eO_d,eO_f,eO_s,eO_2p,eO_2u);
  obtenerPartes(uO,uO_d,uO_f,uO_s,uO_2p,uO_2u);

  obtenerPartesDEC(OutAP,OutAP_d,OutAP_f,OutAP_s,OutAP_2p,OutAP_2u);
  obtenerPartesDEC(OutAI,OutAI_d,OutAI_f,OutAI_s,OutAI_2p,OutAI_2u);
  obtenerPartesDEC(OutAD,OutAD_d,OutAD_f,OutAD_s,OutAD_2p,OutAD_2u);

  obtenerPartes(OutM,OutM_d,OutM_f,OutM_s,OutM_2p,OutM_2u);

  
  send_data[0] = 'T';
  send_data[1] = SPo_2p;
  send_data[2] = SPo_2u;
  send_data[3] = SPo_f;
  send_data[4] = SPo_s;
  send_data[5] = Out_2p;
  send_data[6] = Out_2u;
  send_data[7] = Out_f;
  send_data[8] = Out_s;
  send_data[9] = e_2p;
  send_data[10] = e_2u;
  send_data[11] = e_f;
  send_data[12] = e_s;
  send_data[13] = int(DutyCicleInverso);
  send_data[14] = RPM_2p;
  send_data[15] = RPM_2u;
  send_data[16] = RPM_f;
  send_data[17] = RPM_s;
  send_data[18] = Ori_2p;
  send_data[19] = Ori_2u;
  send_data[20] = Ori_f;
  send_data[21] = SPoO_2p;
  send_data[22] = SPoO_2u;
  send_data[23] = SPoO_f;
  send_data[24] = SPoO_s;

  send_data[25] = eO_2p;
  send_data[26] = eO_2u;
  send_data[27] = eO_f;
  send_data[28] = eO_s;

  send_data[29] = uO_2p;
  send_data[30] = uO_2u;
  send_data[31] = uO_f;
  send_data[32] = uO_s;

  //*********************
  send_data[33] = OutAP_d;
  send_data[34] = OutAP_2p;
  send_data[35] = OutAP_2u;
  send_data[36] = OutAP_s;

  send_data[37] = OutAI_d;
  send_data[38] = OutAI_2p;
  send_data[39] = OutAI_2u;
  send_data[40] = OutAI_s;

  send_data[41] = OutAD_d;
  send_data[42] = OutAD_2p;
  send_data[43] = OutAD_2u;
  send_data[44] = OutAD_s;

  send_data[45] = OutM_2p;
  send_data[46] = OutM_2u;
  send_data[47] = OutM_f;
  send_data[48] = OutM_s;
  send_data[sizedata-1] = calculate_checksum(send_data);
}
