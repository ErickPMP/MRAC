#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
float SPo,Out,e,RPM,DutyCicleInverso,Ori,SPoO,eO,uO;
float KP,KI,KD;
float ModeloRef;
uint8_t recv_data[50];
//**************SIMULINK*********
typedef union{
  float number;
  uint8_t bytes[4];
}valor;

valor SPo_ML;
valor Out_ML;
valor e_ML;
valor u_ML;
valor RPM_ML;
valor Ori_ML;
valor SPoO_ML;
valor eO_ML;
valor uO_ML;
valor KP_ML;
valor KI_ML;
valor KD_ML;
valor ModeloRef_ML;
//******************************
void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
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

void loop() {
  
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 50);
    
    if (recv_data[0] != 'T') {
      Serial.println("Receive error!");
      return;
    }

    if (recv_data[49] != calculate_checksum(recv_data)) {
      Serial.println("Decode error!");
      return;
    }
    //Serial.printf("left_x: %d, left_y: %d, right_x: %d, right_y: %d\n", recv_data[1], recv_data[2], recv_data[3], recv_data[4]);
    if (recv_data[4]==1){SPo = (recv_data[1]*100.0+recv_data[2])+recv_data[3]/100.0;}
    else {SPo = -1*((recv_data[1]*100.0+recv_data[2])+recv_data[3]/100.0);}
    if (recv_data[8]==1){Out = (recv_data[5]*100.0+recv_data[6])+recv_data[7]/100.0;}
    else {Out = -1*((recv_data[5]*100.0+recv_data[6])+recv_data[7]/100.0);}
    if (recv_data[12]==1){e = (recv_data[9]*100.0+recv_data[10])+recv_data[11]/100.0;}
    else {e = -1*((recv_data[9]*100.0+recv_data[10])+recv_data[11]/100.0);}
    DutyCicleInverso = recv_data[13];
    if (recv_data[17]==1){RPM = (recv_data[14]*100.0+recv_data[15])+recv_data[16]/100.0;}
    else {RPM = -1*((recv_data[14]*100.0+recv_data[15])+recv_data[16]/100.0);}
    Ori = (recv_data[18]*100.0+recv_data[19])+recv_data[20]/100.0;

    if (recv_data[24]==1){SPoO = (recv_data[21]*100.0+recv_data[22])+recv_data[23]/100.0;}
    else {SPoO = -1*((recv_data[21]*100.0+recv_data[22])+recv_data[23]/100.0);}
    if (recv_data[28]==1){eO = (recv_data[25]*100.0+recv_data[26])+recv_data[27]/100.0;}
    else {eO = -1*((recv_data[25]*100.0+recv_data[26])+recv_data[27]/100.0);}
    if (recv_data[32]==1){uO = (recv_data[29]*100.0+recv_data[30])+recv_data[31]/100.0;}
    else {uO = -1*((recv_data[29]*100.0+recv_data[30])+recv_data[31]/100.0);}

    if (recv_data[36]==1){KP = recv_data[33]+recv_data[34]/100.0+recv_data[35]/10000.0;}
    else {KP = -1*(recv_data[33]+recv_data[34]/100.0+recv_data[35]/10000.0);}

    if (recv_data[40]==1){KI = recv_data[37]+recv_data[38]/100.0+recv_data[39]/10000.0;}
    else {KI = -1*(recv_data[37]+recv_data[38]/100.0+recv_data[39]/10000.0);}
    
    if (recv_data[44]==1){KD = recv_data[41]+recv_data[42]/100.0+recv_data[43]/10000.0;}
    else {KD = -1*(recv_data[41]+recv_data[42]/100.0+recv_data[43]/10000.0);}

    if (recv_data[48]==1){ModeloRef = (recv_data[45]*100.0+recv_data[46])+recv_data[47]/100.0;}
    else {ModeloRef = -1*((recv_data[45]*100.0+recv_data[46])+recv_data[47]/100.0);}
    /*Serial.print(SPo);
    Serial.print(",");
    Serial.print(Out);
    Serial.print(",");
    Serial.print(DutyCicleInverso);
    Serial.print(",");
    Serial.print(e);
    Serial.print(",");
    Serial.println(RPM);*/
  }
  //delay(200);
  SPo_ML.number = SPo;
    Out_ML.number = Out;
    e_ML.number = e;
    u_ML.number = DutyCicleInverso;
    RPM_ML.number = RPM;
    Ori_ML.number = Ori;
    SPoO_ML.number = SPoO;
    eO_ML.number = eO;
    uO_ML.number = uO;

    KP_ML.number = KP;
    KI_ML.number = KI;
    KD_ML.number = KD;

    ModeloRef_ML.number = ModeloRef;
    Serial.write('V');
  for(int i=0; i<4; i++){
    Serial.write(SPo_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(Out_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(e_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(u_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(RPM_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(Ori_ML.bytes[i]);
  }
 for(int i=0; i<4; i++){
    Serial.write(SPoO_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(eO_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(uO_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(KP_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(KI_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(KD_ML.bytes[i]);
  }
  for(int i=0; i<4; i++){
    Serial.write(ModeloRef_ML.bytes[i]);
  }
  Serial.write('\n');
}
