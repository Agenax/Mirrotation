//Librairies :

#include <Stepper.h>
#include <Arduino.h>
#include <Wire.h>
#include <String.h>
#include <stdlib.h>
#include <SensirionI2CSht4x.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//////////////////////////////////////////////////////////////////////////  
//VARIABLES INITIALISATION
/////////////////////////////////////////////////////////// MOTOR 
const int stepsPerRevolution = 32 * 64;
Stepper myStepper1(stepsPerRevolution, 33, 26, 25, 32);
Stepper myStepper2(stepsPerRevolution, 27, 4, 16, 14);
Stepper myStepper3(stepsPerRevolution, 18, 17, 5, 19);
int motor = 0;
int steps = 0;
char *motorchar = NULL;
int mt1Last = -1;
int mt2Last = -2;
int mt3Last = -3;
int backlash1 = 20;
int backlash2 = 20;
int backlash3 = 20;
int steptd1 = 0;
int steptd2 = 0;
int steptd3 = 0;
/////////////////////////////////////////////////////////// ENVIRONMENTAL SENSORS
SensirionI2CSht4x sensorint;
SensirionI2CSht4x sensorout;
SensirionI2CSht4x sensorout2;
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0
static int16_t errorint;
static int16_t errorout;
static int16_t errorout2;

float tint;
float hint;
float tout;
float hout;
float tout02;
float hout02;
float tint1;
float hint1;
float tout1;
float hout1;
float tout2;
float hout2;
bool cntdint = false;
bool cntdout = false;
bool cntdout2 = false;

int S11 = 33;
int S12 = 26;
int S13 = 25;
int S14 = 32;
int S21 = 27;
int S22 = 4;
int S23 = 16;
int S24 = 14;
int S31 = 18;
int S32 = 17;
int S33 = 5;
int S34 = 19;
int mode1 = 0;
int mode2 = 0;
int mode3 = 0;
////////////////////////////////////////////////////////// PWM OUTPUT
int cyc1;
int freq1;
int rez1;
int cyc2;
int freq2;
int rez2;
const byte pwm_gpio1 = 12;
const byte pwm_gpio2 = 13;
const byte led15 = 15;
const byte led2 = 2;
const byte led22 = 22;

////////////////////////////////////////////////////////// BLUETOOTH 
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t value = 0;

String output;
String outputbt;
String d1;
String d2;
String e1;
String e2;
String str1, str2, str3, strT, strS;
String rxValue1;
String buffer[32];
int bi = 0;
int br = 0;
int output12;
int output22;
float Stime = 1.5;
float lastT = 0.1;
float EnvMesT = 5000;
float lastTempT = 0;
int ledB = 20;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        rxValue1 = String(rxValue.c_str());
        //Serial.println("*********");
        //Serial.print("Received Value: ");
        //for (int i = 0; i < rxValue.length(); i++)
          //{Serial.print(rxValue[i]);}

        //Serial.println();
        //Serial.println("*********");
        rxValue = "";
      }
    }
};

///////////////////////////////////Test remains
#define TWO_SEC 40
#define DUTY 50
//Variables
byte countState;
byte lastCountState = HIGH;
byte zeroState;
byte lastZeroState = HIGH;
uint16_t pressCount = 0, dutyCount = 0;
uint32_t prevTime = 0;
bool countPress = false, zeroPress = false;

void loopAux(void *pvParameters){    // Boucle delayée pour les moteurs et auxiliaires, chaque tour -> 1pas et temps de boucle fct de vitesse
                                    // auxiliaires en plus voire tous les x boucles pour ne pas surcharger
  while (1){
      if (millis()-lastTempT > EnvMesT ){
      lastTempT = millis();
      errorint = sensorint.measureHighPrecision(tint, hint);
      if (errorint != NO_ERROR) {cntdint = false;}
      else {cntdint = true;}
      errorout = sensorout.measureHighPrecision(tout, hout);
      if (errorout != NO_ERROR) {cntdout = false;}
      else {cntdout = true;}
      errorout2 = sensorout2.measureHighPrecision(tout02, hout02);
      if (errorout2 != NO_ERROR) {cntdout2 = false;}
      else {cntdout2 = true;}

      if (! isnan(tint))
        {tint1 = tint;}
      if (! isnan(hint))
        {hint1 = hint;}

      if (! isnan(tout))
        {tout1 = tout;}
      if (! isnan(hout))
        {hout1 = hout;}

      if (! isnan(tout02))
        {tout2 = tout02;}
      if (! isnan(hout02))
        {hout2 = hout02;}

      if( cntdint == false ){
        if(deviceConnected){
          String strNC = "10";
          pCharacteristic->setValue(strNC.c_str());
          pCharacteristic->notify();
          value++;
          led(255, 0, 0, ledB);
          delay(10);
          }
          }
      if( cntdint == true ) {
        String strT = str1 + tint1 + strS + hint1;
        if(deviceConnected){
          pCharacteristic->setValue(strT.c_str());
          pCharacteristic->notify();
          value++;
          led(255, 0, 0, ledB);
          delay(10);
          }
        Serial.println(strT);
        }

      if( cntdout == false ){
        if(deviceConnected){
          String strNC = "20";
          pCharacteristic->setValue(strNC.c_str());
          pCharacteristic->notify();
          value++;
          led(255, 0, 0, ledB);
          delay(10);
          }
          }
      else {
        strT = str2 + tout1 + strS + hout1;
        if(deviceConnected){
          pCharacteristic->setValue(strT.c_str());
          pCharacteristic->notify();
          value++;
          led(255, 0, 0, ledB);
          delay(10);
          }
        Serial.println(strT);
        }

      if( cntdout2 == false ){
        if(deviceConnected){
          String strNC = "30";
          pCharacteristic->setValue(strNC.c_str());
          pCharacteristic->notify();
          value++;
          led(0, 0, 255, ledB);
          delay(10);
          }
          }
      else {
        strT = str3 + tout2 + strS + hout2;
        if(deviceConnected){
          pCharacteristic->setValue(strT.c_str());
          pCharacteristic->notify();
          value++;
          led(255, 0, 0, ledB);
          delay(10);
          }
        Serial.println(strT);
      }
    }

        if (deviceConnected) {
          led(255, 0, 0, ledB);
        }

        if(millis()-lastT>Stime)
        {
        lastT = millis();
        if(steptd1 > 1){Step(1); steptd1 += -1;}
        if(steptd2 > 1){Step(2); steptd2 += -1;}
        if(steptd3 > 1){Step(3); steptd3 += -1;}
        if(steptd1 < -1){Step(-1); steptd1 += 1;}
        if(steptd2 < -1){Step(-2); steptd2 += 1;}
        if(steptd3 < -1){Step(-3); steptd3 += 1;}
        if(steptd1 == 1 || steptd1 == -1){
          digitalWrite(S11, LOW);
          digitalWrite(S12, LOW);
          digitalWrite(S13, LOW);
          digitalWrite(S14, LOW);
          steptd1 = 0;}
        if(steptd2 == 1 || steptd2 == -1){
          digitalWrite(S21, LOW);
          digitalWrite(S22, LOW);
          digitalWrite(S23, LOW);
          digitalWrite(S24, LOW);
          steptd2 = 0;}
        if(steptd3 == 1 || steptd3 == -1){
          digitalWrite(S31, LOW);
          digitalWrite(S32, LOW);
          digitalWrite(S33, LOW);
          digitalWrite(S34, LOW);
          steptd3 = 0;}
        }
        // Auxiliary actions depending on the input - add list analysis

        // something like a while loop working till the input list isn't voided
        output12 = 0;
        d1 = '\0';
        d2 = '\0';
        if (br!=bi){
          String outputN = buffer[br];
          int index = outputN.indexOf(' ');
          int endstr = outputN.length();
          d1 = outputN.substring(0,index);
          d2 = outputN.substring(index+1, endstr);
          //e1 = buffer[br][0];
          int l1 = d1.length();
          char output1[l1 + 1];
          strcpy(output1, d1.c_str());
          output12 = atoi(output1);
          //d2 = buffer[br][1];
          buffer[br] = '\0';
          br +=1;
          if (br == 32){br = 0;}
        }

        if(output12 == 1 || output12 == -1){
          int l2 = d2.length();
          char output2[l2 + 1];
          strcpy(output2, d2.c_str());
          int output22 = atoi(output2);
          if(output12 == mt1Last){
            mt1Last = -output12;
            output22 += backlash1;
          }
          steptd1 = output22*output12/abs(output12);
          
        }

        if(output12 == 2 || output12 == -2){
          int l2 = d2.length();
          char output2[l2 + 1];
          strcpy(output2, d2.c_str());
          int output22 = atoi(output2);
          if(output12 == mt2Last){
            mt2Last = -output12;
            output22 += backlash2;
          }
          steptd2 = output22*output12/abs(output12);
        }

        if(output12 == 3 || output12 == -3){
          int l2 = d2.length();
          char output2[l2 + 1];
          strcpy(output2, d2.c_str());
          int output22 = atoi(output2);
          if(output12 == mt3Last){
            mt3Last = -output12;
            output22 += backlash3;
          }
          steptd3 = output22*output12/abs(output12);
        }

        if(output12 == 4){
          int index1 = e2.indexOf(' ');
          int endstr1 = e2.length();
          //String cycstr = d2.substring(0,index1);
          String PWMindex = e2.substring(0,index1);
          int l2 = PWMindex.length();
          char outputindex[l2 + 1];
          strcpy(outputindex, e2.c_str());
          int outputindex2 = atoi(outputindex);
          String d3 = e2.substring(index1+1, endstr1);

          switch (outputindex2) {
            case 0:
              {int index1 = d3.indexOf(' ');
              int endstr1 = d3.length();
              String cycstr = d3.substring(0,index1);
              String d4 = d3.substring(index1+1, endstr1);

              index1 = d4.indexOf(' ');
              endstr1 = d4.length();
              String freqstr = d4.substring(0,index1);
              String rezstr = d4.substring(index1+1, endstr1);
          
              int lcyc = cycstr.length();
              char cycchr[lcyc + 1];
              strcpy(cycchr, cycstr.c_str());
              int cyc2 = atoi(cycchr);
              Serial.println(cyc2);
          
              int lfreq = freqstr.length();
              char freqchr[lfreq + 1];
              strcpy(freqchr, freqstr.c_str());
              int freq2 = atoi(freqchr);
              Serial.println(freq2);
          
              int lrez = rezstr.length();
              char rezchr[lrez + 1];
              strcpy(rezchr, rezstr.c_str());
              int rez2 = atoi(rezchr);
              Serial.println(rez2);
              double power2 = pow(2,rez2);
              double cyc22 = cyc2 * power2 / 100;
              //analogWrite(12, cyc12, freq1, rez1, 0);
              ledcSetup(0, freq2, rez2);
              ledcWrite(0, cyc22);
              Serial.println(cyc22);
              }
              break;
            case 1:
              Serial.println("case1");
              break;
            case 2 :
              Serial.println("case2");
              break;
            case 3 :
              Serial.println("case3");
              break;
            case 4 :
              Serial.println("case4");
              break;
            case 5 :
              ledcWrite(0, 250);
              Serial.println("case5");
              break;
          } 
          //Serial.println(rez1);
        }

        if(output12 == 5){
          int index1 = e2.indexOf(' ');
          int endstr1 = e2.length();
          //String cycstr = d2.substring(0,index1);
          String PWMindex = e2.substring(0,index1);
          int l2 = PWMindex.length();
          char outputindex[l2 + 1];
          strcpy(outputindex, e2.c_str());
          int outputindex2 = atoi(outputindex);
          String d3 = e2.substring(index1+1, endstr1);

          switch (outputindex2) {
            case 0:
              {int index1 = d3.indexOf(' ');
              int endstr1 = d3.length();
              String cycstr = d3.substring(0,index1);
              String d4 = d3.substring(index1+1, endstr1);

              index1 = d4.indexOf(' ');
              endstr1 = d4.length();
              String freqstr = d4.substring(0,index1);
              String rezstr = d4.substring(index1+1, endstr1);
          
              int lcyc = cycstr.length();
              char cycchr[lcyc + 1];
              strcpy(cycchr, cycstr.c_str());
              int cyc1 = atoi(cycchr);
              Serial.println(cyc1);
          
              int lfreq = freqstr.length();
              char freqchr[lfreq + 1];
              strcpy(freqchr, freqstr.c_str());
              int freq1 = atoi(freqchr);
              Serial.println(freq1);
          
              int lrez = rezstr.length();
              char rezchr[lrez + 1];
              strcpy(rezchr, rezstr.c_str());
              int rez1 = atoi(rezchr);
              Serial.println(rez1);
              double power = pow(2,rez1);
              double cyc12 = cyc1 * power / 100;
              //analogWrite(12, cyc12, freq1, rez1, 0);
              ledcSetup(1, freq1, rez1);
              ledcWrite(1, cyc12);
              Serial.println(cyc12);
              }
              break;
            case 1:
              Serial.println("case1");
              break;
            case 2 :
              Serial.println("case2");
              break;
            case 3 :
              Serial.println("case3");
              break;
            case 4 :
              Serial.println("case4");
              break;
            case 5 :
              ledcWrite(1, 250);
              Serial.println("case5");
              break;
          } 
        }
        ////////////// Led brighness level
        if(output12 == 6){
          int l2 = e2.length();
          char output2[l2 + 1];
          strcpy(output2, e2.c_str());
          int ledB1 = atoi(output2);
          if (ledB1>=0 && ledB1 <= 100){
            ledB = ledB1;
            }
        }

        if( output12 == 777 ){
          Serial.println("1911");
        }

        if(output12 == 8){
          int index1 = e2.indexOf(' ');
          int endstr1 = e2.length();
          //String cycstr = d2.substring(0,index1);
          String bckindex = e2.substring(0,index1);
          int l2 = bckindex.length();
          char outputindex[l2 + 1];
          strcpy(outputindex, e2.c_str());
          int outputindex2 = atoi(outputindex);
          String d3 = e2.substring(index1+1, endstr1);

          switch (outputindex2) {
            case 0:{
              String mesBck = "8 " + backlash1 + strS + backlash2 + strS + backlash3; 
              if(deviceConnected){
                pCharacteristic->setValue(mesBck.c_str());
                pCharacteristic->notify();
                value++;
                led(255, 0, 0, ledB);
                delay(10);
                }
              Serial.println(mesBck);
            }
            break;
            case 1:{
              int endstr1 = d3.length();
              char d3chr[endstr1 + 1];
              strcpy(d3chr, d3.c_str());
              int backlash1 = atoi(d3chr);
              }
              break;
            case 2:{
              int endstr1 = d3.length();
              char d3chr[endstr1 + 1];
              strcpy(d3chr, d3.c_str());
              int backlash2 = atoi(d3chr);
              }
              break;
            case 3:{
              int endstr1 = d3.length();
              char d3chr[endstr1 + 1];
              strcpy(d3chr, d3.c_str());
              int backlash3 = atoi(d3chr);
              }
              break;
        }
        }


  }
}












void loopData(void *pvParameters){   // Boucle des communictions serial et bt (rapide)
  while (1) {

String output="";
if(Serial.available()>0){
  output = Serial.readStringUntil('\n');
  int index = output.indexOf(' ');
  int endstr = output.length();
  led(255, 0, 0, ledB);
  d1 = output.substring(0,index);
  d2 = output.substring(index+1, endstr);
  buffer[bi]=output;
  bi += 1;
  if (bi == 32){bi = 0;}
  d1 = "";
  d2 = "";
  }
String outputbt="";
if(rxValue1.length() > 0){
  outputbt = rxValue1;
  rxValue1 = "";
  int index = outputbt.indexOf(' ');
  int endstr = outputbt.length();
  d1 = outputbt.substring(0,index);
  d2 = outputbt.substring(index+1, endstr);
  led(255, 0, 0, ledB);
  
  buffer[bi]=outputbt;
  bi += 1;
  if (bi == 32){bi = 0;} 
  d1 = "";
  d2 = "";
}


////////////////////////////////////////////////////////////
/////////////////////////////// REECRITURE NECESSIARE EN LISTE D'EVENEMENTS //////////
//                         Faire une liste comportant output12 - d2
/////////////////////////////////////////////////////////////////

if (!deviceConnected && oldDeviceConnected) {
  delay(500);
  pServer->startAdvertising(); 
  Serial.println("start advertising");
  oldDeviceConnected = deviceConnected;
}
if (deviceConnected && !oldDeviceConnected) {
  oldDeviceConnected = deviceConnected;
}

  }
}











void setup(){
  Serial.begin(115200);
  //////////////Mode selector
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);

  ///////////////data strings
  str1 = String("1 ");
  str2 = String("2 ");
  str3 = String("3 ");
  strS = String(" ");

  /////////////PWM setup
  ledcAttachPin(pwm_gpio1, 0);
  ledcAttachPin(pwm_gpio2, 1);
  ledcSetup(0, 4000, 8);
  ledcSetup(1, 4000, 8);
  ledcAttachPin(led2, 2);
  ledcAttachPin(led15,15);
  ledcAttachPin(led22,22);
  led(255, 0, 0, ledB);
  
  /////////////Sensors
  Wire.begin();
  sensorint.begin(Wire, SHT40_I2C_ADDR_44);
  sensorint.softReset();
  sensorout.begin(Wire, SHT40_I2C_ADDR_45);
  sensorout.softReset();
  sensorout2.begin(Wire, SHT40_I2C_ADDR_46);
  sensorout2.softReset();

  ////////////////////////////Bluetooth
  BLEDevice::init("Mirrotation N°1");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  xTaskCreatePinnedToCore(loopAux, "loopAux", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(loopData, "loopData", 4096, NULL, 1, NULL, 0);
}

void loop(){}

void led(int r,int g,int b, int l){
  ledcSetup(15, 5000, 8);
  ledcWrite(15, 255 - (r * l / 100));
  ledcSetup(2, 5000, 8);
  ledcWrite(2, 255 - (g * l / 100));
  ledcSetup(23, 5000, 8);
  ledcWrite(23, 255 - (b * l / 100));
}  
/*
const int stepsPerRevolution = 32 * 64;
Stepper myStepper1(stepsPerRevolution, 33, 26, 25, 32);
Stepper myStepper2(stepsPerRevolution, 27, 4, 16, 14);
Stepper myStepper3(stepsPerRevolution, 18, 17, 5, 19);
*/

void Step(int m){
  switch (m){
    case 1:{
      switch (mode1){
        case 0:{
        digitalWrite(S11, HIGH);
        digitalWrite(S12, LOW);
        digitalWrite(S13, HIGH);
        digitalWrite(S14, LOW);
        mode1 += 1;
        break;
        }
        case 1:{
        digitalWrite(S11, LOW);
        digitalWrite(S12, HIGH);
        digitalWrite(S13, HIGH);
        digitalWrite(S14, LOW);
        mode1 += 1;
        break;
        }
        case 2:{
        digitalWrite(S11, LOW);
        digitalWrite(S12, HIGH);
        digitalWrite(S13, LOW);
        digitalWrite(S14, HIGH);
        mode1 += 1;
        break;
        }
        case 3:{
        digitalWrite(S11, HIGH);
        digitalWrite(S12, LOW);
        digitalWrite(S13, LOW);
        digitalWrite(S14, HIGH);
        mode1 = 0;
        break;
        }
      }
    }
    break;
    case 2:{
          switch (mode2){
            case 0:{
            digitalWrite(S21, HIGH);
            digitalWrite(S22, LOW);
            digitalWrite(S23, HIGH);
            digitalWrite(S24, LOW);
            mode2 += 1;
            break;
            }
            case 1:{
            digitalWrite(S21, LOW);
            digitalWrite(S22, HIGH);
            digitalWrite(S23, HIGH);
            digitalWrite(S24, LOW);
            mode2 += 1;
            break;
            }
            case 2:{
            digitalWrite(S21, LOW);
            digitalWrite(S22, HIGH);
            digitalWrite(S23, LOW);
            digitalWrite(S24, HIGH);
            mode2 += 1;
            break;
            }
            case 3:{
            digitalWrite(S21, HIGH);
            digitalWrite(S22, LOW);
            digitalWrite(S23, LOW);
            digitalWrite(S24, HIGH);
            mode2 = 0;
            break;
            }
          }
        }
        break;
    case 3:{
          switch (mode3){
            case 0:{
            digitalWrite(S31, HIGH);
            digitalWrite(S32, LOW);
            digitalWrite(S33, HIGH);
            digitalWrite(S34, LOW);
            mode3 += 1;
            break;
            }
            case 1:{
            digitalWrite(S31, LOW);
            digitalWrite(S32, HIGH);
            digitalWrite(S33, HIGH);
            digitalWrite(S34, LOW);
            mode3 += 1;
            break;
            }
            case 2:{
            digitalWrite(S31, LOW);
            digitalWrite(S32, HIGH);
            digitalWrite(S33, LOW);
            digitalWrite(S34, HIGH);
            mode3 += 1;
            break;
            }
            case 3:{
            digitalWrite(S31, HIGH);
            digitalWrite(S32, LOW);
            digitalWrite(S33, LOW);
            digitalWrite(S34, HIGH);
            mode3 = 0;
            break;
            }
          }
        }
        break;
    case -1:{
      switch (mode1){
        case 0:{
        digitalWrite(S11, HIGH);
        digitalWrite(S12, LOW);
        digitalWrite(S13, HIGH);
        digitalWrite(S14, LOW);
        mode1 += 3;
        break;
        }
        case 1:{
        digitalWrite(S11, LOW);
        digitalWrite(S12, HIGH);
        digitalWrite(S13, HIGH);
        digitalWrite(S14, LOW);
        mode1 += -1;
        break;
        }
        case 2:{
        digitalWrite(S11, LOW);
        digitalWrite(S12, HIGH);
        digitalWrite(S13, LOW);
        digitalWrite(S14, HIGH);
        mode1 += -1;
        break;
        }
        case 3:{
        digitalWrite(S11, HIGH);
        digitalWrite(S12, LOW);
        digitalWrite(S13, LOW);
        digitalWrite(S14, HIGH);
        mode1 += -1;
        break;
        }
      }
    }
    break;
    case -2:{
          switch (mode2){
            case 0:{
            digitalWrite(S21, HIGH);
            digitalWrite(S22, LOW);
            digitalWrite(S23, HIGH);
            digitalWrite(S24, LOW);
            mode2 = 3;
            break;
            }
            case 1:{
            digitalWrite(S21, LOW);
            digitalWrite(S22, HIGH);
            digitalWrite(S23, HIGH);
            digitalWrite(S24, LOW);
            mode2 += -1;
            break;
            }
            case 2:{
            digitalWrite(S21, LOW);
            digitalWrite(S22, HIGH);
            digitalWrite(S23, LOW);
            digitalWrite(S24, HIGH);
            mode2 += -1;
            break;
            }
            case 3:{
            digitalWrite(S21, HIGH);
            digitalWrite(S22, LOW);
            digitalWrite(S23, LOW);
            digitalWrite(S24, HIGH);
            mode2 += -1;
            break;
            }
          }
        }
        break;
    case -3:{
          switch (mode3){
            case 0:{
            digitalWrite(S31, HIGH);
            digitalWrite(S32, LOW);
            digitalWrite(S33, HIGH);
            digitalWrite(S34, LOW);
            mode3 = 3;
            break;
            }
            case 1:{
            digitalWrite(S31, LOW);
            digitalWrite(S32, HIGH);
            digitalWrite(S33, HIGH);
            digitalWrite(S34, LOW);
            mode3 += -1;
            break;
            }
            case 2:{
            digitalWrite(S31, LOW);
            digitalWrite(S32, HIGH);
            digitalWrite(S33, LOW);
            digitalWrite(S34, HIGH);
            mode3 += -1;
            break;
            }
            case 3:{
            digitalWrite(S31, HIGH);
            digitalWrite(S32, LOW);
            digitalWrite(S33, LOW);
            digitalWrite(S34, HIGH);
            mode3 += -1;
            break;
            }
          }
        }
        break;
  }

}