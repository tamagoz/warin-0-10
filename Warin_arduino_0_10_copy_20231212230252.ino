#include <ModbusMaster.h>
#include <EEPROM.h>

#define NPU 8
#define AHU 9
#define HUMI_OUTPUT 10
#define TEMP_OUTPUT 11

#define analogOutPin 3

#define PW1 7
#define PW2 6
#define VMS_ID 1
#define HMI_ID 2

#define E_TEMP_ADD 0
#define E_HUMI_ADD 2

#define P_ROOM_ADD 0
#define P_ACH_ADD 1
#define TEMP_ADD 2
#define HUMI_ADD 3
#define SET_TEMP_ADD 4
#define SET_HUMI_ADD 5
#define EDIT_TEMP_ADD 6
#define EDIT_HUMI_ADD 7
#define PRESSURE_SW1_ADD 0
#define PRESSURE_SW2_ADD 1
#define RUN_ANI_ADD 2
#define RUN_COIL_ADD 3
#define SET_COIL_ADD 4

ModbusMaster hmi;
ModbusMaster vms300;
const int numReadings = 10;  // Number of readings to average
int outputValue = 0;         // value output to the PWM (analog out)
int pressure1[numReadings];
int pressure2[numReadings];  // Array to store the readings
int index = 0;               // Index of the current reading
long pTotal1 = 0;            // Total sum of the readings
long pTotal2 = 0;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
  for (int i = 0; i < numReadings; i++) {
    pressure1[i] = 0;  // Initialize readings array
    pressure2[i] = 0;
  }
  pinMode(NPU, OUTPUT);
  pinMode(AHU, OUTPUT);
  pinMode(HUMI_OUTPUT, OUTPUT);
  pinMode(TEMP_OUTPUT, OUTPUT);
  pinMode(PW1, INPUT);
  pinMode(PW2, INPUT);
  //Serial.println(analogRead(A3));
  vms300.begin(VMS_ID, Serial);
  hmi.begin(HMI_ID, Serial);
  int setTemp = readEprom(E_TEMP_ADD);
  int setHumi = readEprom(E_HUMI_ADD);
  hmi.clearTransmitBuffer();
  hmi.setTransmitBuffer(0, setTemp);
  hmi.setTransmitBuffer(1, setHumi);
  hmi.writeMultipleRegisters(SET_TEMP_ADD, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t temp = 0, humi = 0;
  uint8_t result;
  //uint16_t mPressure1;
  boolean onCoil;
  //read pressure switch
  if (digitalRead(PW1) == 1) {
    hmi.writeSingleCoil(PRESSURE_SW1_ADD, 0);
  } else {
    hmi.writeSingleCoil(PRESSURE_SW1_ADD, 1);
  }
  delay(100);
  if (digitalRead(PW2) == 1) {
    hmi.writeSingleCoil(PRESSURE_SW2_ADD, 0);
  } else {
    hmi.writeSingleCoil(PRESSURE_SW2_ADD, 1);
  }
  delay(100);
  // read ADC
  int pTotal3 = 0;
  int pTotal4 = 0;

  pTotal1 -= pressure1[index];  // Subtract the oldest reading from the total
  pTotal2 -= pressure2[index];
  pressure1[index] = analogRead(A0);  // Read from ADC
  pressure2[index] = analogRead(A0);
  pTotal3 = analogRead(A2);  // Read from ADC
  pTotal4 = analogRead(A3);  // Read from ADC

  pTotal1 += pressure1[index];  // Add the newest reading to the total
  pTotal2 += pressure2[index];
  index = (index + 1) % numReadings;  // Increment index, wrap around if necessary
  // Calculate the average
  int pAverage1 = pTotal1 / numReadings;
  int pAverage2 = pTotal2 / numReadings;
  int pAverage3 = 0;
  int pAverage4 = 0;
  int pAverage5 = 0;
  int pAverage6 = 0;

  pAverage1 = map(pAverage1, 0, 1024, 0, 250);   //ISOLATE  Master
  pAverage2 = map(pAverage2, 0, 1024, 0, 250);   //ANTE     Master
 
  pAverage3 = map(pTotal3, 0, 1024, -50, 50);   //ISOLATE
  pAverage4 = map(pTotal4, 0, 1024, -50, 50);   //ANTE

  //pAverage1 = (pAverage1 - 50);
  //pAverage2 = (pAverage2 - 50);

  
  pAverage1 = (pAverage1 + pAverage3);
  pAverage2 = (pAverage2 + pAverage4);
  

  //Serial.println(pAverage1);
  //Serial.println(pAverage2);

  //read temp humi
  vms300.clearResponseBuffer();
  result = vms300.readHoldingRegisters(0, 2);
  if (result == vms300.ku8MBSuccess) {
    humi = vms300.getResponseBuffer(0);
    temp = vms300.getResponseBuffer(1);
  }
  delay(100);
  //read run coil
  hmi.clearResponseBuffer();
  result = hmi.readCoils(RUN_COIL_ADD, 1);
  if (result == hmi.ku8MBSuccess) {
    onCoil = hmi.getResponseBuffer(0);
  }
  delay(100);

  if (onCoil == 1) {
    hmi.writeSingleCoil(RUN_ANI_ADD, 1);
    digitalWrite(NPU, HIGH);
    digitalWrite(AHU, HIGH);
    int setTemp = readEprom(E_TEMP_ADD);
    int setHumi = readEprom(E_HUMI_ADD);
    
    outputValue = map(setTemp, 100, 300, 0, 255);
    
    analogWrite(analogOutPin, outputValue);

    if (temp >= (setTemp + 10)) digitalWrite(TEMP_OUTPUT, HIGH);
    if (temp <= (setTemp - 10)) digitalWrite(TEMP_OUTPUT, LOW);

    if (humi >= (setHumi + 10)) digitalWrite(HUMI_OUTPUT, HIGH);
    if (humi <= (setHumi - 10)) digitalWrite(HUMI_OUTPUT, LOW);

  } 
  
  else {
    hmi.writeSingleCoil(RUN_ANI_ADD, 0);
    digitalWrite(NPU, LOW);
    digitalWrite(AHU, LOW);
    digitalWrite(TEMP_OUTPUT, LOW);
    digitalWrite(HUMI_OUTPUT, LOW);

    temp = 0;
    humi = 0;
    pAverage1 = 0;
    pAverage2 = 0;
  }
 
  //set hmi value
  hmi.clearTransmitBuffer();
  hmi.setTransmitBuffer(0, pAverage1);
  hmi.setTransmitBuffer(1, pAverage2);
  hmi.setTransmitBuffer(2, humi);
  hmi.setTransmitBuffer(3, temp);
  hmi.writeMultipleRegisters(P_ROOM_ADD, 4);

  //read setting coil
  hmi.clearResponseBuffer();
  result = hmi.readCoils(SET_COIL_ADD, 1);
  if (result == hmi.ku8MBSuccess) {
    onCoil = hmi.getResponseBuffer(0);
  }
  delay(100);
  if (onCoil == 1) {
    hmi.clearResponseBuffer();
    result = hmi.readHoldingRegisters(EDIT_TEMP_ADD, 2);
    if (result == hmi.ku8MBSuccess) {
      temp = hmi.getResponseBuffer(0);
      humi = hmi.getResponseBuffer(1);
    }
    delay(100);
    hmi.clearTransmitBuffer();
    hmi.setTransmitBuffer(0, temp);
    hmi.setTransmitBuffer(1, humi);
    hmi.writeMultipleRegisters(SET_TEMP_ADD, 2);
    delay(100);
    hmi.writeSingleCoil(SET_COIL_ADD, 0);
    writeEprom(E_TEMP_ADD, temp);
    writeEprom(E_HUMI_ADD, humi);
  }
  delay(2000);
}

void writeEprom(int address, int value) {
  byte byte1 = highByte(value);
  byte byte2 = lowByte(value);

  // Write the two bytes to EEPROM
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}
int readEprom(int address) {
  byte byteRead1 = EEPROM.read(address);
  byte byteRead2 = EEPROM.read(address + 1);
  int valueRead = (byteRead1 << 8) | byteRead2;
  return valueRead;
}

//////////////////////////////////END/////////////////////////////
