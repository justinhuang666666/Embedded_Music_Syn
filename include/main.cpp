#include <Arduino.h>
#include <U8g2lib.h>
#include <stdio.h>
#include <iostream>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

const uint32_t sampleRate = 22000;
const double semitoneFactor = std::pow(2, 1.0/12);

constexpr std::array<uint32_t, 12> calc_stepSizes() {
    std::array<uint32_t, 12> stepSizes {};

    double frequencies[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    frequencies[9] = 440.0;  // A4

    for (int i = 10; i < 12; i++) {
        frequencies[i] = frequencies[i-1] * semitoneFactor;
    }
    for (int i = 8; i >= 0; i--) {
        frequencies[i] = frequencies[i+1] / semitoneFactor;
    }
    for (int i = 0; i < 12; i++) {
        double frequency = frequencies[i];
        stepSizes[i] = static_cast<uint32_t>((1ULL << 32) * frequency / sampleRate);
    }
    return stepSizes;
}

const std::array<uint32_t, 12> stepSizes = calc_stepSizes();
const std::array<const char*,12> keyNote = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

volatile uint32_t currentStepSize;

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Setup timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols() {
  // set row select address low and row select enable high to read Row 0
  // read the inputs from the columns and concatenate them into a single byte
  uint8_t col0 = digitalRead(C0_PIN) ? 1 : 0;
  uint8_t col1 = digitalRead(C1_PIN) ? 2 : 0;
  uint8_t col2 = digitalRead(C2_PIN) ? 4 : 0;
  uint8_t col3 = digitalRead(C3_PIN) ? 8 : 0;

  return col0 | col1 | col2 | col3;
}

const char* mapKeys(uint32_t keys){
  const char* localkeyNote = "0";
  for(int i = 0; i < 12; i++){
    localkeyNote = ((keys&0x1<<i)>0)? localkeyNote : keyNote[i];
  } 
  return localkeyNote;
}

uint32_t mapStepsize(uint32_t keys){
  uint32_t localStepSize = 0;
  for(int i = 0; i < 12; i++){
    localStepSize = ((keys&0x1<<i)>0)? localStepSize : stepSizes[i];
  } 
  return localStepSize;
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory

    uint8_t keyArray[7];
    for(int i = 0; i < 3; i++){
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }

    u8g2.setCursor(2,20);
    uint32_t keys = keyArray[2]<<8 | keyArray[1]<<4 | keyArray[0];
    u8g2.print(keys,BIN);
    //u8g2.setCursor(2,30);
    //u8g2.print(currentStepSize,HEX);
    const char* k=mapKeys(keys);

    uint32_t localCurrentStepSize=mapStepsize(keys);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    //u8g2.drawStr(2,30,k);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}