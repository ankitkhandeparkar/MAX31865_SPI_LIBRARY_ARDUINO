/**************************************************************************************************
 * 
 *  Author : Ankit Khandeparkar
 *  
 *  About : This code is written for interfacing MAX31865 and Arduino
 *          using the SPI library.
 *          
 *  Note : SPI config : MODE1 is used (MODE3 works too)
 *                      Bit order is MSB first
 *                      Clock should be 1 Mhz
 * 
 *         RTD value : The value of RTD is 2 Bytes
 *         
 *         Read function : Take 3 parameters 
 *                         (Address, Byte_size_to_transfer, location_to_store_read_value)
 *                         
 *                          If the Byte size is 2 (only used for RTD value)then Array buffer 
 *                          is passed this Array is then converted to int16_t using Big
 *                          Endian. 
 * 
 **************************************************************************************************/

#include <stdint.h>
#include <SPI.h>
#include "MAX31865_SPI_BARE.h"

#define _CS 10
#define RREF      430.0                     // 430.0 for PT100 and 4300.0 for PT100
#define RNOMINAL  100.0                     // 100.0 for PT100, 1000.0 for PT1000

byte _buff[2]; // 2 Bytes Buffer

void setup() {
  
  Serial.begin(9600);
  
  MAX31865_init(MAX31865_3WIRE);

  delay(1000); 

}

void loop() {
  
  byte rtd = readRTD();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(temperature(RNOMINAL, RREF));

  Serial.println();
  delay(1000);
  
}

void MAX31865_init(max31865_numwires_t wires) {
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  pinMode(_CS, OUTPUT);
  digitalWrite(_CS, HIGH);
  
  setWires(wires);
  enableBias(0);
  autoConvert(0);
  clearFault();
  
}

void setWires(max31865_numwires_t wires) {
  byte _b;
  Read(MAX31865_CONFIG_REG, 1, &_b);
  _b |= MAX31865_CONFIG_3WIRE;
  Write(MAX31865_CONFIG_REG, _b); 
}

void enableBias(int b) {
  byte _b;
  Read(MAX31865_CONFIG_REG, 1, &_b);
  if(b == 1) {
    _b |= MAX31865_CONFIG_BIAS; // enable bias
  }
  else {
    _b |= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  Write(MAX31865_CONFIG_BIAS, _b);
}

void autoConvert(int b) {
  byte _b;
  Read(MAX31865_CONFIG_REG, 1, &_b);
  if(b == 1) {
    _b |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  }
  else {
    _b |= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
  Write(MAX31865_CONFIG_BIAS, _b);
}

void clearFault(void) {
  byte _b;
  Read(MAX31865_CONFIG_REG, 1, &_b);
  _b &= ~0x2C;
  _b |= MAX31865_CONFIG_FAULTSTAT;
  Write(MAX31865_CONFIG_BIAS, _b);
}

int16_t readRTD(void) {
  clearFault();
  enableBias(1);
  delay(50);
  
  byte _b;
  Read(MAX31865_CONFIG_REG, 1, &_b);
  _b |= MAX31865_CONFIG_1SHOT;
  Write(MAX31865_CONFIG_BIAS, _b);
  delay(100);

  int16_t rtd;
  Read(MAX31865_RTDMSB_REG, 2, _buff);

  rtd = be16_to_cpu_signed(_buff);
  
  enableBias(0); // 0xFF

  rtd >>= 1;

  return rtd;
}

int16_t be16_to_cpu_signed(const uint8_t data[2]) {
    uint32_t val = (((uint32_t)data[0]) << 8) | 
                   (((uint32_t)data[1]) << 0);
    return ((int32_t) val) - 0x10000u;
}

/*
int16_t le16_to_cpu_signed(const uint8_t data[2]) {
    uint32_t val = (((uint32_t)data[0]) << 0) | 
                   (((uint32_t)data[1]) << 8);
    return ((int32_t) val) - 0x10000u;
}
*/

float temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

void Write(byte __reg_address, byte __val) {
  digitalWrite(_CS, LOW);
  SPI.transfer(__reg_address);
  SPI.transfer(__val);
  digitalWrite(_CS, HIGH);
}

void Read(byte __reg_address, int num, byte _buff[]) {
  // Read: Most Sig Bit of Reg Address Set
  //char _address = 0x80 | __reg_address;
  // If Multi-Byte Read: Bit 6 Set
  //if(num > 1) {
  //  __reg_address = __reg_address | 0x40;
 // }

  digitalWrite(_CS, LOW);
  SPI.transfer(__reg_address);   // Transfer Starting Reg Address To Be Read
  for(int i=0; i<num; i++){
    _buff[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS, HIGH);
}
