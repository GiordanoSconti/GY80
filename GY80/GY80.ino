#include <Wire.h>

#define ADXL345_DEVID_REG 0x00 //0, RESETVALUE: 11100101, R, Device ID
#define ADXL345_THRESH_TAP_REG 0x1D //29, RESETVALUE: 00000000, R/W(basso), Tap threshold
#define ADXL345_OFSX_REG 0x1E //30, RESETVALUE: 00000000, R/W(basso), X-axis offset
#define ADXL345_OFSY_REG 0x1F //31, RESETVALUE: 00000000, R/W(basso), Y-axis offset
#define ADXL345_OFSZ_REG 0x20 //32, RESETVALUE: 00000000, R/W(basso), Z-axis offset
#define ADXL345_DUR_REG 0x21 //33, RESETVALUE: 00000000, R/W(basso), Tap duration
#define ADXL345_LATENT_REG 0x22 //34, RESETVALUE: 00000000, R/W(basso), Tap latency
#define ADXL345_WINDOW_REG 0x23 //35, RESETVALUE: 00000000, R/W(basso), Tap window
#define ADXL345_THRESH_ACT_REG 0x24 //36, RESETVALUE: 00000000, R/W(basso), Activity threshold
#define ADXL345_THRESH_INACT_REG 0x25 //37, RESETVALUE: 00000000, R/W(basso), Inactivity threshold
#define ADXL345_TIME_INACT_REG 0x26 //38, RESETVALUE: 00000000, R/W(basso), Inactivity time
#define ADXL345_ACT_INACT_CTL_REG 0x27 //39, RESETVALUE: 00000000, R/W(basso), Axis enable control for activity and inactivity detection
#define ADXL345_THRESH_FF_REG 0x28 //40, RESETVALUE: 00000000, R/W(basso), Free-fall threshold
#define ADXL345_TIME_FF_REG 0x29 //41, RESETVALUE: 00000000, R/W(basso), Free-fall time
#define ADXL345_TAP_AXES_REG 0x2A //42, RESETVALUE: 00000000, R/W(basso), Axis control for single tap/double tap
#define ADXL345_ACT_TAP_STATUS_REG 0x2B //43, RESETVALUE: 00000000, R, Source of single tap/double tap
#define ADXL345_BW_RATE_REG 0x2C //44, RESETVALUE: 00001010, R/W(basso), Data rate and power mode control
#define ADXL345_POWER_CTRL_REG 0x2D //45, RESETVALUE: 00000000, R/W(basso), Power-saving features control
#define ADXL345_INT_ENABLE_REG 0x2E //46, RESETVALUE: 00000000, R/W(basso), Interrupt enable control
#define ADXL345_INT_MAP_REG 0x2F //47, RESETVALUE: 00000000, R/W(basso), Interrupt mapping control
#define ADXL345_INT_SOURCE_REG 0x30 //48, RESETVALUE: 00000010, R, Source of interrupts
#define ADXL345_DATA_FORMAT_REG 0x31 //49, RESETVALUE: 00000000, R/W(basso), Data format control
#define ADXL345_DATAX_LOW_REG 0x32 //50, RESETVALUE: 00000000, R, X-Axis Data 0
#define ADXL345_DATAX_HIGH_REG 0x33 //51, RESETVALUE: 00000000, R, X-Axis Data 1
#define ADXL345_DATAY_LOW_REG 0x34 //52, RESETVALUE: 00000000, R, Y-Axis Data 0
#define ADXL345_DATAY_HIGH_REG 0x35 //53, RESETVALUE: 00000000, R, Y-Axis Data 1
#define ADXL345_DATAZ_LOW_REG 0x36 //54, RESETVALUE: 00000000, R, Z-Axis Data 0
#define ADXL345_DATAZ_HIGH_REG 0x37 //55, RESETVALUE: 00000000, R, Z-Axis Data 1
#define ADXL345_FIFO_CTL_REG 0x38 //56, RESETVALUE: 00000000, R/W(basso), FIFO control
#define ADXL345_FIFO_STATUS_REG 0x39 //57, RESETVALUE: 00000000, R, FIFO status
#define ADXL345_I2C_ADDR 83 //I2C address of the ADXL345 (HEX: 0x53; DEC: 83)

#define L3G4200D_WHO_AM_I_REG 0x0F //15, RESETVALUE: 11010011, R, Device Identification Register
#define L3G4200D_CTRL_REG1_REG 0x20 //32, RESETVALUE: 00000111, RW, Device Control Register 1 (Enable/Disable Axes and Change Power Mode)
#define L3G4200D_CTRL_REG2_REG 0x21 //33, RESETVALUE: 00000000, RW, Device Control Register 2 (Adjust/Use the High-Pass Filtering HPF)
#define L3G4200D_CTRL_REG3_REG 0x22 //34, RESETVALUE: 00000000, RW, Device Control Register 3 (Enable/Disable Interrupts)
#define L3G4200D_CTRL_REG4_REG 0x23 //35, RESETVALUE: 00000000, RW, Device Control Register 4 (Control the Full-Scale Range)
#define L3G4200D_CTRL_REG5_REG 0x24 //36, RESETVALUE: 00000000, RW, Device Control Register 5 (Control high-pass filtering of outputs)
#define L3G4200D_REFERENCE_REG 0x25 //37, RESETVALUE: 00000000, RW, Reference value for Interrupt Generation
#define L3G4200D_OUT_TEMP_REG 0x26 //38, RESETVALUE: output, R, Temperature Data
#define L3G4200D_STATUS_REG_REG 0x27 //39, RESETVALUE: output, R, Status X, Y, Z Axes Register
#define L3G4200D_OUT_X_L_REG 0x28 //40, RESETVALUE: output, R, X Axis Angular Rate Data (Low Byte)
#define L3G4200D_OUT_X_H_REG 0x29 //41, RESETVALUE: output, R, X Axis Angular Rate Data (High Byte)
#define L3G4200D_OUT_Y_L_REG 0x2A //42, RESETVALUE: output, R, Y Axis Angular Rate Data (Low Byte)
#define L3G4200D_OUT_Y_H_REG 0x2B //43, RESETVALUE: output, R, Y Axis Angular Rate Data (High Byte)
#define L3G4200D_OUT_Z_L_REG 0x2C //44, RESETVALUE: output, R, Z Axis Angular Rate Data (Low Byte)
#define L3G4200D_OUT_Z_H_REG 0x2D //45, RESETVALUE: output, R, Z Axis Angular Rate Data (High Byte)
#define L3G4200D_FIFO_CTRL_REG_REG 0x2E //46, RESETVALUE: 00000000, RW, FIFO Control Register
#define L3G4200D_FIFO_SRC_REG_REG 0x2F //47, RESETVALUE: output, R, FIFO Status Register
#define L3G4200D_INT1_CFG_REG 0x30 //48, RESETVALUE: 00000000, RW, Interrupt Control Register
#define L3G4200D_INT1_SRC_REG 0x31 //49, RESETVALUE: output, R, Interrupt Status Register
#define L3G4200D_INT1_TSH_XH_REG 0x32 //50, RESETVALUE: 00000000, RW, Interrupt Threshold X High
#define L3G4200D_INT1_TSH_XL_REG 0x33 //51, RESETVALUE: 00000000, RW, Interrupt Threshold X Low
#define L3G4200D_INT1_TSH_YH_REG 0x34 //52, RESETVALUE: 00000000, RW, Interrupt Threshold Y High
#define L3G4200D_INT1_TSH_YL_REG 0x35 //53, RESETVALUE: 00000000, RW, Interrupt Threshold Y Low
#define L3G4200D_INT1_TSH_ZH_REG 0x36 //54, RESETVALUE: 00000000, RW, Interrupt Threshold Z High
#define L3G4200D_INT1_TSH_ZL_REG 0x37 //55, RESETVALUE: 00000000, RW, Interrupt Threshold Z Low
#define L3G4200D_INT1_DURATION_REG 0x38 //56, RESETVALUE: 00000000, RW, Interrupt Duration Value
#define L3G4200D_I2C_ADDR 105//I2C address of the L3G4200D (HEX: 0x69; DEC: 105)

#define HMC5883L_CRA_REG 0x00 //0, RESETVALUE: 00010000, Read/Write, Configuration Register A
#define HMC5883L_CRB_REG 0x01 //1, RESETVALUE: 00100000, Read/Write, Configuration Register B
#define HMC5883L_MR_REG 0x02 //2, RESETVALUE: 00000001, Read/Write, Mode Register
#define HMC5883L_DXRA_REG 0x03 //3, RESETVALUE: 00000000, Read, Data Output X MSB Register
#define HMC5883L_DXRB_REG 0x04 //4, RESETVALUE: 00000000, Read, Data Output X LSB Register
#define HMC5883L_DYRA_REG 0x05 //5, RESETVALUE: 00000000, Read, Data Output Y MSB Register
#define HMC5883L_DYRB_REG 0x06 //6, RESETVALUE: 00000000, Read, Data Output Y LSB Register
#define HMC5883L_DZRA_REG 0x07 //7, RESETVALUE: 00000000, Read, Data Output Z MSB Register
#define HMC5883L_DZRB_REG 0x08 //8, RESETVALUE: 00000000, Read, Data Output Z LSB Register
#define HMC5883L_SR_REG 0x09 //9, RESETVALUE: 00000000, Read, Status Register
#define HMC5883L_IRA_REG 0x0A //10, RESETVALUE: 01001000, Read, Identification Register A
#define HMC5883L_IRB_REG 0x0B //11, RESETVALUE: 00110100, Read, Identification Register B
#define HMC5883L_IRC_REG 0x0C //12, RESETVALUE: 00110011, Read, Identification Register C
#define HMC5883L_I2C_ADDR 30//I2C address of the HMC5883L (HEX: 0x1E; DEC: 30)

//Accelerometer variables:
int ADXL345XValue; // X Axis
int ADXL345YValue; // Y Axis
int ADXL345ZValue; // Z Axis

//Gyroscope variables:
int L3G4200DXValue; // X Axis
int L3G4200DYValue; // Y Axis
int L3G4200DZValue; // Z Axis
int L3G4200DTemperatureValue; // Temperature

//Magnetometer variables:
int HMC5883LXValue; // X Axis
int HMC5883LYValue; // Y Axis
int HMC5883LZValue; // Z Axis

void startADXL345()
{
  writeRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTRL_REG, 0b00000000);
  writeRegister(ADXL345_I2C_ADDR, ADXL345_POWER_CTRL_REG, 0b00011001);
  writeRegister(ADXL345_I2C_ADDR, ADXL345_DATA_FORMAT_REG, 0b00001000);
  writeRegister(ADXL345_I2C_ADDR, ADXL345_BW_RATE_REG, 0b00001001);
}

void startL3G4200D(int scale)
{
  writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG1_REG, 0b00001111);
  writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG2_REG, 0b00000000);
  writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG3_REG, 0b00000000);
  if(scale == 250){
    writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG4_REG, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG4_REG, 0b00010000);
  }else{
    writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG4_REG, 0b00110000);
  }
  writeRegister(L3G4200D_I2C_ADDR, L3G4200D_CTRL_REG5_REG, 0b00000000);
}

void startHMC5883L()
{
  writeRegister(HMC5883L_I2C_ADDR, HMC5883L_CRA_REG, 0b00010000);
  writeRegister(HMC5883L_I2C_ADDR, HMC5883L_CRB_REG, 0b00100000);
  writeRegister(HMC5883L_I2C_ADDR, HMC5883L_MR_REG, 0b00000001);
}

void writeRegister(int deviceAddress, int registerAddress, int value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

int writeRegister(int deviceAddress, int registerAddress, byte *buffer)
{
	int writtenBytes = 0;
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	writtenBytes = Wire.write(buffer, sizeof(buffer));
	Wire.endTransmission();
	return writtenBytes;
}

byte readRegister(int deviceAddress, int registerAddress){
  byte value = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  while(!Wire.available()){}
  value = Wire.read();
  return value;
}

void readRegister(int deviceAddress, int registerAddress, byte *buffer)
{
  if(sizeof(buffer) == 6)
  {
    Wire.beginTransmission(deviceAddress);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 6);
    while(Wire.available() < 6){}
    byte xLSB = Wire.read();
    byte xMSB = Wire.read();
    byte yLSB = Wire.read();
    byte yMSB = Wire.read();
    byte zLSB = Wire.read();
    byte zMSB = Wire.read();
    *(buffer + sizeof(byte) * 0) = xLSB;
    *(buffer + sizeof(byte) * 1) = xMSB;
    *(buffer + sizeof(byte) * 2) = yLSB;
    *(buffer + sizeof(byte) * 3) = yMSB;
    *(buffer + sizeof(byte) * 4) = zLSB;
    *(buffer + sizeof(byte) * 5) = zMSB;
  }
}

void getADXL345Values()
{
  byte xMSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAX_HIGH_REG);
  byte xLSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAX_LOW_REG);
  ADXL345XValue = ((xMSB << 8) | xLSB);
  byte yMSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAY_HIGH_REG);
  byte yLSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAY_LOW_REG);
  ADXL345YValue = ((yMSB << 8) | yLSB);
  byte zMSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAZ_HIGH_REG);
  byte zLSB = readRegister(ADXL345_I2C_ADDR, ADXL345_DATAZ_LOW_REG);
  ADXL345ZValue = ((zMSB << 8) | zLSB);
}

void getL3G4200DValues()
{
  byte xMSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_X_H_REG);
  byte xLSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_X_L_REG);
  L3G4200DXValue = ((xMSB << 8) | xLSB);
  byte yMSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_Y_H_REG);
  byte yLSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_Y_L_REG);
  L3G4200DYValue = ((yMSB <<  8) | yLSB);
  byte zMSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_Z_H_REG);
  byte zLSB = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_Z_L_REG);
  L3G4200DZValue = ((zMSB << 8) | zLSB);
  L3G4200DTemperatureValue = readRegister(L3G4200D_I2C_ADDR, L3G4200D_OUT_TEMP_REG);
}

void getHMC5883LValues()
{
  byte xMSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DXRA_REG);
  byte xLSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DXRB_REG);
  HMC5883LXValue = ((xMSB << 8) | xLSB);
  byte yMSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DYRA_REG);
  byte yLSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DYRB_REG);
  HMC5883LYValue = ((yMSB << 8) | yLSB);
  byte zMSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DZRA_REG);
  byte zLSB = readRegister(HMC5883L_I2C_ADDR, HMC5883L_DZRB_REG);
  HMC5883LZValue = ((zMSB << 8) | zLSB);
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  startADXL345();
  startL3G4200D(250);
  startHMC5883L();
  delay(1500);
}

void loop()
{
  getADXL345Values();
  getL3G4200DValues();
  getHMC5883LValues();
  Serial.println("Accelerometer Values: ");
  Serial.print("X: ");
  Serial.println(ADXL345XValue);
  Serial.print("Y: ");
  Serial.println(ADXL345YValue);
  Serial.print("Z: ");
  Serial.println(ADXL345ZValue);
  Serial.println("Gyroscope Values: ");
  Serial.print("X: ");
  Serial.println(L3G4200DXValue);
  Serial.print("Y: ");
  Serial.println(L3G4200DYValue);
  Serial.print("Z: ");
  Serial.println(L3G4200DZValue);
  Serial.print("Temperature: ");
  Serial.println(L3G4200DTemperatureValue);
  Serial.println("Magnetometer Values: ");
  Serial.print("X: ");
  Serial.println(HMC5883LXValue);
  Serial.print("Y: ");
  Serial.println(HMC5883LYValue);
  Serial.print("Z: ");
  Serial.println(HMC5883LZValue);
  Serial.println();
  startHMC5883L();
  delay(1000);
}
