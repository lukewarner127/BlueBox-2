#include <SPI.h> 

TaskHandle_t Task1;

const int CS = 21;
const int LED = 2;
const int INT1 = 33;
const int INT2 = 25;
const uint8_t READ = 0x80;

/////////////////////////////// Sensor's memory register addresses ////////////////////////////////
const uint8_t ISM330_ADDRESS = 0x6A;
const uint8_t ISM330_WHO_AM_I = 0x0F;           // default: 1101010 / 0x6A

const uint8_t ISM330_FUNC_CFG_ACCESS = 0x01;
const uint8_t FUNC_CFG_ACCESS = 0x00;
const uint8_t ISM330_INT1_CTRL = 0x0D;
const uint8_t ISM330_INT2_CTRL = 0x0E;
const uint8_t ISM330_DRDY_PULSE_CFG = 0x0B;

const uint8_t ISM330_OUT_TEMP_L = 0x20;         // TEMPERATURE REGISTERS

const uint8_t ISM330_CTRL1_XL = 0x10;           // ACCELEROMETER REGISTERS
const uint8_t CTRL1_XL = 0x30;                  // ODR: 52 Hz, +/-2 G
const uint8_t ISM330_OUTX_L_XL = 0x28;
const uint8_t ISM330_OUTY_L_XL = 0x2A;
const uint8_t ISM330_OUTZ_L_XL = 0x2C;

const uint8_t ISM330_CTRL2_G = 0x11;            // GYROSCOPE REGISTERS
const uint8_t CTRL2_G = 0x30;                   // ODR: 52 Hz, +/-250 dps
const uint8_t ISM330_OUTX_L_G = 0x22;
const uint8_t ISM330_OUTY_L_G = 0x24;
const uint8_t ISM330_OUTZ_L_G = 0x26;

const uint8_t ISM330_CTRL6_C = 0x00;
const uint8_t CTRL6_C = 0x10;                   // High-performance mode (low-performance: 0x10)

////////////////////////////////////////////// SETUP //////////////////////////////////////////////
void setup(){
  Serial.begin(115200);                         // initialise serial comms
  pinMode(CS, OUTPUT);                          // initialise chip-select pin
  SPI.begin();                                  // initialise SPI comms
  SPI.setClockDivider(SPI_CLOCK_DIV128);        // slow down clock for breadboarding
  SPI.setBitOrder(MSBFIRST);                    // SPI setting
  SPI.setDataMode(SPI_MODE3);                   // SPI Mode 3: clock idles high, data sampled on rising 
                                                // edge and shifted out on falling edge
  delay(50);
  
  pinMode(LED, OUTPUT);
  sensorSettings();

  xTaskCreatePinnedToCore(                      // dual core setup
    Task1code,                                  // function to implement the task
    "Task1",                                    // task name
    10000,                                      // stack size in words
    NULL,                                       // task input parameter
    0,                                          // task priority
    &Task1,                                     // task handle
    0);                                         // core where the task should run
}
 
void Task1code(void *parameter){                // code to run on core 0; default code runs on core 1
  for(;;){
    blink();                                    // code here
    }
  // vTaskDelete(Task1); to delete created task
}

////////////////////////////////////////////// LOOP ///////////////////////////////////////////////
void loop(){
  float tempReading = tempConvert(readRegister(ISM330_OUT_TEMP_L, 2));
  Serial.print("Temperature: ");
  Serial.print(tempReading);
  Serial.println(" C");
  
  float xAccelReading = accelConvert(readRegister(ISM330_OUTX_L_XL, 2));
  float yAccelReading = accelConvert(readRegister(ISM330_OUTY_L_XL, 2));
  float zAccelReading = accelConvert(readRegister(ISM330_OUTZ_L_XL, 2));
  Serial.print("Acceleration in X, Y, Z: (");
  Serial.print(xAccelReading);
  Serial.print(", ");
  Serial.print(yAccelReading);
  Serial.print(", ");
  Serial.print(zAccelReading);
  Serial.println(") g");
  
  float xGyroReading = rotaConvert(readRegister(ISM330_OUTX_L_G, 2));
  float yGyroReading = rotaConvert(readRegister(ISM330_OUTY_L_G, 2));
  float zGyroReading = rotaConvert(readRegister(ISM330_OUTZ_L_G, 2));
  Serial.print("Rotation on X, Y, Z: (");
  Serial.print(xGyroReading);
  Serial.print(", ");
  Serial.print(yGyroReading);
  Serial.print(", ");
  Serial.print(zGyroReading);
  Serial.println(") dps ");
  
  delay(5000);
}

/* converts and scales unsigned 16-bit word to floating point */
float tempConvert(uint16_t tempDigital){
  uint8_t temp_lsb = (tempDigital & 0x00FF);
  uint8_t temp_msb = ((tempDigital & 0xFF00) >> 8);
  //Serial.println(tempDigital);
  //Serial.println(tempDigital, HEX);
  float fraction = temp_lsb / 256.0;            // fraction part
  
  if (temp_msb >= 0x80){                        // whole part
    temp_msb -= 231;                            // convert unsigned to "signed" int
  }
  else{
    temp_msb += 25;
  }
  float whole = temp_msb;
  
  float temp_c = whole + fraction;
  return temp_c;
}

/* converts and scales unsigned 16-bit word to floating point */
float accelConvert(uint16_t accelDigital){                    
  float sens = 2.0 / 32768.0;                   // select sensitivity depending on measurement range
                                                // sens = full-scale selection / 32768.0
  float acceleration;

  if (accelDigital >= 0x8000){                  // convert unsigned to signed int
    accelDigital = ~accelDigital;
    accelDigital += 1;
    acceleration = accelDigital * sens;
  }
  else {
    acceleration = - (accelDigital * sens);
  }
  
  return acceleration;
}

/* converts and scales unsigned 16-bit word to floating point */
float rotaConvert(uint16_t rotaDigital){
  float sens = 8.75 / 1000;                     // select sensitivity depending on measurement range
  float rotation;

  if (rotaDigital >= 0x8000){                   // convert unsigned to signed int
    rotaDigital = ~rotaDigital;
    rotaDigital += 1;
    rotation = rotaDigital * sens;
  }
  else {
    rotation = - (rotaDigital * sens);
  }
  
  return rotation;
}

/* read from the IMS330 */
uint16_t readRegister(uint8_t thisRegister, int bytesToRead) {
  uint8_t inByte;                               // incoming byte from the SPI
  uint16_t result;                              // result to return
  uint8_t dataToSend = thisRegister | READ;     // combine the address and the READ command into one byte
  digitalWrite(CS, LOW);                        // take the chip select line LOW to enable the device
  SPI.transfer(dataToSend);                     // send the device register you want to read
  result = SPI.transfer(0x00);                  // send a value of 0 to read the first byte returned
  bytesToRead--;                                // decrement the number of bytes still left to read
  if (bytesToRead > 0) {                        // for bytes still left to be read
    inByte = SPI.transfer(0x00);
    result = result | (inByte << 8);            // shift the second byte and combine with the first byte
    bytesToRead--;                              // decrement the number of bytes still left to read
  }
  digitalWrite(CS, HIGH);                       // take the chip select line HIGH to disable the device
  
  return result;                                // return the result
}

/* write to the IMS330 */
void writeRegister(uint8_t thisRegister, uint8_t thisValue) {
  digitalWrite(CS, LOW);                        // start comms
  SPI.transfer(thisRegister);                   // send register location
  SPI.transfer(thisValue);                      // send value to that register
  digitalWrite(CS, HIGH);                       // end comms
}

/* change settings */
void sensorSettings(){
  digitalWrite(CS, LOW);                        // start comms
  writeRegister(ISM330_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS); // select register to change, and byte to change to
  writeRegister(ISM330_CTRL1_XL, CTRL1_XL);
  writeRegister(ISM330_CTRL2_G, CTRL2_G);
  digitalWrite(CS, HIGH);                       // end comms
  Serial.println("Sensor changes applied");     // confirmation message
}

void blink(){
  digitalWrite(LED, HIGH);
  delay(1);
  digitalWrite(LED, LOW);
  delay(3);
  digitalWrite(LED, HIGH);
  delay(1);
  digitalWrite(LED, LOW);
  delay(3);
  digitalWrite(LED, HIGH);
  delay(1);
  digitalWrite(LED, LOW);
  delay(3);
  digitalWrite(LED, HIGH);
  delay(1);
  digitalWrite(LED, LOW);
  delay(4987);                                  // blink once every 5 seconds
}
