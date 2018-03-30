/*
   The following code is an average measurement approach to decay the error,
   code was tested for 4 sensors, Range measurement from 18-200mm to be exact.
   below 18mm the result may not be accurate & above 200mm will return system
   error. ALS is working perfectly now.

   Setup:

   1. VL6180X, Arduino
   ====================
   VL6180X      Arduino
   -----------------
   SCL          SCL
   SDA          SDA
   GPI/O        Dedicated Pin according to the Sketch (Active High)
   AVDD         5V
   GND          GND
   RGB LED      11,12,13,GND
   ====================

   2. User defined Range could be given through the variables from the User
   Input section, by default it was set to 100mm.
   3. User must declare each measurement should include how many internal
   measurements from 'User Input Section' through the
   'numberOfMeasurementPerCount' variable.
   4. RGB at Pin 11,12,13 accordingly.
   5. Each sensor GPIO to ArduinoPin must set from '4-Sensor Configuration'
   Menu.

   Arduino source code by Rahman.
   Original library taken from Pololu and Modified by Rahman.
*/

#include <Wire.h>
#include <Arduino.h>

/////////////////User Input Section////////////////////
#define givenRange 190         // mm
#define givenAmbientRange 1400 // lux
#define measurementInterval 6  // ms
#define numberOfMeasurementPerCount                                            \
  5 // average value consists of how many measurements? "35" is the best with
    // regards time consumption
#define stimeout                                                               \
  500 // ms     'stimeout' must be bigger than 'measurementInterval'
//////////////////////////////////////////////////////

/////////////////////Extended View////////////////////
//#define Analyze         //to enable detailed Raw Data
//#define SHOWADDRESS   //to show the sensors addresses
//#define Ambient  //Enable Ambient Measurement
//#define RGB       //to enable RGB PIN in 11,12,13
//#define showAllMeasurement
//////////////////////////////////////////////////////

//////////////4-Sensor Configuration//////////////////
#define Pin1 7 // Sensor 1 GPIO
#define Pin2 8 // Sensor 2 GPIO
#define Pin3 6 // Sensor 3 GPIO
#define Pin4 5 // Sensor 4 GPIO
//////////////////////////////////////////////////////

// Class Begin
#ifndef DropSensor_h
#define DropSensor_h

class DropSensor
{
public:
  // register addresses
  enum regAddr
  {
    IDENTIFICATION__MODEL_ID = 0x000,
    IDENTIFICATION__MODEL_REV_MAJOR = 0x001,
    IDENTIFICATION__MODEL_REV_MINOR = 0x002,
    IDENTIFICATION__MODULE_REV_MAJOR = 0x003,
    IDENTIFICATION__MODULE_REV_MINOR = 0x004,
    IDENTIFICATION__DATE_HI = 0x006,
    IDENTIFICATION__DATE_LO = 0x007,
    IDENTIFICATION__TIME = 0x008, // 16-bit

    SYSTEM__MODE_GPIO0 = 0x010,
    SYSTEM__MODE_GPIO1 = 0x011,
    SYSTEM__HISTORY_CTRL = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014,
    SYSTEM__INTERRUPT_CLEAR = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD = 0x017,

    SYSRANGE__START = 0x018,
    SYSRANGE__THRESH_HIGH = 0x019,
    SYSRANGE__THRESH_LOW = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022, // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD = 0x026, // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES = 0x02D,
    SYSRANGE__VHV_RECALIBRATE = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE = 0x031,

    SYSALS__START = 0x038,
    SYSALS__THRESH_HIGH = 0x03A,
    SYSALS__THRESH_LOW = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD = 0x03E,
    SYSALS__ANALOGUE_GAIN = 0x03F,
    SYSALS__INTEGRATION_PERIOD = 0x040,

    RESULT__RANGE_STATUS = 0x04D,
    RESULT__ALS_STATUS = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO = 0x04F,
    RESULT__ALS_VAL = 0x050,          // 16-bit
    RESULT__HISTORY_BUFFER_0 = 0x052, // 16-bit
    RESULT__HISTORY_BUFFER_1 = 0x054, // 16-bit
    RESULT__HISTORY_BUFFER_2 = 0x056, // 16-bit
    RESULT__HISTORY_BUFFER_3 = 0x058, // 16-bit
    RESULT__HISTORY_BUFFER_4 = 0x05A, // 16-bit
    RESULT__HISTORY_BUFFER_5 = 0x05C, // 16-bit
    RESULT__HISTORY_BUFFER_6 = 0x05E, // 16-bit
    RESULT__HISTORY_BUFFER_7 = 0x060, // 16-bit
    RESULT__RANGE_VAL = 0x062,
    RESULT__RANGE_RAW = 0x064,
    RESULT__RANGE_RETURN_RATE = 0x066,            // 16-bit
    RESULT__RANGE_REFERENCE_RATE = 0x068,         // 16-bit
    RESULT__RANGE_RETURN_SIGNAL_COUNT = 0x06C,    // 32-bit
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070, // 32-bit
    RESULT__RANGE_RETURN_AMB_COUNT = 0x074,       // 32-bit
    RESULT__RANGE_REFERENCE_AMB_COUNT = 0x078,    // 32-bit
    RESULT__RANGE_RETURN_CONV_TIME = 0x07C,       // 32-bit
    RESULT__RANGE_REFERENCE_CONV_TIME = 0x080,    // 32-bit

    RANGE_SCALER = 0x096, // 16-bit - see STSW-IMG003 core/inc/DropSensor_def.h

    READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A,
    FIRMWARE__BOOTUP = 0x119,
    FIRMWARE__RESULT_SCALER = 0x120,
    I2C_SLAVE__DEVICE_ADDRESS = 0x212,
    INTERLEAVED_MODE__ENABLE = 0x2A3,
  };

  uint8_t last_status; // status of last I2C transmission

  DropSensor(void);

  void setAddress(uint8_t new_addr);

  void init(void);

  void configureDefault(void);

  void writeReg(uint16_t reg, uint8_t value);
  void writeReg16Bit(uint16_t reg, uint16_t value);
  void writeReg32Bit(uint16_t reg, uint32_t value);
  uint8_t readReg(uint16_t reg);
  uint16_t readReg16Bit(uint16_t reg);
  uint32_t readReg32Bit(uint16_t reg);
  void printError(DropSensor sens);
  void setScaling(uint8_t new_scaling);

  uint8_t readRangeSingle(void);
  inline uint16_t readRangeSingleMillimeters(void) { return readRangeSingle(); }
  uint16_t readAmbientSingle(void);

  void startRangeContinuous(uint16_t period = 100);
  void startAmbientContinuous(uint16_t period = 500);
  void startInterleavedContinuous(uint16_t period = 500);
  void stopContinuous();

  uint8_t readRangeContinuous(void);
  inline uint16_t readRangeContinuousMillimeters(void)
  {
    return readRangeContinuous();
  }
  uint16_t readAmbientContinuous(void);

  inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
  inline uint16_t getTimeout(void) { return io_timeout; }
  bool timeoutOccurred(void);

private:
  uint8_t address;
  uint8_t ptp_offset;
  uint16_t io_timeout;
  bool did_timeout;
};

#endif
// Class Ends here

// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

// RANGE_SCALER values for 1x, 2x, 3x scaling - see STSW-IMG003
// core/src/DropSensor_api.c (ScalerLookUP[])
// static uint16_t const ScalerValues[] = { 0, 253, 127, 84 };

// Constructors ////////////////////////////////////////////////////////////////

DropSensor::DropSensor(void)
  : address(ADDRESS_DEFAULT) // C++11; Member variable initialization*
  , ptp_offset(0) // A different style, prominent if a class having multiple
                  // constructors.
  , io_timeout(0) // no timeout    //Which gives a chance to override values
                  // when needed.
  , did_timeout(false)
{
  /* Normal way
    address = ADDRESS_DEFAULT;
    ptp_offset = 0;
    io_timeout = 0;
    did_timeout = false;
  */
}

// Public Methods //////////////////////////////////////////////////////////////

void
DropSensor::setAddress(uint8_t new_addr)
{
  writeReg(I2C_SLAVE__DEVICE_ADDRESS,
           new_addr & 0x7F); // writing new address to the register
                             // 'I2C_SLAVE__DEVICE_ADDRESS = 0x212'*
  address = new_addr;
}

// Initialize sensor with settings from ST application note AN4545, section 9 -
// "Mandatory : private registers"
void
DropSensor::init()
{
  // Store part-to-part range offset so it can be adjusted if scaling is changed
  ptp_offset = readReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET);

  if (readReg(SYSTEM__FRESH_OUT_OF_RESET) == 1) {
    // scaling = 1;

    writeReg(0x207, 0x01);
    writeReg(0x208, 0x01);
    writeReg(0x096, 0x00);
    writeReg(0x097, 0xFD); // RANGE_SCALER = 253
    writeReg(0x0E3, 0x00);
    writeReg(0x0E4, 0x04);
    writeReg(0x0E5, 0x02);
    writeReg(0x0E6, 0x01);
    writeReg(0x0E7, 0x03);
    writeReg(0x0F5, 0x02);
    writeReg(0x0D9, 0x05);
    writeReg(0x0DB, 0xCE);
    writeReg(0x0DC, 0x03);
    writeReg(0x0DD, 0xF8);
    writeReg(0x09F, 0x00);
    writeReg(0x0A3, 0x3C);
    writeReg(0x0B7, 0x00);
    writeReg(0x0BB, 0x3C);
    writeReg(0x0B2, 0x09);
    writeReg(0x0CA, 0x09);
    writeReg(0x198, 0x01);
    writeReg(0x1B0, 0x17);
    writeReg(0x1AD, 0x00);
    writeReg(0x0FF, 0x05);
    writeReg(0x100, 0x05);
    writeReg(0x199, 0x05);
    writeReg(0x1A6, 0x1B);
    writeReg(0x1AC, 0x3E);
    writeReg(0x1A7, 0x1F);
    writeReg(0x030, 0x00);

    writeReg(SYSTEM__FRESH_OUT_OF_RESET, 0);
  } else {
    // Sensor has already been initialized, so try to get scaling settings by
    // reading registers.
    ptp_offset *= 1;
  }
}

// Configure some settings for the sensor's default behavior from AN4545 -
// "Recommended : Public registers" and "Optional: Public registers"
//
// Note that this function does not set up GPIO1 as an interrupt output as
// suggested, though you can do so by calling:
// writeReg(SYSTEM__MODE_GPIO1, 0x10);
void
DropSensor::configureDefault(void)
{
  // "Recommended : Public registers"

  // readout__averaging_sample_period = 48
  writeReg(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);

  // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01
  // according to Table 14 in datasheet)
  writeReg(SYSALS__ANALOGUE_GAIN, 0x46);

  // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature
  // recalibration after every 255 range measurements)
  writeReg(SYSRANGE__VHV_REPEAT_RATE, 0xFF);

  // sysals__integration_period = 99 (100 ms)
  // AN4545 incorrectly recommends writing to register 0x040; 0x63 should go in
  // the lower byte, which is register 0x041.
  writeReg16Bit(SYSALS__INTEGRATION_PERIOD, 0x0063);

  // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
  writeReg(SYSRANGE__VHV_RECALIBRATE, 0x01);

  // "Optional: Public registers"

  // sysrange__intermeasurement_period = 9 (100 ms)
  writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);

  // sysals__intermeasurement_period = 49 (500 ms)
  writeReg(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);

  // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4
  // (range new sample ready interrupt)
  writeReg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);

  // Reset other settings to power-on defaults

  // sysrange__max_convergence_time = 49 (49 ms)
  writeReg(DropSensor::SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);

  // disable interleaved mode
  writeReg(INTERLEAVED_MODE__ENABLE, 0);
}

// Writes an 8-bit register
void
DropSensor::writeReg(uint16_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff); // reg high byte
  Wire.write(reg & 0xff);        // reg low byte
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Writes a 16-bit register
void
DropSensor::writeReg16Bit(uint16_t reg, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff);   // reg high byte
  Wire.write(reg & 0xff);          // reg low byte
  Wire.write((value >> 8) & 0xff); // value high byte
  Wire.write(value & 0xff);        // value low byte
  last_status = Wire.endTransmission();
}

// Writes a 32-bit register
void
DropSensor::writeReg32Bit(uint16_t reg, uint32_t value)
{
  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff);    // reg high byte
  Wire.write(reg & 0xff);           // reg low byte
  Wire.write((value >> 24) & 0xff); // value highest byte
  Wire.write((value >> 16) & 0xff);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff); // value lowest byte
  last_status = Wire.endTransmission();
}

// Reads an 8-bit register
uint8_t
DropSensor::readReg(uint16_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff); // reg high byte
  Wire.write(reg & 0xff);        // reg low byte
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads a 16-bit register
uint16_t
DropSensor::readReg16Bit(uint16_t reg)
{
  uint16_t value;

  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff); // reg high byte
  Wire.write(reg & 0xff);        // reg low byte
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  value = (uint16_t)Wire.read() << 8; // value high byte
  value |= Wire.read();               // value low byte
  Wire.endTransmission();

  return value;
}

// Reads a 32-bit register
uint32_t
DropSensor::readReg32Bit(uint16_t reg)
{
  uint32_t value;

  Wire.beginTransmission(address);
  Wire.write((reg >> 8) & 0xff); // reg high byte
  Wire.write(reg & 0xff);        // reg low byte
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)4);
  value = (uint32_t)Wire.read() << 24; // value highest byte
  value |= (uint32_t)Wire.read() << 16;
  value |= (uint16_t)Wire.read() << 8;
  value |= Wire.read(); // value lowest byte
  Wire.endTransmission();

  return value;
}

// Performs a single-shot ranging measurement
uint8_t
DropSensor::readRangeSingle()
{
  writeReg(SYSRANGE__START, 0x01);
  return readRangeContinuous();
}

// Performs a single-shot ambient light measurement
uint16_t
DropSensor::readAmbientSingle()
{
  writeReg(SYSALS__START, 0x01);
  return readAmbientContinuous();
}

// readRangeSingle() also calls this function after starting a single-shot
// range measurement
uint8_t
DropSensor::readRangeContinuous()
{
  uint16_t millis_start = millis();
  while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x04) == 0) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout) {
      did_timeout = true;
      return 255;
    }
  }

  uint8_t range = readReg(RESULT__RANGE_VAL);
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);

  return range;
}

// readAmbientSingle() also calls this function after starting a single-shot
// ambient light measurement
uint16_t
DropSensor::readAmbientContinuous()
{
  uint16_t millis_start = millis();
  while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x20) == 0) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout) {
      did_timeout = true;
      return 0;
    }
  }

  uint16_t ambient = readReg16Bit(RESULT__ALS_VAL);
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x02);

  return ambient;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool
DropSensor::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

//////////////////////////////////////////////
/* Arduino Starts Here */
//////////////////////////////////////////////

// User Input.
DropSensor sensor;
int ioPin = Pin1;
int U[numberOfMeasurementPerCount];
double startTime = 0;
unsigned int K = 0;
int siz = 0;
int count, Sizda;
void
setup()
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(ioPin, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.print("Initializing"); // Ready message
  for (int i = 0; i != 5; i++) {
    Serial.print(".");
    delay(50);
  }
  Serial.println("Ready!");
}

void
loop()
{

  ////////////////////////////
  /*Sensor 1*/
  ///////////////////////////

  digitalWrite(ioPin, HIGH); // Toggle sensor 1
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(stimeout);
  sensor.setAddress(0x39); // Here's the trick, Initial Address(0x52)

  ///////////////////Sizda algorithm//////////////////
  int rng = (numberOfMeasurementPerCount);

  if (rng < 100) {        //if range is less than 100mm
    for (int i = 0; i != 2000000; i++) {
      rng = sensor.readRangeSingleMillimeters();
      if (rng > 100) {
        count = i;
        break;
      } else {
        count = 0;
      }
    }
  }
  if (count != NULL) {
    Sizda += 1;
  }
  if (Sizda > 1) {
    Serial.println(Sizda / 2);

    if (Sizda > 8) // Resetting to 'ONE' after subsequent 4 Raqat, setting back to Zero may lead confusion in gesture 
    {
      Sizda = 1;
    }
    if (Sizda == 2) {
      digitalWrite(13, HIGH);
    } else if (Sizda == 4) {
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
    } else if (Sizda == 6) {
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
    } else if (Sizda == 8) {
      digitalWrite(11, LOW);
    }
  }
  ///////////////////Sizda algorithm//////////////////
}

