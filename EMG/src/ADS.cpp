#include <Arduino.h>
#include <SPI.h>

// Hardware UART for data output - defined like in blink.h
HardwareSerial myDataSerial(1);  // Use UART1
#define DATA_SERIAL myDataSerial  // Hardware UART for ADS1256 data
#define DEBUG_SERIAL Serial       // USB Serial for system messages

//Pins defined for ESP32-C3
#define ADS_RST_PIN    3  //ADS1256 reset pin
#define ADS_RDY_PIN    2  //ADS1256 data ready...INTERRUPT PIN  
#define ADS_CS_PIN     7  //ADS1256 chip select

// ESP32-C3 SPI pins (for reference and debugging)
// MOSI: GPIO 6
// MISO: GPIO 5  
// SCK:  GPIO 4
// SS:   Can be any GPIO (we use GPIO 7)

//Function declarations
void initADS();
int32_t read_Value();
void swapChannel(int swapTo);
void waitforDRDY();
void DRDY_Interuppt();
long GetRegisterValue(uint8_t regAdress);
void SendCMD(uint8_t cmd);
void Reset();
void SetRegisterValue(uint8_t regAdress, uint8_t regValue);
int32_t read_Value1();
int32_t read_Value2();
void displayAllRegisters();
void showModeSelectionMenu();
void selectDifferentialChannels();
void selectSingleEndedChannels();
void parseChannelSelection(String input);
void testADS1256Communication();

float myVolts = 1.0000;
int32_t val1;//Holds the returned value

unsigned long myTimer;//millis() timer, holds the next time
int myChannel;//used to toggle between the channels
//The line below was worked out manually as in my finished project I have a voltage divider so use a different value
long voltAdjuster = 1580510;//converts the returned value to Volts.

//ads1256_constants
//using the definitions in this library: https://github.com/Flydroid/ADS12xx-Library

#define SPI_SPEED 2500000

/* For information to the register and settings see manual page (p..) */

/* ADS1248 Register (see p42 for Register Map) */

#define    STATUS    0x00 //Status Control Register 0
#define   MUX     0x01 //Multiplexer Control Register 0
#define   ADCON     0x02 //A/D Control Register 0
#define   DRATE   0x03 //A/D Data Rate Control Register 0
#define   IO        0X04 //GPIO Control Register 0
#define   OFC0    0x05 //Offset Calibration Coefficient Register 1
#define   OFC1    0x06 //Offset Calibration Coefficient Register 2
#define   OFC2    0x07 //Offset Calibration Coefficient Register 2
#define   FSC0    0x08 //Full scale Callibration Coefficient Register 0
#define   FSC1    0x09 //Full scale Callibration Coefficient Register 1
#define   FSC2    0x0A //Full scale Callibration Coefficient REgister 2

/*STATUS - Status Control Register 0 ( see p30)*/
/* BIT7 - BIT6 -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* ID   - ID   -  ID     -  ID     -  ORDER  -  ACAL   -  BUFEN  -  DRDY */
#define STATUS_RESET 0x01 // Reset STATUS Register
/*Bits 7 - 4 ID3, ID2, ID1, ID0 Factory Programmed Identification Bits(Read Only)*/
/*ORDER1:0  Data Output Bit Order*/
#define ORDER_MSB B00000000 // Most significant Bit first (default)
#define ORDER_LSB B00001000//Least significant Bit first
/*Input data is always shifted in most significant byte and bit first. Output data is always shifted out most significant
byte first. The ORDER bit only controls the bit order of the output data within the byte.*/
/*ACAL1:0 Auto Calibration*/
#define ACAL_OFF B00000000 // Auto Calibration Disabled (default)
#define ACAL_ON  B00000100 // Auto Calibration Enabled
/*When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
values.*/
/*BUFEN1:0 Analog Input Buffer Enable*/
#define BUFEN_OFF B00000000 //Buffer Disabled (default)
#define BUFEN_ON  B00000010 //BUffer Enabled
/*DRDY1:0 Data Ready (Read Only) Duplicates the state of the DRDY pin*/

/* MUX - Multiplexer Control Register 0 (see p31 - bring together with bitwise OR | */
/* BIT7  - BIT6  -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* PSEL3 - PSEL2 -  PSEL1  -  PSEL0  -  NSEL3  -  NSEL2   - NSEL1   - NSEL0 */
#define MUX_RESET 0x01      // Reset MUX0 Register
/* PSEL3:0 Positive input channel selection bits */
#define P_AIN0 B00000000 //(default)
#define P_AIN1 B00010000
#define P_AIN2 B00100000
#define P_AIN3 B00110000
#define P_AIN4 B01000000
#define P_AIN5 B01010000
#define P_AIN6 B01100000
#define P_AIN7 B01110000
#define P_AINCOM B10000000
/* NSEL3:0 Negativ input channel selection bits */
#define N_AIN0 B00000000
#define N_AIN1 B00000001 //(default)
#define N_AIN2 B00000010
#define N_AIN3 B00000011
#define N_AIN4 B00000100
#define N_AIN5 B00000101
#define N_AIN6 B00000110
#define N_AIN7 B00000111
#define N_AINCOM B00001000

/*ADCON - A/D Control Register 0 ( see p31)*/
/* BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* 0    - CLK1   -  CLK0   -  SDCS1  -  SDCS0  -  PGA2   -  PGA1   -  PAG0 */
#define ADCON_RESET 0x20 // Reset ADCON Register
/*CLK2:0 D0/CLKOUT Clock Out Rate Setting*/
#define CLK_OFF B00000000 //Clock Out off
#define CLK_1   B00100000 //Clock Out Frequency = fCLKIN (default)
#define CLK_2   B01000000 //Clock Out Frequency = fCLKIN/2
#define CLK_4   B01100000 //Clock Out Frequency = fCLKIN/4
/*When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.*/
/*SDCS2:0 Sensor Detection Current Sources*/
#define SDCS_OFF B00000000//Sensor Detect Off (default)
#define SDCS_05  B00001000//Sensor Detect Current 0.5?A
#define SDCS_2   B00010000//Sensor Detect Current 2?A
#define SDCS_10  B00011000//Sensor Detect Current 10?A
/*The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the
ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.*/
/*PGA3:0 Programmable Gain Amplifier Setting*/
#define PGA_1 //(default)
#define PGA_2
#define PGA_4
#define PGA_8
#define PGA_16
#define PGA_32
#define PGA_64 B00100111

/*DRATE - A/D Data Rate Register 0 ( see p32)*/
/* BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* DR7  - DR6    -  DR5    -  DR4    -  DR3    -  DR2    -  DR1    -  DR0 */
#define DRATE_RESET 0xF0 // Reset DRATE Register
/*DR7:0 Data Rate Setting*/
#define DR_30000 B11110000 //30.000 SPS (default)
#define DR_15000 B11100000 //15.000 SPS
#define DR_7500  B11010000 //7.500 SPS
#define DR_3750  B11000000 //3.750 SPS
#define DR_2000  B10110000 //2.000 SPS
#define DR_1000  B10100001 //1.000 SPS
#define DR_500   B10010010 //500 SPS
#define DR_100   B10000010 //100 SPS
#define DR_60    B01110010 //60 SPS
#define DR_50    B01100011 //50 SPS
#define DR_30    B01010011 //30 SPS
#define DR_25    B01000011 //25 SPS
#define DR_15    B00110011 //15 SPS
#define DR_10    B00100011 //10 SPS
#define DR_5     B00010011 //5 SPS
#define DR2_5    B00000011 //2,5 SPS

/*IO - GPIO Control Register 0 ( see p32)*/
/* BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* DIR3 - DIR2   -  DIR1   -  DIR0   -  DIO3   -  DIO2   -  DIO1   -  DIO0 */
#define IO_RESET 0xE0 // Reset IO Register
/*DIR3 - Digital I/O Direction for Pin D3*/
#define DIR3_OUT B00000000 //D3 is an output
#define DIR_IN   B10000000 //D3 is an input (default)
/*DIR2 - Digital I/O Direction for Pin D3*/
#define DIR2_OUT B00000000 //D2 is an output
#define DIR2_IN   B01000000 //D2 is an input (default)
/*DIR1 - Digital I/O Direction for Pin D3*/
#define DIR1_OUT B00000000 //D1 is an output
#define DIR1_IN   B00100000 //D1 is an input (default)
/*DIR0 - Digital I/O Direction for Pin D3*/
#define DIR0_OUT B00000000 //D0/CLKOUT is an output
#define DIR0_IN   B00010000 //D0/CLKOUT is an input (default)
/*DIO3:0 Status of Digital I/O, Read Only*/

/* SPI COMMAND DEFINITIONS (p34) */
/*SYSTEM CONTROL */
#define   WAKEUP    0x00  //Exit Sleep Mode
#define   STANDBY   0xFD  //Enter Sleep Mode
#define   SYNC    0xFC    //Synchornize the A/D Conversion
#define   RESET   0xFE  //Reset To Power UP values
#define   ADS_NOP     0xFF  //No operation (renamed to avoid conflict with ESP32 NOP())
/*DATA READ*/
#define   RDATA   0x01  //Read data once
#define   RDATAC    0x03  //Read data continously
#define   SDATAC    0x0F  //Stop reading data continously
/*READ REGISTER */
#define   RREG    0x10  //Read From Register
#define   WREG    0x50  //Write To Register
/*Calibration */
#define   SYSOCAL   0xF3  //System Offset Calibration
#define   SYSGCAL   0xF2  //System Gain Calibration
#define   SELFCAL     0xF0  //Self Offset Calibration

//Global variables for interrupt handling
volatile int DRDY_state = HIGH;

// Global variables for mode selection and buffering
bool useDifferentialMode = true;
uint8_t selectedChannels[8] = {0};
uint8_t numSelectedChannels = 0;
uint8_t currentChannelIndex = 0;

// Data buffering system for high-speed sampling
#define BUFFER_SIZE 512  // ESP32 internal buffer size
#define BATCH_SIZE 32    // Send data in batches to reduce UART load

struct DataPoint {
  uint8_t channel;
  int32_t raw_value;
  float voltage;
  unsigned long timestamp;
};

DataPoint dataBuffer[BUFFER_SIZE];
uint16_t bufferWriteIndex = 0;
uint16_t bufferReadIndex = 0;
uint16_t bufferedSamples = 0;

// Timing control
unsigned long lastSampleTime = 0;
unsigned long lastTransmitTime = 0;
const unsigned long SAMPLE_INTERVAL = 33;  // 30kSPS = 33.3us
const unsigned long TRANSMIT_INTERVAL = 50000; // Transmit every 50ms

void setup() {
  delay(1000);//let everything settle
  DEBUG_SERIAL.begin(115200);
  
  // Add multiple serial flush and delays to ensure USB Serial is ready
  delay(1000);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== ADS1256 ESP32 Data Logger ===");
  DEBUG_SERIAL.println("DEBUG_SERIAL (USB) is working!");
  DEBUG_SERIAL.flush();
  
  // Initialize Hardware UART with specific pins like in blink.h
  // GPIO 20 (RX), GPIO 21 (TX) 
  DATA_SERIAL.begin(115200, SERIAL_8N1, 20, 21);
  delay(500);
  
  DEBUG_SERIAL.println("Hardware UART initialized on GPIO 20(RX), 21(TX)");
  DEBUG_SERIAL.flush();
  
  // Test hardware UART
  DATA_SERIAL.println("# Hardware UART Test - ADS1256 Data Logger");
  
  DEBUG_SERIAL.println("Testing pin initialization...");
  DEBUG_SERIAL.flush();
  
  //initialize pins for the ADS
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);

  DEBUG_SERIAL.println("Pins initialized. Starting SPI...");
  DEBUG_SERIAL.flush();

  SPI.begin();
  
  DEBUG_SERIAL.println("SPI started. About to initialize ADS1256...");
  DEBUG_SERIAL.flush();
  
  DEBUG_SERIAL.println("Before initADS() call");
  DEBUG_SERIAL.flush();
  
  //set up the ads1256 board
  initADS();
  
  // Run detailed communication test
  testADS1256Communication();
  
  DEBUG_SERIAL.println("After initADS() call - ADS1256 initialization complete.");
  DEBUG_SERIAL.flush();
  
  // Mode selection interface
  showModeSelectionMenu();
  
  // Display all register values before starting
  displayAllRegisters();
  
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== Starting Data Collection ===");
  DEBUG_SERIAL.print("Mode: ");
  if (useDifferentialMode) {
    DEBUG_SERIAL.println("Differential Input");
  } else {
    DEBUG_SERIAL.println("Single-Ended Input");
  }
  DEBUG_SERIAL.print("Selected channels: ");
  for (int i = 0; i < numSelectedChannels; i++) {
    DEBUG_SERIAL.print(selectedChannels[i]);
    if (i < numSelectedChannels - 1) DEBUG_SERIAL.print(", ");
  }
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Sample Rate: 30000 SPS (Maximum)");
  DEBUG_SERIAL.println("Data will be sent to Hardware UART (GPIO21/20)");
  DEBUG_SERIAL.println("Format: CHx:raw_value,voltage");
  DEBUG_SERIAL.flush();
  
  // Send startup message to data UART
  DATA_SERIAL.println("# ADS1256 Data Stream Started");
  if (useDifferentialMode) {
    DATA_SERIAL.println("# Mode: Differential");
  } else {
    DATA_SERIAL.println("# Mode: Single-Ended");
  }
  DATA_SERIAL.println("# Format: CHx:raw_value,voltage");
  
  DEBUG_SERIAL.println("Setup complete - entering main loop");
  DEBUG_SERIAL.flush();
}

void loop() {
  unsigned long currentTime = micros();
  
  // High-speed sampling: 30kSPS
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    if (numSelectedChannels == 0) return;
    
    // Get current channel from selected channels array
    uint8_t currentChannel = selectedChannels[currentChannelIndex];
    
    // Configure channel
    if (useDifferentialMode) {
      uint8_t muxValue;
      switch (currentChannel) {
        case 0: muxValue = 0x01; break; // AIN0-AIN1
        case 1: muxValue = 0x10; break; // AIN1-AIN0
        case 2: muxValue = 0x23; break; // AIN2-AIN3
        case 3: muxValue = 0x32; break; // AIN3-AIN2
        case 4: muxValue = 0x45; break; // AIN4-AIN5
        case 5: muxValue = 0x54; break; // AIN5-AIN4
        case 6: muxValue = 0x67; break; // AIN6-AIN7
        case 7: muxValue = 0x76; break; // AIN7-AIN6
        default: muxValue = 0x01; break;
      }
      swapChannel(muxValue);
    } else {
      uint8_t muxValue = (currentChannel << 4) | 0x08; // AINx(+) - AINCOM(-)
      swapChannel(muxValue);
    }

    // Read ADC value
    val1 = read_Value();
    
    // Store in internal buffer if valid reading
    if (val1 != -1 && bufferedSamples < BUFFER_SIZE) {
      dataBuffer[bufferWriteIndex].channel = currentChannel;
      dataBuffer[bufferWriteIndex].raw_value = val1;
      dataBuffer[bufferWriteIndex].voltage = (1.0000 * val1) / voltAdjuster;
      dataBuffer[bufferWriteIndex].timestamp = currentTime;
      
      bufferWriteIndex = (bufferWriteIndex + 1) % BUFFER_SIZE;
      bufferedSamples++;
      
      // Move to next channel for next iteration
      currentChannelIndex = (currentChannelIndex + 1) % numSelectedChannels;
    }
  }
  
  // Batch transmission: Send data every 50ms
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL && bufferedSamples > 0) {
    lastTransmitTime = currentTime;
    
    // Send batch header
    DATA_SERIAL.print("BATCH:");
    DATA_SERIAL.print((uint16_t)min((uint16_t)bufferedSamples, (uint16_t)BATCH_SIZE));
    DATA_SERIAL.println();
    
    // Send batch data
    uint16_t sentCount = 0;
    while (bufferedSamples > 0 && sentCount < BATCH_SIZE) {
      DataPoint& point = dataBuffer[bufferReadIndex];
      
      DATA_SERIAL.print("CH");
      DATA_SERIAL.print(point.channel);
      DATA_SERIAL.print(":");
      DATA_SERIAL.print(point.raw_value);
      DATA_SERIAL.print(",");
      DATA_SERIAL.print(point.voltage, 4);
      DATA_SERIAL.print(",");
      DATA_SERIAL.println(point.timestamp);
      
      bufferReadIndex = (bufferReadIndex + 1) % BUFFER_SIZE;
      bufferedSamples--;
      sentCount++;
    }
    
    // Send batch footer
    DATA_SERIAL.println("END_BATCH");
    
    // Debug info
    DEBUG_SERIAL.print("Sent batch: ");
    DEBUG_SERIAL.print(sentCount);
    DEBUG_SERIAL.print(" samples, remaining: ");
    DEBUG_SERIAL.println(bufferedSamples);
  }
}

//Interrupt function
void DRDY_Interuppt() {
  DRDY_state = LOW;
}

void waitforDRDY() {
  unsigned long timeout = micros() + 50000; // 50ms timeout
  unsigned long startTime = micros();
  
  // 使用純輪詢方式，不依賴中斷
  while (digitalRead(ADS_RDY_PIN) == HIGH && micros() < timeout) {
    delayMicroseconds(10);
  }
  
  if (micros() >= timeout) {
    unsigned long elapsed = micros() - startTime;
    DEBUG_SERIAL.print("DRDY timeout after ");
    DEBUG_SERIAL.print(elapsed);
    DEBUG_SERIAL.println(" microseconds");
  }
  
  // 重置中斷狀態變數（雖然我們不依賴它）
  noInterrupts();
  DRDY_state = HIGH;
  interrupts();
}

void initADS() {
  DEBUG_SERIAL.println("Attaching interrupt...");
  attachInterrupt(digitalPinToInterrupt(ADS_RDY_PIN), DRDY_Interuppt, FALLING);

  DEBUG_SERIAL.println("Resetting ADS1256 via RST pin...");
  digitalWrite(ADS_RST_PIN, LOW);
  delay(10);
  digitalWrite(ADS_RST_PIN, HIGH);
  delay(1000);

  DEBUG_SERIAL.println("Sending RESET command via SPI...");
  Reset();
  delay(2000);

  DEBUG_SERIAL.print("Reading STATUS register: ");
  uint8_t status = GetRegisterValue(STATUS);
  DEBUG_SERIAL.print(status);
  DEBUG_SERIAL.print(" (0x");
  DEBUG_SERIAL.print(status, HEX);
  DEBUG_SERIAL.println(")");
  
  if (status == 0 || status == 255) {
    DEBUG_SERIAL.println("WARNING: ADS1256 not responding properly!");
    DEBUG_SERIAL.println("Check connections:");
    DEBUG_SERIAL.println("  MOSI -> GPIO 6");
    DEBUG_SERIAL.println("  MISO -> GPIO 5");
    DEBUG_SERIAL.println("  SCK  -> GPIO 4");
    DEBUG_SERIAL.println("  CS   -> GPIO 7");
    DEBUG_SERIAL.println("  DRDY -> GPIO 2");
    DEBUG_SERIAL.println("  RST  -> GPIO 3");
    DEBUG_SERIAL.println("  VDD  -> 3.3V or 5V");
    DEBUG_SERIAL.println("  GND  -> GND");
  }

  DEBUG_SERIAL.println("Setting MUX register...");
  SetRegisterValue(MUX,MUX_RESET);

  // Set to maximum sample rate for best performance
  DEBUG_SERIAL.println("Setting DRATE register to maximum (30000 SPS)...");
  SetRegisterValue(DRATE, DR_30000); // Use maximum sample rate
  DEBUG_SERIAL.println("Data rate set to 30000 SPS");

  delay(2000);

  DEBUG_SERIAL.println("Performing self-calibration...");
  SendCMD(SELFCAL);
  SendCMD(BUFEN_ON);

  delay(5);

  DEBUG_SERIAL.println("Reading calibration coefficients:");
  DEBUG_SERIAL.print("OFC0: ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC0));
  DEBUG_SERIAL.print("OFC1: ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC1));
  DEBUG_SERIAL.print("OFC2: ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC2));
  DEBUG_SERIAL.print("FSC0: ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC0));
  DEBUG_SERIAL.print("FSC1: ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC1));
  DEBUG_SERIAL.print("FSC2: ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC2));
}

//function to read a value
int32_t read_Value() {
  int32_t adc_val = 0;
  
  // 直接使用輪詢等待 DRDY，不檢查中斷狀態變數
  waitforDRDY(); // Wait until DRDY is LOW
  
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS_PIN, LOW);
  delayMicroseconds(5);
  
  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  
  delayMicroseconds(5);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val > 0x7fffff) { //if MSB == 1
    adc_val = adc_val - 16777216; //do 2's complement, keep the sign this time!
  }

  return adc_val;
}

long GetRegisterValue(uint8_t regAdress) {
  uint8_t bufr;
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(RREG | regAdress); // send 1st command byte, address of the register
  SPI.transfer(0x00);     // send 2nd command byte, read only one register
  delayMicroseconds(10);
  bufr = SPI.transfer(ADS_NOP); // read data of the register
  delayMicroseconds(10);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
  return bufr;
}

void SendCMD(uint8_t cmd) {
  waitforDRDY();
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with 4Mhz clock, MSB first, SPI Mode0
  digitalWrite(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(cmd);
  delayMicroseconds(10);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}

void Reset() {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(RESET); //Reset
  delay(2); //Minimum 0.6ms required for Reset to finish.
  SPI.transfer(SDATAC); //Issue SDATAC
  delayMicroseconds(100);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}

void SetRegisterValue(uint8_t regAdress, uint8_t regValue) {
  uint8_t regValuePre = GetRegisterValue(regAdress);
  if (regValue != regValuePre) {
    delayMicroseconds(10);
    waitforDRDY();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with SPI_SPEED, MSB first, SPI Mode1
    digitalWrite(ADS_CS_PIN, LOW);
    delayMicroseconds(10);
    SPI.transfer(WREG | regAdress); // send 1st command byte, address of the register
    SPI.transfer(0x00);   // send 2nd command byte, write only one register
    SPI.transfer(regValue);         // write data (1 Byte) for the register
    delayMicroseconds(10);
    digitalWrite(ADS_CS_PIN, HIGH);
    SPI.endTransaction();
    
    if (regValue != GetRegisterValue(regAdress)) {   //Check if write was succesfull
      DEBUG_SERIAL.print("Write to Register 0x");
      DEBUG_SERIAL.print(regAdress, HEX);
      DEBUG_SERIAL.println(" failed!");
    }
    else {
      DEBUG_SERIAL.println("success");
    }
  }
}

//function to swap between the different channels
void swapChannel(int swapTo) {
  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1247

  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(swapTo);     //pins registers 2 and 3
  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}

int32_t read_Value1() {
  int32_t adc_val = 0;
  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1247

  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x67);     //pins registers 2 and 3

  //now we need to sync
  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val > 0x7fffff) { //if MSB == 1
    adc_val = adc_val - 16777216; //do 2's complement, keep the sign this time!
  }

  return adc_val;
}

int32_t read_Value2() {
  int32_t adc_val = 0;
  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1247

  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x01);     //pins registers 2 and 3

  //now we need to sync
  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(ADS_NOP);
  digitalWrite(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val > 0x7fffff) { //if MSB == 1
    adc_val = adc_val - 16777216; //do 2's complement, keep the sign this time!
  }

  return adc_val;
}

void displayAllRegisters() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== ADS1256 Register Status ===");
  
  uint8_t statusReg = GetRegisterValue(STATUS);
  DEBUG_SERIAL.print("STATUS (0x00): 0x");
  DEBUG_SERIAL.print(statusReg, HEX);
  DEBUG_SERIAL.print(" (");
  DEBUG_SERIAL.print(statusReg, BIN);
  DEBUG_SERIAL.println(")");
  DEBUG_SERIAL.print("  - Chip ID: ");
  DEBUG_SERIAL.println((statusReg >> 4) & 0x0F);
  DEBUG_SERIAL.print("  - Buffer: ");
  DEBUG_SERIAL.println((statusReg & BUFEN_ON) ? "Enabled" : "Disabled");
  DEBUG_SERIAL.print("  - Auto Cal: ");
  DEBUG_SERIAL.println((statusReg & ACAL_ON) ? "Enabled" : "Disabled");
  
  uint8_t muxReg = GetRegisterValue(MUX);
  DEBUG_SERIAL.print("MUX (0x01): 0x");
  DEBUG_SERIAL.print(muxReg, HEX);
  DEBUG_SERIAL.print(" - Pos: AIN");
  DEBUG_SERIAL.print((muxReg >> 4) & 0x0F);
  DEBUG_SERIAL.print(", Neg: AIN");
  DEBUG_SERIAL.println(muxReg & 0x0F);
  
  uint8_t adconReg = GetRegisterValue(ADCON);
  DEBUG_SERIAL.print("ADCON (0x02): 0x");
  DEBUG_SERIAL.print(adconReg, HEX);
  DEBUG_SERIAL.print(" - PGA: ");
  switch(adconReg & 0x07) {
    case 0: DEBUG_SERIAL.println("1"); break;
    case 1: DEBUG_SERIAL.println("2"); break;
    case 2: DEBUG_SERIAL.println("4"); break;
    case 3: DEBUG_SERIAL.println("8"); break;
    case 4: DEBUG_SERIAL.println("16"); break;
    case 5: DEBUG_SERIAL.println("32"); break;
    case 6: DEBUG_SERIAL.println("64"); break;
    default: DEBUG_SERIAL.println("Unknown"); break;
  }
  
  uint8_t drateReg = GetRegisterValue(DRATE);
  DEBUG_SERIAL.print("DRATE (0x03): 0x");
  DEBUG_SERIAL.print(drateReg, HEX);
  DEBUG_SERIAL.print(" - Sample Rate: ");
  switch(drateReg) {
    case DR_30000: DEBUG_SERIAL.println("30000 SPS"); break;
    case DR_15000: DEBUG_SERIAL.println("15000 SPS"); break;
    case DR_7500: DEBUG_SERIAL.println("7500 SPS"); break;
    case DR_3750: DEBUG_SERIAL.println("3750 SPS"); break;
    case DR_2000: DEBUG_SERIAL.println("2000 SPS"); break;
    case DR_1000: DEBUG_SERIAL.println("1000 SPS"); break;
    case DR_500: DEBUG_SERIAL.println("500 SPS"); break;
    case DR_100: DEBUG_SERIAL.println("100 SPS"); break;
    case DR_60: DEBUG_SERIAL.println("60 SPS"); break;
    case DR_50: DEBUG_SERIAL.println("50 SPS"); break;
    case DR_30: DEBUG_SERIAL.println("30 SPS"); break;
    case DR_25: DEBUG_SERIAL.println("25 SPS"); break;
    case DR_15: DEBUG_SERIAL.println("15 SPS"); break;
    case DR_10: DEBUG_SERIAL.println("10 SPS"); break;
    case DR_5: DEBUG_SERIAL.println("5 SPS"); break;
    case DR2_5: DEBUG_SERIAL.println("2.5 SPS"); break;
    default: DEBUG_SERIAL.println("Unknown"); break;
  }
  
  DEBUG_SERIAL.print("IO (0x04): 0x");
  DEBUG_SERIAL.println(GetRegisterValue(IO), HEX);
  
  DEBUG_SERIAL.println("Calibration Coefficients:");
  DEBUG_SERIAL.print("  OFC0 (0x05): ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC0));
  DEBUG_SERIAL.print("  OFC1 (0x06): ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC1));
  DEBUG_SERIAL.print("  OFC2 (0x07): ");
  DEBUG_SERIAL.println(GetRegisterValue(OFC2));
  DEBUG_SERIAL.print("  FSC0 (0x08): ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC0));
  DEBUG_SERIAL.print("  FSC1 (0x09): ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC1));
  DEBUG_SERIAL.print("  FSC2 (0x0A): ");
  DEBUG_SERIAL.println(GetRegisterValue(FSC2));
}

void showModeSelectionMenu() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== Mode Selection Menu ===");
  DEBUG_SERIAL.println("Select input mode:");
  DEBUG_SERIAL.println("1. Differential Input");
  DEBUG_SERIAL.println("2. Single-Ended Input");
  DEBUG_SERIAL.print("Enter choice (1 or 2): ");
  DEBUG_SERIAL.flush();
  
  while (true) {
    if (DEBUG_SERIAL.available() > 0) {
      int choice = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.readStringUntil('\n'); // Clear buffer
      
      if (choice == 1) {
        useDifferentialMode = true;
        DEBUG_SERIAL.println("1");
        DEBUG_SERIAL.println("Differential mode selected.");
        selectDifferentialChannels();
        break;
      } else if (choice == 2) {
        useDifferentialMode = false;
        DEBUG_SERIAL.println("2");
        DEBUG_SERIAL.println("Single-Ended mode selected.");
        selectSingleEndedChannels();
        break;
      } else {
        DEBUG_SERIAL.print("Invalid choice. Enter 1 or 2: ");
        DEBUG_SERIAL.flush();
      }
    }
    delay(100);
  }
}

void selectDifferentialChannels() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== Differential Channel Selection ===");
  DEBUG_SERIAL.println("Available differential pairs:");
  DEBUG_SERIAL.println("0: AIN0-AIN1    1: AIN1-AIN0");
  DEBUG_SERIAL.println("2: AIN2-AIN3    3: AIN3-AIN2");
  DEBUG_SERIAL.println("4: AIN4-AIN5    5: AIN5-AIN4");
  DEBUG_SERIAL.println("6: AIN6-AIN7    7: AIN7-AIN6");
  DEBUG_SERIAL.println("Enter channels (e.g., 0,6 for AIN0-AIN1 and AIN6-AIN7):");
  DEBUG_SERIAL.print("Channels: ");
  DEBUG_SERIAL.flush();
  
  while (true) {
    if (DEBUG_SERIAL.available() > 0) {
      String input = DEBUG_SERIAL.readStringUntil('\n');
      DEBUG_SERIAL.println(input);
      parseChannelSelection(input);
      break;
    }
    delay(100);
  }
}

void selectSingleEndedChannels() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== Single-Ended Channel Selection ===");
  DEBUG_SERIAL.println("Available channels: 0, 1, 2, 3, 4, 5, 6, 7");
  DEBUG_SERIAL.println("Enter channels (e.g., 0,1,2 for AIN0, AIN1, AIN2):");
  DEBUG_SERIAL.print("Channels: ");
  DEBUG_SERIAL.flush();
  
  while (true) {
    if (DEBUG_SERIAL.available() > 0) {
      String input = DEBUG_SERIAL.readStringUntil('\n');
      DEBUG_SERIAL.println(input);
      parseChannelSelection(input);
      break;
    }
    delay(100);
  }
}

void parseChannelSelection(String input) {
  numSelectedChannels = 0;
  int startIndex = 0;
  
  for (int i = 0; i <= input.length(); i++) {
    if (i == input.length() || input.charAt(i) == ',' || input.charAt(i) == ' ') {
      if (i > startIndex) {
        int channel = input.substring(startIndex, i).toInt();
        if (useDifferentialMode && channel >= 0 && channel <= 7) {
          selectedChannels[numSelectedChannels++] = channel;
        } else if (!useDifferentialMode && channel >= 0 && channel <= 7) {
          selectedChannels[numSelectedChannels++] = channel;
        }
      }
      startIndex = i + 1;
    }
  }
  
  DEBUG_SERIAL.print("Selected ");
  DEBUG_SERIAL.print(numSelectedChannels);
  DEBUG_SERIAL.print(" channels: ");
  for (int i = 0; i < numSelectedChannels; i++) {
    DEBUG_SERIAL.print(selectedChannels[i]);
    if (i < numSelectedChannels - 1) DEBUG_SERIAL.print(", ");
  }
  DEBUG_SERIAL.println();
}

void testADS1256Communication() {
  DEBUG_SERIAL.println("=== ADS1256 Communication Test ===");
  
  // 測試多次讀取 STATUS 註冊器
  for (int i = 0; i < 5; i++) {
    uint8_t status = GetRegisterValue(STATUS);
    DEBUG_SERIAL.print("STATUS read ");
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(": 0x");
    DEBUG_SERIAL.println(status, HEX);
    delay(100);
  }
  
  // 測試讀取其他註冊器
  uint8_t registers[] = {STATUS, MUX, ADCON, DRATE, IO};
  const char* names[] = {"STATUS", "MUX", "ADCON", "DRATE", "IO"};
  
  DEBUG_SERIAL.println("All register readings:");
  for (int i = 0; i < 5; i++) {
    uint8_t value = GetRegisterValue(registers[i]);
    DEBUG_SERIAL.print(names[i]);
    DEBUG_SERIAL.print(": 0x");
    DEBUG_SERIAL.println(value, HEX);
  }
  
  // 檢查是否所有讀取都是 0xFF
  bool allFF = true;
  for (int i = 0; i < 5; i++) {
    if (GetRegisterValue(registers[i]) != 0xFF) {
      allFF = false;
      break;
    }
  }
  
  if (allFF) {
    DEBUG_SERIAL.println("❌ CRITICAL: All registers read as 0xFF");
    DEBUG_SERIAL.println("❌ ADS1256 appears to be damaged or not connected");
  } else {
    DEBUG_SERIAL.println("✅ Some communication detected - may be salvageable");
  }
}

