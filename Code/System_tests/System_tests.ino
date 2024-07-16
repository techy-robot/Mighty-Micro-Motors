/*
  Created by Asher Edwards
  Last updated Jul 16, 2024
*/

#include <hal_conf_extra.h> 
#include <SPI.h>

// All I have in here is definition for the external clock frequency I have

// Clock settings
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. Here I have enabled the external clcok as the source
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

const byte READ = 0b01000000;     // MA735 read command
const byte WRITE = 0b10000000;   // MA735's write command

#define LED PB12

#define CSA_A PA0 //PA0 - 2
#define CSA_B PA3 //PA3 - 5
#define CSA_C PA6 //PA6,PA7,PB0

#define OUT_A PB1 //PB1, PB14, PB14
#define OUT_B PA8 //PA8, PA9, PC6
#define OUT_C PA6 //PC7, PA10, PA11

#define CS1 PD2 //for the first SPI bus

//Found in the datasheet. This is in mV
#define VREFINT 1212
#define ADC_RANGE 4096

//                      RX    TX
HardwareSerial Serial1(PB7, PB6);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB12 as an output. This is the LED pin on my stm32 based motor control board
  pinMode(LED, OUTPUT);

  //pinMode(CS1, OUTPUT);
  //SPI.begin();

  pinMode(CSA_A, INPUT);
  pinMode(CSA_B, INPUT);
  pinMode(CSA_C, INPUT);
  analogReadResolution(12);//Max value is 4096, at 3.3v

  Serial1.begin(19200);

  //Blink twice on start
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}

// the loop function runs over and over again forever
void loop() {

  int a = analogRead(CSA_A);
  int b = analogRead(CSA_B);
  int c = analogRead(CSA_C);
  float vref = readVref();

  Serial1.print("Clock freq is: ");//check system clock speed
  Serial1.println(F_CPU);
  Serial1.print("CSA Voltages: ");
  Serial1.print(String(readVoltage(vref, CSA_A)) + ", ");
  Serial1.print(String(readVoltage(vref, CSA_B)) + ", ");
  Serial1.println(String(readVoltage(vref, CSA_C)));
  Serial1.println("Vref(mV):" + String(vref));
  Serial1.println("Vbat(V):" + String(readVoltage(vref, AVBAT)));
  Serial1.println("Temperature(°C):" + String(readTempSensor(vref)));
  //Serial1.println("Sensor Data: " + String(readSensor()));
  delay(2000);
}

//alternative analog to voltage, but accounting for voltage reference
static float readVoltage(int32_t VRef, uint32_t pin) {
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin), LL_ADC_RESOLUTION_12B))/1000.0;
}


static int32_t readVref() {
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
}

static int32_t readTempSensor(int32_t VRef) {
  return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION_12B));
}

//Read the sensor data
unsigned int readSensor() {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return

  // take the chip select low to select the device:
  digitalWrite(CS1, LOW);
  // send a value of 0 to read the bytes
  result = SPI.transfer(0x00);//first part
  result = result << 8;
  inByte = SPI.transfer(0x00);//second part
  // combine the byte you just got with the previous one:
  result = result | inByte;
  // take the chip select high to de-select:
  digitalWrite(CS1, HIGH);
  // return the result:
  return (result);
}

//Read from a register:
unsigned int readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  Serial.print(thisRegister, BIN);
  Serial.print("\t");
  // MA735 expects the register name to be in the lower 5 bits
  // of the byte. So shift the bits right by three bits:
  thisRegister = thisRegister >> 3;
  // now combine the address and the command into one byte
  uint16_t dataToSend = READ & thisRegister ; //Read should be 010 in binary
  Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(CS1, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(CS1, HIGH);
  // return the result:
  return (result);
}


