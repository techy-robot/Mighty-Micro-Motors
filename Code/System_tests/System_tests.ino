#include <hal_conf_extra.h> 

/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman
  modified 5 July 2024 for motor control board
  by Asher Edwards

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

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

#define LED PB12

#define CSA_A PA0 //PA0 - 2
#define CSA_B PA3 //PA3 - 5
#define CSA_C PA6 //PA6,PA7,PB0

#define OUT_A PB1 //PB1, PB14, PB14
#define OUT_B PA8 //PA8, PA9, PC6
#define OUT_C PA6 //PC7, PA10, PA11

#define CS1 PD2 //for the first SPI bus

//                      RX    TX
HardwareSerial Serial1(PB7, PB6);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB12 as an output. This is the LED pin on my stm32 based motor control board
  pinMode(LED, OUTPUT);
  pinMode(CS1, INPUT);//check if external pullup works

  pinMode(CSA_A, INPUT);
  pinMode(CSA_B, INPUT);
  pinMode(CSA_C, INPUT);
  analogReadResolution(12);//Max value is 4096, at 3.3v

  Serial1.begin(19200);

  //Blink twice rapidly on start
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

// the loop function runs over and over again forever
void loop() {

  int a = analogRead(CSA_A);
  int b = analogRead(CSA_B);
  int c = analogRead(CSA_C);
  bool cs = digitalRead(CS1);
  
  Serial1.print("Clock freq is: ");//check system clock speed
  Serial1.println(F_CPU);
  Serial1.print("CSA Voltages: ");
  Serial1.print(String(anToV(a, 12)) + ", ");
  Serial1.print(String(anToV(b, 12)) + ", ");
  Serial1.println(String(anToV(c, 12)));
  Serial1.println("Chip Select: " + String(cs));
  delay(500);
}

//Converts analog value to volts, taking the bit resolution into account.
float anToV(int input, int res) {
  return input * (3.3 / pow(2, res));
}
