#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
//#include "encoders.h"
//#include "ma730/MA730.h"
//#include "ma730/MagneticSensorMA730SSI.h"
#include "encoders/ma735/MagneticSensorMA735.h"
#include <hal_conf_extra.h> 

// Clock settings, the only thing changed from default was the clock source for external
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

//-----------------Pins-------------------

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

//-------------------Interfaces-------------------------
//                      RX    TX
HardwareSerial Serial1(PB7, PB6);

//         MOSI  MISO  SCLK  SSEL
SPIClass SPI_1(PB5, PB4, PB3);

//-------------------Motor Driver--------------------

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(6, 0.2, 19000);
//Pole pairs is the number of poles / 2. BTW I DON'T know the phase resistance yet

// Update to the MA735 code after this
// MagneticSensorSPI(int cs, float bit_resolution, int angle_register)
//MagneticSensorSPI sensor = MagneticSensorSPI(CS1, 13, 0x3FFF);
MagneticSensorMA735 sensor(CS1);

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(OUT_A, OUT_B, OUT_C);

//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01, 25, CSA_A, CSA_B, CSA_C);

//instantiate commander
Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}

void setup() {

  pinMode(LED, OUTPUT);
  
  // MA735 supports mode 0 and mode 3
  //sensor.spi_mode = SPI_MODE0;
  // speed of the SPI clock signal - default 1MHz
  //sensor.clock_speed = 1000000;
  // initialize magnetic sensor hardware
  sensor.init(&SPI_1);
  // // link the motor to the sensor
  // motor.linkSensor(&sensor);

  //Note: Do not run register writes every time the program runs. The chip has only 1000 flash write cycles! My driver has a check for this, others do not!
  sensor.setBiasCurrentTrimming(0);//fine, since it is my driver

  // //init driver
  // // pwm frequency to be used [Hz]
  // driver.pwm_frequency = 20000;
  // // power supply voltage [V]
  // driver.voltage_power_supply = 4.2;
  // // Max DC voltage allowed - default voltage_power_supply
  // driver.voltage_limit = 5;
  // // driver init
  // driver.init();
  // // link the motor to the driver
  // motor.linkDriver(&driver);

  // // link the driver with the current sense
  // current_sense.linkDriver(&driver);

  // // Changing gain values to those in the drv8311 docs. I have to figure this out still. I may have to modifiy the library source to implement what the docs need
  // //current_sense.gain_a = 1.0 / shunt_resistor / gain;
  // //current_sense.gain_b = 1.0 / shunt_resistor / gain;
  // //current_sense.gain_c = 1.0 / shunt_resistor / gain;


  // // init current sense
  // current_sense.init();
  //  // link the motor to current sense
  // motor.linkCurrentSense(&current_sense);

  // // set control loop type to be used
  // motor.controller = MotionControlType::velocity;

  // // choose FOC modulation
  // // SinePWM; (default)
  // // SpaceVectorPWM; Similar to sine wave, not sure the diff
  // // Trapezoid_120; Faster,but less efficient
  // // Trapezoid_150; Same, except the angle offset is more
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // motor.voltage_limit = 1;//Should be really low for drone motors

  // // initialize motor
  // motor.init();

  // // align encoder and start FOC. This can be skipped once you have tuned your motor and got the absolute zero offset of the encoder. See docs
  // motor.initFOC();


  // use monitoring with the BLDCMotor
  Serial1.begin(115200);
  // monitoring port
  //motor.useMonitoring(Serial1);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}

void loop() {
   // FOC algorithm function
  //motor.loopFOC();

  // velocity control loop function
  // setting the target velocity to 2rad/s
  //motor.move(2);

  // monitoring function outputting motor variables to the serial terminal 
  //motor.monitor();//This slows things down BTW!

  // read user commands
  //commander.run();

    // update the sensor (only needed if using the sensor without a motor)
  sensor.update();

  // get the angle, in radians, including full rotations
  float angle_sum = sensor.getAngle();

  // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
  float vel = sensor.getVelocity();

  // get the angle, in radians, no full rotations
  float angle = sensor.getCurrentAngle();
  float degrees = angle * 360 / 6.28318530718;
  Serial1.print("Angle (rad): ");
  Serial1.println(angle,7);//7 specifies the decimal places
  Serial1.print("Angle (deg): ");
  Serial1.println(degrees,5);

  // get the field strength
  FieldStrength fs = sensor.getFieldStrength();
  Serial1.print("Field strength: ");
  Serial1.println(fs);

  float res = sensor.getResolution();
  Serial1.print("Resolution: ");
  Serial1.println(res);

  int Time = sensor.getUpdateTime();
  Serial1.print("Update Time (microsecs): ");
  Serial1.println(Time);

  uint16_t home = sensor.getZero();
  Serial1.print("Zero position: ");
  Serial1.println(home);

  uint8_t bias = sensor.getBiasCurrentTrimming();
  Serial1.print("Bias Current Trimming: ");
  Serial1.println(bias);

  uint8_t dir = sensor.getRotationDirection();
  Serial1.print("Rotation Direction: ");
  Serial1.println(dir);

  delay(400);
  // set pulses per turn for encoder mode
  //sensor.setPulsesPerTurn(999); // set to 999 if we want 1000 PPR == 4000 CPR
}

