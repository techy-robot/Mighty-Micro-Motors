#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
//#include "encoders.h"
//#include "ma730/MA730.h"
//#include "ma730/MagneticSensorMA730SSI.h"
#include "encoders/ma735/MagneticSensorMA735.h"
#include <hal_conf_extra.h>

// Clock settings, the only thing changed from default was the clock source for external
extern "C" void SystemClock_Config(void) {
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

#define CSA_A PA0  //PA0 - 2
#define CSA_B PA3  //PA3 - 5
#define CSA_C PA6  //PA6,PA7,PB0

//I need the pins to be on the same timer,
// because Simple foc couldn't find a master timer replacement for any connected to TIM1. 
// I guess TIM1 should be reserved for master clock, it is the advanced timer.

#define OUT_A PB1  //PB1, PB14, PB14
#define OUT_B PC6  //PA8, PA9, PC6
#define OUT_C PC7  //PC7, PA10, PA11

#define CS1 PD2  //for the first SPI bus

//Found in the datasheet. This is in mV
#define VREFINT 1212
#define ADC_RANGE 4096

unsigned long t0;
unsigned long t2;

//-------------------Interfaces-------------------------
//                      RX    TX
HardwareSerial Serial1(PB7, PB6);

//         MOSI  MISO  SCLK  SSEL
SPIClass SPI_1(PB5, PB4, PB3);

//-------------------Motor Driver--------------------

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(6, 0.2, 22000);  //KV on my motor is 19000, docs suggest going 50% higher than that
//Pole pairs is the number of poles / 2. Phase resistance is 0.2 ohms, since I got 0.4 ohm between two leads and I saw a little wire tail indicating Wye configuration

// Update to the MA735 code after this
// MagneticSensorSPI(int cs, float bit_resolution, int angle_register)
//MagneticSensorSPI sensor = MagneticSensorSPI(CS1, 13, 0x3FFF);
MagneticSensorMA735 sensor(CS1);

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(OUT_A, OUT_B, OUT_C);

//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01, 25, CSA_A, CSA_B, CSA_C);
//DRV8311 I set to have a gain of 0.25V/A. Translating this to resistance it should be 0.01ohms and 25 gain. V = IR, V = 1a*0.01. V is 0.01, x 25 = 0.25 volts an amp.

//instantiate commander
Commander command = Commander(Serial1, '\n', false);

void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}
void doLimitCurrent(char* cmd) {
  command.scalar(&motor.current_limit, cmd);
}
void readSensor(char* cmd) {
  // get the angle, in radians, no full rotations
  float angle = sensor.getCurrentAngle();
  float degrees = angle * 360 / 6.28318530718;
  Serial1.print("Angle (rad): ");
  Serial1.println(angle, 7);  //7 specifies the decimal places
  Serial1.print("Angle (deg): ");
  Serial1.println(degrees, 5);

  // get the angle, in radians, including full rotations
  float angle_sum = sensor.getAngle();

  // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
  float vel = sensor.getVelocity();
  Serial1.print("Velocity: ");
  Serial1.println(vel);

  // get the field strength
  FieldStrength fs = sensor.getFieldStrength();
  Serial1.print("Field strength: ");
  Serial1.println(fs);
}

void flashLED(char* cmd) {
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}


void setup() {

  pinMode(LED, OUTPUT);

  // use monitoring with the BLDCMotor
  Serial1.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial1);

  SimpleFOCDebug::enable(&Serial1);  //enable debug code too

  // MA735 supports mode 0 and mode 3
  //Might want to increase speed beyond 1mhz later
  // initialize magnetic sensor hardware
  sensor.init(&SPI_1);
  sensor.setResolution(9);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  //Note: Do not run register writes every time the program runs. The chip has only 1000 flash write cycles! My driver has a check for this, others do not!
  //sensor.setBiasCurrentTrimming(0);  //fine, since it is my driver
  //sensor.setResolution(10.0);

  //init driver
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 4.2;
  // driver init
  driver.init();
  // link the motor to the driver
  motor.linkDriver(&driver);

  // link the driver with the current sense
  current_sense.linkDriver(&driver);

  // Changing gain values to those in the drv8311 docs. I have to figure this out still. I may have to modifiy the library source to implement what the docs need
  //current_sense.gain_a = 1.0 / shunt_resistor / gain;
  //current_sense.gain_b = 1.0 / shunt_resistor / gain;
  //current_sense.gain_c = 1.0 / shunt_resistor / gain;

  // init current sense
  current_sense.init();
  current_sense.skip_align = true;//I am sure in my configuration
  // link the motor to current sense
  motor.linkCurrentSense(&current_sense);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // voltage torque control mode
  motor.torque_controller = TorqueControlType::voltage;//voltage, dc_current, or foc_current

  // choose FOC modulation
  // SinePWM; (default)
  // SpaceVectorPWM; Similar to sine wave, not sure the diff
  // Trapezoid_120; Faster,but less efficient
  // Trapezoid_150; Same, except the angle offset is more
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //motor.voltage_limit = 1;//Should be really low for drone motors. Ignored since I provided phase resistance
  motor.current_limit = 2.5;  // Amps

  motor.voltage_sensor_align = 0.5;//I=V/R. This should get me 2.5 current during alignment. I COMPLETELY spaced on this earlier when I set it to 5v and nothing happened (waveforms capped)
  motor.zero_electric_angle = 1.65;//Skip alignment since I know the position. The auto align always gets alignment wrong and cause one direction to be faster than other.
  motor.sensor_direction = Direction::CCW;


  // PID velocity
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 4.0;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;//seconds

  motor.motion_downsample = 2;

  // initialize motor
  motor.init();

  // align encoder and start FOC. This can be skipped once you have tuned your motor and got the absolute zero offset of the encoder. See docs
  motor.initFOC();
  //current_sense.driverAlign(1);

  command.add('M', onMotor, "my motor");  //default motor call
  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  command.add('S', readSensor, "Read magnetic encoder");

  command.add('F', flashLED, "flashLED");

  motor.monitor();

  //Finished setup so flash led
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
}

void loop() {
  t0=_micros();
  // FOC algorithm function
  motor.loopFOC();
  //driver.setPwm(1.1,2.5,4);

  // velocity control loop function
  // setting the target velocity to 2rad/s
  motor.move();

  if (motor.target == 0.0f && motor.enabled==1)
    motor.disable();
  if (motor.target != 0.0f && motor.enabled==0)
    motor.enable();

  // monitoring function outputting motor variables to the serial terminal
  motor.monitor();//This slows things down BTW!

  //PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  //float current_magnitude = current_sense.getDCCurrent();

  // Serial1.print(currents.a*1000); // milli Amps
  // Serial1.print("\t");
  // Serial1.print(currents.b*1000); // milli Amps
  // Serial1.print("\t");
  // Serial1.print(currents.c*1000); // milli Amps
  // Serial1.print("\t");
  // Serial1.println(current_magnitude*1000); // milli Amps

  // read user commands
  command.run();

  t2=_micros();
  //Serial1.println(t2-t0);

}
