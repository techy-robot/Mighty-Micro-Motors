#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/ma735/MagneticSensorMA735.h"
#include <Adafruit_NeoPixel.h>


//-----------------Pins-------------------

#define LED 21

#define CSA_A 1
#define CSA_B 2
#define CSA_C 3 

#define OUT_A 4  //GPIO 4
#define OUT_B 5  //GPIO 5
#define OUT_C 6  //GPIO 6

#define CS1 7  

unsigned long t0;
unsigned long t2;
int diff; //continuously calculated

//-------------------Interfaces-------------------------

SPIClass SPI_1(HSPI);// Had to move pin definitions to init

//                     num pixel, pin, pixel format
Adafruit_NeoPixel pixel(1, LED, NEO_GRB + NEO_KHZ800);

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


//BROKEN IN 2.3.4. Something changed
//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01, 25, CSA_A, CSA_B, CSA_C);
//DRV8311 I set to have a gain of 0.25V/A. Translating this to resistance it should be 0.01ohms and 25 gain. V = IR, V = 1a*0.01. V is 0.01, x 25 = 0.25 volts an amp.

//instantiate commander
Commander command = Commander(Serial, '\n', false);

void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void readSensor(char* cmd) {
  // get the angle, in radians, no full rotations
  float angle = sensor.getCurrentAngle();
  float degrees = angle * 360 / 6.28318530718;
  Serial.print("Angle (rad): ");
  Serial.println(angle, 7);  //7 specifies the decimal places
  Serial.print("Angle (deg): ");
  Serial.println(degrees, 5);

  // get the angle, in radians, including full rotations
  float angle_sum = sensor.getAngle();

  // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
  float vel = sensor.getVelocity();
  Serial.print("Velocity: ");
  Serial.println(vel);

  // get the field strength
  FieldStrength fs = sensor.getFieldStrength();
  Serial.print("Field strength: ");
  Serial.println(fs);
}

void flashLED(char* cmd) {
  pixel.setPixelColor(0, pixel.Color(100, 0, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();
}

void readUpdateTime(char* cmd) {
  Serial.print("Update time");
  Serial.println(diff);
}


void setup() {
  pixel.begin();

  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  while(!Serial && millis()<10000) {
    //wait for serial to connect, for up to 10 seconds
  }
  //Flash that serial is connected
  pixel.setPixelColor(0, pixel.Color(100, 0, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();
  // monitoring port
  motor.useMonitoring(Serial);

  SimpleFOCDebug::enable(&Serial);  //enable debug code too

  // MA735 supports mode 0 and mode 3
  // In the driver I have it set to 20mhz instead of 1
  // initialize magnetic sensor hardware
  //     MOSI  MISO  SCLK  SSEL
  SPI_1.begin(8, 9, 10, CS1);
  sensor.init(&SPI_1);
  sensor.setResolution(9);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  //Note: Do not run register writes every time the program runs. The chip has only 1000 flash write cycles! My driver has a check for this, others do not!
  //sensor.setBiasCurrentTrimming(0);  //fine, since it is my driver

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
  motor.torque_controller = TorqueControlType::foc_current;//voltage, dc_current, or foc_current

  // choose FOC modulation
  // SinePWM; (default)
  // SpaceVectorPWM; Similar to sine wave, not sure the diff
  // Trapezoid_120; Faster,but less efficient
  // Trapezoid_150; Same, except the angle offset is more
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//Trapezoid_120 is faster, but SVPWM is more efficient
  //motor.voltage_limit = 1;//Should be really low for drone motors. Ignored since I provided phase resistance
  motor.current_limit = 2.5;  // Amps

  motor.voltage_sensor_align = 0.5;//I=V/R. This should get me 2.5 current during alignment. I COMPLETELY spaced on this earlier when I set it to 5v and nothing happened (waveforms capped)
  //motor.zero_electric_angle = 1.65;//Skip alignment since I know the position. The auto align always gets alignment wrong and cause one direction to be faster than other.
  //motor.sensor_direction = Direction::CCW;


  // PID velocity
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 4.0;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;//seconds

  motor.PID_current_d.P = 0.2;
  motor.PID_current_d.I = 20.0;

  motor.PID_current_q.P = 0.2;
  motor.PID_current_q.I = 20.0;

  motor.motion_downsample = 2;

  // initialize motor
  motor.init();

  // align encoder and start FOC. This can be skipped once you have tuned your motor and got the absolute zero offset of the encoder. See docs
  motor.initFOC();
  //current_sense.driverAlign(1);

  command.add('M', onMotor, "my motor");  //default motor call;

  command.add('S', readSensor, "Read magnetic encoder");

  command.add('F', flashLED, "flashLED");
  command.add('T', readUpdateTime, "Read the update time");

  motor.monitor();

  //Finished setup so flash led
  pixel.setPixelColor(0, pixel.Color(100, 0, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();
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

  // Serial.print(currents.a*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents.b*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents.c*1000); // milli Amps
  // Serial.print("\t");
  // Serial.println(current_magnitude*1000); // milli Amps

  // read user commands
  command.run();

  t2=_micros();
  diff = t2-t0;

}
