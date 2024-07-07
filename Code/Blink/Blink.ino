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

#define LED PB12

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB12 as an output. This is the LED pin on my stm32 based motor control board
  pinMode(LED, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  float in, out;
  
  for (in = 0; in < 6.283; in = in + 0.001)
  {
    out = sin(in) * 127.5 + 127.5;
    analogWrite(LED,out);
  }
}
