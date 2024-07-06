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

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB12 as an output. This is the LED pin on my stm32 based motor control board
  pinMode(PB12, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PB12, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(PB12, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}
