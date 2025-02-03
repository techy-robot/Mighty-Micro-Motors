# stm32g070 and DRV8311 single motor board - V1 Alpha

- Since I'm using 3x pwm mode for the driver, I should have tied the unused lowside pins to 3.3v, and not left them floating, according to the datasheet. To fix on my board, I have to solder a little wire to several leads on the QFN package, which is a hassle.
- I bought the wrong mosfets, which ended up being way to small for the footprint. I had a bodge fix put in place. Fixed in commit #f2471eab10ce7c932bcc07bac4c89c711445ac51
- The Nmos transistors do not work very well (besides size), and have a voltage drop of 0.6v!
- I can't get a 3.3volt power output from the driver chip on power-up, becauser there is a recursive design flaw. The driver chip has an enable pin with an internal weak pulldown. I put a pull-up resistor on the outside, to always keep the chip on. Guess what that is connected to though? The regulator output from the chip, which is disabled! :skull: I can however jumpstart this if I supply an external 3.3 volts, and then remove it.
- The LED circuit initially was designed for a tri-state, however the status indicator on the driver chip is not a tri-state. I fixed this in commit #f2471eab10ce7c932bcc07bac4c89c711445ac51, and only have the highside LED to indicated charging.
- LED resistor values were found last minute, and were not correct in the design. Fixed in commit #f2471eab10ce7c932bcc07bac4c89c711445ac51
- The 220uF aluminum capacitor is the wrong size, but still manages to work. The 220uF surface mount capacitor in design does not have a real life counterpart.

# esp32-s3 single motor & battery controller

- The polarity is swapped on the Balance charger plug, the positive wire on my turnagy batteries is the left one.
- I forgot to add a series resistor on the 40mhz crystal.
- There is a 10**thousand** ohm resistor on the VCC pin for the charger chip, limiting the current to 0.9 milliamps and effectively crippling the chip. It should have been a 10 ohm resistor to limit spikes.
- I have a redundant input current rush limiter circuit on VCC and on the adaptor input.
- The small mosfets have the wrong footprint, Sot323 instead of Sot23, meaning the part is  too large to fit on the pads.
- All the Mosfets have the wrong pin numbers assigned to them in the schematic! The large mosfets have source and drain swapped, and the small mosfets have gate and drain swapped
- I have one capacitor that was supposed to be on the current sense line instead being on the charger input line. It will be noisy!
- The i2c lines are crossing a ground plane split, and are going right near noisy components!
