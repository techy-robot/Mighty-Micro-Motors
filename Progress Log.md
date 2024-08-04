# Progress Log
This is where I document my progress with pictures if I'm building something, or a description of what I did if I'm coding.

## July 1st, 2024
Today I assembled the bottom side of my control board.

**Setup:**

![1](Media/Build%20Log/IMG_20240701_144509.jpg)
![1](Media/Build%20Log/IMG_20240701_144550.jpg)

**Pasting**

![1](Media/Build%20Log/IMG_20240701_145850.jpg)

Got all the capacitors on the bottom

![1](Media/Build%20Log/IMG_20240701_153903.jpg)
![1](Media/Build%20Log/IMG_20240701_155053.jpg)

Added all the resistors

![1](Media/Build%20Log/IMG_20240701_161703.jpg)

I had to place the battery protection IC using a microscope cause it's so tiny!

![1](Media/Build%20Log/IMG_20240701_162515.jpg)
![1](Media/Build%20Log/IMG_20240701_162531.jpg)
![1](Media/Build%20Log/IMG_20240701_163042.jpg)

Reflowing

![1](Media/Build%20Log/IMG_20240701_173111.jpg)

Finished the bottom, so I moved on to inspecting under the microscope and testing for shorts.

![1](Media/Build%20Log/IMG_20240701_175342.jpg)
![1](Media/Build%20Log/IMG_20240701_180736.jpg)
![1](Media/Build%20Log/IMG_20240701_181505.jpg)
![1](Media/Build%20Log/IMG_20240701_195700.jpg)

## July 2nd, 2024
Today I assembled the top side of my control board, as well as the encoder board.

Pasting

![1](Media/Build%20Log/IMG_20240702_122101.jpg)

The layer is a bit thick, but it should work

![1](Media/Build%20Log/IMG_20240702_124521.jpg)

Added all the capacitors

![1](Media/Build%20Log/IMG_20240702_134653.jpg)

Added all the resistors

![1](Media/Build%20Log/IMG_20240702_135552.jpg)
![1](Media/Build%20Log/IMG_20240702_141529.jpg)

Started adding ICs

![1](Media/Build%20Log/IMG_20240702_142909.jpg)
![1](Media/Build%20Log/IMG_20240702_144345.jpg)
![1](Media/Build%20Log/IMG_20240702_144721.jpg)

I realized at this point I bought the wrong mosfet package size. I had an idea for a quick fix though.

![1](Media/Build%20Log/IMG_20240702_150721.jpg)

Reflow

![1](Media/Build%20Log/IMG_20240702_154831.jpg)

![1](Media/Build%20Log/IMG_20240702_160042.jpg)

Inspecting. There is definitely too much solder

![1](Media/Build%20Log/IMG_20240702_163517.jpg)

Had to add some extra solder on the mosfets.

![1](Media/Build%20Log/IMG_20240702_174821.jpg)

I realized I put the wrong resistors down for the leds, so I replaced them.

![1](Media/Build%20Log/IMG_20240702_190624.jpg)

The big capacitor wouldn't solder to the ground plane, so I'm heating the board up to make it easier

![1](Media/Build%20Log/IMG_20240702_200538.jpg)
![1](Media/Build%20Log/IMG_20240702_204344.jpg)

Doing the magnetic encoder board here.

![1](Media/Build%20Log/IMG_20240702_202758.jpg)
![1](Media/Build%20Log/IMG_20240702_204916.jpg)
![1](Media/Build%20Log/IMG_20240702_205138.jpg)
![1](Media/Build%20Log/IMG_20240702_211012.jpg)
![1](Media/Build%20Log/IMG_20240702_212432.jpg)
![1](Media/Build%20Log/IMG_20240702_213016.jpg)

## July 3rd, 2024
Today I am mostly soldering wires to the test points and the power inputs.

![1](Media/Build%20Log/IMG_20240703_153124.jpg)
![1](Media/Build%20Log/IMG_20240703_153828.jpg)
![1](Media/Build%20Log/IMG_20240703_160351.jpg)
![1](Media/Build%20Log/IMG_20240703_161011.jpg)
![1](Media/Build%20Log/IMG_20240703_162940.jpg)
![1](Media/Build%20Log/IMG_20240703_164506.jpg)

Accidentally misplaced a component

![1](Media/Build%20Log/IMG_20240703_165514.jpg)

Fixed it now.

![1](Media/Build%20Log/IMG_20240703_170848.jpg)
![1](Media/Build%20Log/IMG_20240703_172419.jpg)

All ready for blast testing!

![1](Media/Build%20Log/IMG_20240703_201545.jpg)

It did not blow up when I applied power, but unfortunately it didn't do anything. For some reason the power protection chip is failing and not opening up the mosfets. I tested manually by appling a small current to the mosfets, which opened up and let power through the whole board. I think I will have to bypass the circuit to continue testing, because visually I don't see anything wrong. There may be a short hidden underneath the chip, or the chip may be faulty.

## July 5th, 2024

I spent most of my afternoon debugging. I have identified two more design flaws. Since I'm using 3x pwm mode for the driver, I should have tied the unused pins to 3.3v, and not left them floating! I found this in the datasheet. To fix on my board, I have to solder a little wire to several leads on the QFN package, which is a hassle. I also can't get a 3.3volt power output from the driver chip, and discovered a recursive design flaw causing this. The driver chip has an enable pin with an internal weak pulldown. I put a pull-up resistor on the outside, to always keep the chip on. Guess what that is connected to though? The regulator output from the chip, which is disabled! :skull:

BTW I added a file for [Design Errata](Design%20Errata.md)

## July 6th, 2024

I managed to solder on the very difficult bodge wire needed to the QFN chip. This ties the three lowside drive pins to 3.3 volts
![1](Media/Build%20Log/IMG_20240706_152508.jpg)
![1](Media/Build%20Log/IMG_20240706_153545.jpg)

I also fixed the design errors I identified yesterday in the kicad files.

## July 7th, 2024

I finally figured out how to add custom programmers to arduino IDE! I wanted to use my Flipper zero with the DAP-link application to program my board, so I looked through the arduino manual and through some of the other programmers to figure out how to do it. There is a file called `programmers.txt` in every installed boards file definition, which in my case was in `.arduino15/packages/STMicroelectronics/hardware/stm32/2.8.0/` (I installed stm32 support from https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json) The only supported programmer was the stlink, so I added my own flipper zero support using openocd like I found in other examples:

```txt
flipper.name=Flipper Zero DAP-link
flipper.communication=USB
flipper.protocol=cmsis-dap.cfg
flipper.program.tool=openocd
flipper.program.tool.default=openocd
flipper.extra_params=-c "transport select swd"
flipper.extra_params=-c "adapter speed 9600"
```

I had to delete `.config/arduino-ide` for it to actually show up, because there were temporary files stored there. This may be different depending on platform, see https://support.arduino.cc/hc/en-us/articles/4415103213714-Find-sketches-libraries-board-cores-and-other-files-on-your-computer User Data folder.

I was finally able to program the basic Blink.ino! It works!

Now I'm trying to setup the clock usage. For some reason my blink code is taking up 40k bytes! Over head with absolutely no code is around 4.6k. The clock settings appear to get the clock to around 32 mhz, not 64. I think I set the wrong clock divisor...

I also tried to setup serial out to print hello world. I failed at wiring several times, but finally got it to print gibberish (Which means the baud rate is wrong). For some reason the serial baud rate on the computer has to be twice that what was configured in code. Clock issues?

I eventually worked it out. I had to specify in `hal_conf_extra.h` this: `#define HSE_VALUE    (16000000UL)` This told the compiler what the external clock frequency I was using. After that I had no serial timing issues and got the full 64mhz.

## July 16th, 2024

Today I finished out the basic hardware testing (except the battery charging, I don't have a battery yet), and moved on to setting up SimpleFOC. I have verified that the digital pins work, and the SPI interface works.

I also started work on a custom MA735 library, to work with the specific registers

## July 17th, 2024

I finished MA735 driver support, so now I can set resolution or update time. I can of course read the position of the magnet.

I also started work on motor setup.

I had to measure resistance between two leads to calculate phase resistance, then divided by 2 since it is a Wye/star configuration. I pretty sure of the configuration since I see a little wire tail going nowhere.

![](Media/Build%20Log/IMG_20240717_123356.jpg)
![](Media/Build%20Log/IMG_20240717_123846.jpg)

Phase resistance ended up being 0.2 ohms! Really low, but that is to be expected.

My temporary motor mount:

![](Media/Build%20Log/IMG_20240717_140932.jpg)
![](Media/Build%20Log/IMG_20240717_140954.jpg)

It definitely unorthodox, but it works to hold the motor in place while running it. The prop guard shown here was for a motor twice the size. Soon, hopefully, I will have a gearbox setup 3D printed that will be long term.


## July 18th, 2024

I spent most of the day debugging the motor. Nothing was working. I did get an encoder mount though.
![](Media/Build%20Log/20240718_224407.jpg)

## July 19th, 2024

I finally found the fix! I tested the output pins and pin C was misbehaving, outputting double the frequency! This meant that my bodge fix was not connected to the lowside pin c. I spent two hours today trying to fix that.

![](Media/Build%20Log/20240719_111950.jpg)
![](Media/Build%20Log/20240719_112709.jpg)

It worked!

Another thing I discovered today was a bug in my code that cause sensor alignment to fail! I needed to call motor.init() AFTER all the config stuff, not inbetween!

## July 20-21, 2024

I tuned the velocity PID loop on the motor, reaching around 800 rad/s, or 7600 rpm. Its fast and loud, but nowhere near the rated motor speed. When its in openloop I can get to 1140 rad/s in little increments before the motor phases out. I'm wondering honestly if the MCU is the bottleneck, its only clocked at 64mhz and it doesn't have an FPU.


## July 22-24, 2024

I spent a few hours adding wires to my board to test out other MCUS to see if I could reach higher speeds. This is an adafruit feather with a stm32f405 processor, at 168mhz.
![](Media/Build%20Log/IMG_20240723_142727.jpg)

After tuning the FOC current control loop (something I couldn't do on the built in either) I was able to push the motor way higher than my previous limit! 2100rad/s, or 20k RPM!!!. I now have reached the manufacture limit value of 19kv, since I am running a 2.5 amp current limit at 1 volt, and have reached 20krpm. I tested again with the onbaord mcu, and it could only reach around 600rad/s.

I did some testing of instantanious motor speeds, and have concluded that 1,700 is the max speed it can go from a stand still.


## July 25-31, 2024

I designed a 16 to 1 cycloidal gearbox to go on my motor. Its rather complex.


![](Media/cycloidal%20gearbox%20inside.png)
![](Media/cycloidal%20gearbox%20inside%20top.png)
![](Media/cycloidal%20gearbox%20outside.png)


## Aug 1-4, 2024
I have printed several revisions of the gearbox in resin, tweeking some tolerances and such. The parts are very fragile with the resin I have. I can't print engineering resin due to the more toxic fumes in my house, so I will have to outsource it when the time comes.

![](Media/Build%20Log/IMG_20240802_202443.jpg)
![](Media/Build%20Log/IMG_20240804_110446.jpg)

One thing to note is that I really did not plan very well in advance with the magnetic encoder board. I soldered all the wires before assemble, and didn't realize that I would have to feed the wires through the cycloidal disc. The base part has slots to insert the wires in after soldering, but the cycloidal disc is continous. I had to cut the disc to get the wires in. My next board revision will have a flex circuit with a built in FPC cable that can be fed through easily.