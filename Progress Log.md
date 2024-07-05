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
