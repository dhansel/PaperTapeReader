# Paper Tape Reader

This is a DIY reader for 8-bit (9-hole) vintage paper tapes.

![FullPicture](/images/FullPicture.jpg)

You can watch a short demo video of the reader connected to an [Altair 8800 emulator](https://www.hackster.io/david-hansel/arduino-altair-8800-simulator-3594a6) here:
<div align="center">
  <a href="https://www.youtube.com/watch?v=uZ2VWKNlPQU"><img src="images/youtube.png" alt="Watch the Video"></a>
</div>

## Reader Head
When I started with this project, constructing the reader head was
the most uncertain aspect. I found some other DIY attempts on the web
but none seemed ideal. I wanted to use miniature photo-transistors that
could fit on a 0.1" grid (to match the .1" spacing of the paper tape holes).
I found the [Kingbright AM2520P3BT03](https://www.digikey.com/product-detail/en/kingbright/AM2520P3BT03/754-2338-1-ND/9647017)
IR phototransistors and wavelength-matched 
[ON Semiconductor QEB363ZR](https://www.digikey.com/product-detail/en/on-semiconductor/QEB363ZR/QEB363ZRCT-ND/3479521)
IR LEDs. Initial tests with these mounted on a perfboard were very promising,
so I made two matched PCBs ([emitter](schematics/readerhead-emitter-pcb.pdf) 
and [sensor](schematics/readerhead-sensor-pcb.pdf)) that can be sandwiched together
to form the reader head:

<p align="center"> <img src="images/ReaderHeadTop.jpg" width="200"> <img src="images/ReaderHeadBottom.jpg" width="200"> </p>
<p align="center"> <img src="images/ReaderHeadFront.jpg" width="200"> <img src="images/ReaderHeadSide.jpg" width="200"> </p>

All resistors on the two boards are 0805 SMD resistors.
Resistors on the sensor board (with the photo-transistors) are 1k Ohm.
Resistors on the emitter board (with the LEDs) are 470 Ohm except for the index-hole LED which uses 330 Ohm 
(to make the LED brighter because the holes are smaller). 
Note that these resistor values were chosen for a **3.3V supply voltage**.

Gerber files for both PCBs are in the [schematics](schematics) subdirectory.

As is the signals from the reader head can be directly attached to the digital inputs of 
any 3.3V microprocessor and should give clear and stable readings. I have tested with
black and pink paper tape.

## Main Board
In addition to the reader head I also made a main board which does the following:
* Read the (parallel) data from the reader head and send it out on a serial connection
* Control a motor to automatically run the tape at proper speed given the serial transmission rate
* Provide a menu system for setting a variety of parameters

<p align="center"><a href="images/mainboard.jpg"><img src="images/mainboard.jpg" width="400"> </a></p>

The board is centered around an Atmega328p running at 3.3V. For the menu system I used a generic
64x128 OLED display paired with a standard rotary encoder. Both are easy to find on Ebay, for the
OLED make sure that the pinout matches that on the board (many different versions exist).

There are two serial connections, one labeled "Terminal" and one labeled "Computer". This
allows the tape reader to be placed between a terminal and a computer. As long as the reader
is not actively running, all data is just passed between the "Terminal" and "Computer" serial
ports. When the reader is running, it disconnects the terminal input and sends the tape data
out to the computer instead.

## Motor Control
The reader head and main board can be used without a motor by manually pulling the tape,
however one of the main features is the ability to control the motor as seen in the video.

Motor power is varied using PWM. The PWM setting gets adjusted by a PID controller whose
parameters can be easily set using the menu system (see below).

Note that there is no direct connection built into the firmware between the serial transmission 
speed and motor speed. Instead, the PID controller varies the motor power to achieve and hold
a 10% fill state of the internal transmit buffer. Since PID parameters and basic motor parameters
are configurable via the menu system, a variety of (DC) motors should be supported.

## Menu System





