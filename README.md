# MCard1802TeensyPixieVideo
Teensy 3.2 based Pixie Video Simulator for the 1802 Membership Card

This code simulates a cdp1861 Pixie Video chip, using a [Teensy 3.2.](https://www.pjrc.com/store/teensy32.html)
This [simulator](https://github.com/fourstix/MCard1802TeensyPixieVideo/blob/master/docs/MCard1802TeensyPixieVideo.pdf)
uses a video ram buffer with a 128 x 64 graphics display supported by the
[U8G2 graphics library](https://github.com/olikraus/u8g2) as a video display.  The code will simulate
the interrupts, external flag 1 signal, and DMA Output requests from the original pixie video.  This
allows [programs](https://github.com/fourstix/MCard1802Arduino/blob/master/docs/Cdp1802SampleProgramCode.txt)
written for the original Cosmac Elf hardware to run directly on the simulator. This simulator supports
32 x 64 and 64 x 64 video resolutions.

The Teensy 3.2 is available from [PJRC.](https://www.pjrc.com/store/teensy32.html) and is also sold by
[Sparkfun](https://www.sparkfun.com/products/13736) and [Adafruit.](https://www.adafruit.com/product/2756)

U8G2 supports many kinds of 128 x 64 displays.  A list of supported displays is available 
[here.](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp)


For example, this [SSD1306 I2C 128 x64 OLED display](https://www.adafruit.com/product/938) available
from Adadruit works fine with the Qwiic interface and is supported by Uthe 8G2 graphics library.

Examples
---------------------
Here are some sample configurations running actual [CDP1802 programs](https://github.com/fourstix/QwiicCosmacElfSim/blob/master/docs/Cdp1802SampleProgramCode.txt).

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/Spaceship.jpg"></td> 
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802IO.JPG"></td>
  </tr>
  <tr align="center">
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Cosmac Elf Spaceship program.</td>
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Tom Pittmann's DMA Test program.</td>
  </tr>
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/Spaceship.jpg"></td> 
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802IO.JPG"></td>
  </tr>  
  <tr align="center">
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Tom Pittmann's Clock program.</td>
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Text Demo program.</td>
  </tr>
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802Outside.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802Inside.jpg"></td> 
  </tr>
  <tr align="center">
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Sprite Demo program.</td>
    <td>1802 Membership card, Teensy 3.2, OLED display and wiring.</td>
  </tr>   
</table>


Repository Contents
-------------------
* **/src/MCard1802TeensyPixieVideo/**  
  * MCard1802TeensyPixieVideo.ino -- Teensy 3.2 based Pixie Video simulator for 1802 Membership card and
  a 128 x 64 graphic display. 
* **/docs** -- documentation files
  * MCard1802TeensyPixieVideo.pdf -- schematic for Pixie Video simulation logic using a Teensy 3.2
  * Cdp1802SampleProgramCode.txt -- Sample 1802 code listings for various programs.
* **/pics** -- example pictures for readme



License Information
-------------------

This code is public domain under the MIT License, but please buy me a beer
if you use this and we meet someday (Beerware).

References to any products, programs or services do not imply
that they will be available in all countries in which their respective owner operates.

Sparkfun, the Sparkfun logo, and other Sparkfun products and services are
trademarks of the Sparkfun Electronics Corporation, in the United States,
other countries or both. 

Adafruit, the Adafruit logo, and other Adafruit products and services are
trademarks of the Adafruit Industries, in the United States,other countries or both. 

PJRC, the PJRC logo, and other PJRC products and services are
trademarks of the PJRC.com LLC, in the United States,other countries or both. 

Other company, product, or services names may be trademarks or services marks of others.

All libraries used in this code are copyright their respective authors.
  
Universal 8bit Graphics Library
Copyright (c) 2016, olikraus
All Rights Reserved

The Teensy 3.2 hardware and software
Copyright (c) 2016-2020 by Paul Stoffregen, PJRC.com LLC 
 
The 1802 Membership Card Microcomputer 
Copyright (c) 2006-2020  by Lee A. Hart.
 
Many thanks to the original authors for making their designs and code avaialble as open source.
 

This code, firmware, and software is released under the [MIT License](http://opensource.org/licenses/MIT).

The MIT License (MIT)

Copyright (c) 2020 by Gaston Williams

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.**