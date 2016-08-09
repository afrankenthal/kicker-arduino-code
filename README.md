# kicker-arduino-code

This package contains the Arduino code for both the Intel Galileo and the Arduino Due boards in the E989 kicker control and monitoring system. The remaining piece of the code, the webserver user interface, can be found at [kicker-server](https://github.com/afrankenthal/kicker-server/). For a detailed description of the entire system please refer to the [User Guide](docs/kickerM&CUserGuide.pdf). Schematics of circuits can be found in [schematics](schematics/).

Note the Arduino code contained here should already be uploaded to the boards and therefore if you simply want to use the existing boards, you don't need this repository at all. This will be useful only if you want to develop the code further or upload it to new boards, or to troubleshoot existing boards.

### Requirements

If you want to do further development to this code then you will need a few tools:

- Arduino IDE 1.6.0 for Intel Galileo: this special version of the Arduino IDE was modified by Intel to be able to communicate with Intel boards such as Galileo. There are other compatible versions, but this version (1.6.0) was tested successfully. Link: [Arduino IDE 1.6.0 for Mac OS X](http://arduino.cc/download_handler.php?f=/arduino-1.6.0-macosx.zip).
- Arduino IDE 1.6.7: this version of the IDE was used for the development of the Arduino Due code. Earlier versions of the IDE (like the 1.6.0 above) don't support the Due, and that's why we needed two independent versions for development. Probably later versions than 1.6.7 will also work. Link: [Arduino IDE 1.6.7 for Mac OS X](http://arduino.cc/download_handler.php?f=/arduino-1.6.7-macosx.zip).
- Due libraries inside the Arduino IDE: Since the Arduino Due uses a different microcontroller than AVR (Atmel SAM series), extra libraries are required to compile programs for the Due. The installation is very straightforward, however, and can be done from within the IDE. Instructions can be found here: [Arduino Due Core](https://www.arduino.cc/en/Guide/Cores)

### Installation Instructions

After the IDEs and the Due core in 1.6.7 are installed, copy the [kickerControlArduinoV2](kickerControlArduinoV2/) and the [kickerControlGalileoV2](kickerControlGalileoV2/) folders to your `~/Documents/Arduino` folder. Also copy the libraries in [libs](libs/) to the `~/Documents/Arduino/libraries` folder. Connect the USB programming port of the Arduino Due to the computer, and the client USB port of the Intel Galileo to the computer. Make sure you know which port is which in the two IDEs. Open the respective files in each IDE and choose the appropriate board and appropriate port for each, in the menu on top (Tools -> Board and Tools -> Port). To upload the code to the boards, simply click on the right arrow button on top of the code window. After the code has been uploaded, go to Tools -> Serial Monitor (or hit Shift-Ctrl-M) on each IDE to open the Serial monitor and see the messages sent via that Serial interface to the computer.
