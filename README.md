# OvidiusRobotOperation_stable
Stable version of Ovidius robot firmware for active and passive operation.  This research has been financially supported by General Secretariat for Research and Technology (GSRT) and the Hellenic Foundation for Research and Innovation (HFRI) (Code: 1184)

[21-2-22]

OVIDIUS PROTOTYPE ROBOT
OPERATION INSTRUCTIONS

INFO:
Active operation file is located in metaOperationMainDueDevel
Passive operation file is located in simplePseudoFirm
Operation folders must be located in the main “Arduino” folder of the PC.
The libraries folders must be located in the “Arduino/libraries”  folder of the PC.
Each operation file has instructions for using it in the beginning of the file.
Everything is stable as is, and the last task headers are loaded. This code is already UPLOADED on the DUE Board of the lab robot.

ACTIVE OPERATION INSTRUCTIONS:

It is assumed that changes in the code have been successfully made, the ino files are compiled and the 32bit-ARM Boards packet is installed from the Arduino Boards Manager. 

How to upload:
Connect the USB hub in the DC power source.
Connect the PROG USB cable of the DUE Board (Red) in a usb port of the hub
Check that the switch located in the back left corner of the DXL Shield attached on the DUE is set to Upload position.
Select the DUE Programming Port in the available boards (is found under ARM Boards) and select the corresponding port in the Arduino IDE.
Press Upload and wait until program is written in flash and verified.
When upload is complete, switch  the DXL Shield switch to the Dynamixel position.
Connect the Native USB Cable in the usb hub. Press the RESET button in the DUE (forward left corner).
Then Select the DUE Native in the available Boards prompt menu and the corresponding Native Port number in the Ports menu in Arduino IDE.
Then open Serial Monitor and the program will start.
How to operate(active mode):
Check the emergency button is NOT pressed.
Power up the main Source and check that the DXL relay switch is ON.
Open the Serial Monitor(Upload process must have been previously executed if code changes have been applied, If no changes were made then just check that DXL Shield switch is in Dynamixel position, that both USB cables are connected in the hub and select the Native Board/Port from the Arduino IDE). Once serial connection is established the program will start and prompts will appear in the Serial Monitor. 
First the program pings the connected DXLs in order to  establish communication. If DXLs are not pinged then must check if the RS485 UART cable is ok. If everything is ok, but still no connection is established, switch the main power off and come back later…
During operation only the available answers to the prompts should be given.
Instructions for operation are also provided in detail in the .ino file. At this time only predefined trajectories/ joint commands can be executed, which must be defined in the corresponding header files as described in the main active operation ino file.
During Active Operation, after some considerable time of running it was observed that some trajectories were loaded but with small number errors (i.e. in header file joint position “1.5708” was set and in the serial monitor the value “1.5507” appears). In that case, terminate operation, switch off the power supply of the DUE board only and reconnect after a few seconds.
If sensor logging is selected, then after p2p segment execution the sensor data are plotted in the Serial Monitor and the user must copy to a file in order to process the offline!

PASSIVE OPERATION INSTRUCTIONS:

The 3 pseudos currently installed in the robot structure are configured and operation-ready. For the remaining 3, the firmware must be re-installed. The necessary modifications for pseudo integration in the structure are provided in the main passive operation ino file. Also, the necessary changes depending on the pseudo gearbox type are provided and keywords are specified in order to find the necessary points that need to be altered in the library source file.

It is assumed that pseudos are appropriately connected to power/motor drivers. Before the metamorphic structure assembly the necessary motor drivers/cables must be checked. At this time only the 3 stepper drivers installed are operational (the other one has its RED light ON) and must be replaced with the available TB6600 drivers. 

The motor+gearbox coupler of the pseudos with the MOTOVARIO gearboxes (Blue) are plastic and MUST be replaced, otherwise the coupler of the first pseudojoint placed after active joint 1 will crack for larger pseudo angles! 

How to upload:
Connect the USB cable to the Nano and the usb hub. Select the Nano Board under MVR Boards and select the Booloader. In some pseudos the Old Bootloader must be selected in order to upload successfully.
Select the Port that appeared in Ports tab and open the Serial Monitor.
When connection is established the settings saved in pseudo’s EEPROM are shown and user must check if the information matches the code uploaded. If there is an ID mismatch then the code asks for a fix.
How to operate(passive mode):
Check that 3 cable pairings are connected to the pseudo before usage. First is the USB, second the 4 wire cable that connects to the stepper motor driver in the controller box beneath the robot, third is the cable of the 2-phase coils of the Nema23 stepper motor.
For safety, the user should SWITCH OFF the relay switch of the DXLs’ when in passive operation. When DXLs’ power is switched off, the TORQUE is OFF and the robot joints will start rotating until enough holding torque is provided. THIS IS BAD. So the user should manually hold the structure, in order to help the mechanical brake of the DXLs’ to lock the robot’s position. This is achieved within a second. Then the robot remains fixed and user can execute passive operation.
Before anatomy metamorphosis user MUST check that no screws are fixed, because this will result in BIG trouble. 
For the cable of the pseudo stepper motor driver connection the configuration shown in the table exists:
The cable of the coils is straightforward but you should check that 2 different phases are not connected.
There are 2ports for the 4pin cables in the pseudo electronics box. In some pseudos the one is used for SPI communication and is deprecated in this firmware version. In one of the two blue pseudos these 4 pins are used for the coils. The lower 4pin port is used for the stepper motor driver cable! 
If firmware is uploaded and all cables are plugged in accordingly, the user should just open the Serial Monitor to start passive operation.
Program starts and asks if HOMING should be executed or if the user wants to manually change the currently saved discrete position. Then the user enters “M” to start the passive operation. An integer number between 1~15 must be given. The corresponding hole for each setting is marked on the pseudo-wing element to help identify the current anatomy.
When anatomy metamorphosis is complete the program saves the new position and the user can just unplug the cables.
After anatomy metamorphosis is complete, the user should manually screw some bolts in order to remove the gear miss-locking. This is optional for all pseudos, except for the first pseudo that is installed in a fixed position after the first active joint. The first pseudo must also be manually screwed-locked after anatomy metamorphosis is complete.

Table 1. Stepper Motor Cable Pins
GND | D6 - ENABL | D7 - ROT DIR  | D8 - PULSE STEP
