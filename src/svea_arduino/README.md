# SVEA-Arduino
The package contains the arduino code for the SVEA cars and various support functions.
The Arduino on the power board has three main tasks:
1. Recieve messages from the TX2 via a rosserial node and translate those into pwm-signals.
2. Read pwm-signals from the reciever and transfere the messages to the TX2.
3. Count the toicks from the wheel encoders and send the results to the TX2.
If the TX2 is not sending anything (or the remote is put into override mode by setting the switch into the forward position), the Arduino can also transmit the input from the remote directly to the pwm board without the TX2 having to be turned on.

## Quickstart
1. Turn on the remote.
2. Make sure that the LED on the reciever shines with a steady green light.
Run `roslaunch svea_arduino basic_remote_reader.launch`. If everything is turned on, installed and setup correctly, it should now be possible to control the car via the remote. The actuation values should also be published on the `/lli/ctrl_actuated` topic in ROS.
 
## Messages
The package contains some custom messages. 
### lli_ctrl
These messages contains four fields. All of them are signed 8-bit integers (can contain values between 127 and -128).
The first two fields are `steering` and `velocity`. Steering goes in a positive clockwise direction, i.e. -127 is maximum left (about 45 degrees, pi/2 radians left) and 127 is maximum right (about 45 degrees, pi/2 radians right). Velocity also goes from -127, full brake/reverse to 127, full speed ahead. The reverse behavior depends on the setting of the ESC. By default a reverse signal will first make the car brake. If a neutral (zero) signal is then sent followed by another negative signal, the car will go into reverse. 
Sending -128 to either steering or velocity will make the Arduino keep sending the last non -128 value to that channel.

The `trans_diff` control the transmision and differentials on the car. The field is treated as a series of boolean (True/False) values, where every bit has a sepparate meaning according to: 

| Bit | Usage |
| --- | ----- |
|  0  | Transmision, 0 = low gear, 1 = high gear     |
|  1  | Front differential, 0 = unlocked, 1 = locked |
|  2  | Rear differential, 0 = unlocked, 1 = locked  |
|  3  | Change transmission according to bit 0 if this is set |
|  4  | Change front differential according to bit 1 if this is set |
|  5  | Change rear differential according to bit 2 if this is set | 

Note: The bits are zero indexed. 
Bits 3, 4, and 5 are do not care bits and have the same role as -128 for steering and velocity. If these bits are set to 0, the Arduino will ignore the corresponding value. 
For example: if a message is sent with bit 0 set to 1, requesting the high gear, but bit 3 is set to 0, the transmision will remain in watever gear it was in before the message was sent. However, if bit 1 is set to 1 and bit 4 is set to 1 in the same message, the front differential will be locked, but the transmision bit will still be ignored.

Messages sent from the arduino to the TX2 will always have the do notcare bits set to 1. 

The `control` field is used to recieving (and in the future, sending) control flags from the Arduino. According to:

| Bit | Meaning |
| --- | ------- |
|  0  | TX2 detected as idle (have not sent any messages for 5 seconds) |
|  1  | Remote disconnected (No pwm signal on channel 5 from the reciever) |
|  2  | Remote in override mode |

The control field is currently only used for recieving messages and is ignored in messages sent to the Arduino.

### lli_encoder
Contains the number of ticks on the right wheel and left wheel during time delta. Note that the resolution of time delta is 10 micro seconds.
Time delta is currently the duration from the previous message being sent to the current message being sent. It will be changed to the duration
between the last tick counted in the previous message and the last tick counted in this message, and split ointo two fields, one for the left wheel and one for the right. 

## Connections
This section describeshow to conect the Arduino and everything else.

### Arduino to PWM Board
Required for control of the car.
The pwm-board is the blue Adafruit board with a large capacitor and lots of pins for connecting servos.

| Arduino pin | Connects to | Notes|
| --- | --- | --- |
| A4/SDA  | SCA pin on pwm board | i2c buss data |
| A5/SCL  | SCL pin on pwm board | i2c buss clock |
| 5V  | VCC pin on pwm board | The VCC pin should be connected to 5V, it does not have to be connected to the 5V pin on the ardunio. |
| GND | GND pin on pwm board |  The GND pin should be connected to ground, it does not have to be connected to the ground pin on the ardunio.|
| D3 | PWM pin 10 on the pwm board | Optional, used for adjusting the output frequency of the pwm board. |

The internal clock on the pwm boards varries with up to 10% between boards. If the D3 pin is connnected the arduino will attempt to meassure the output frequency of the pwm board and adjust it to be close to 99 Hz. The servos work for higher frequencies, but the ESC will not recognize the signal if it is even slightly above 100 Hz. 
The auto adjustment uses external interrupt 1 (INT1) on the arduino. The interrupt is disabled after the adjustment process is completed. 

### PWM Board to other
Required for control of the car. Unless otherwise noted all three pins from the servo should be connected to the indicated channel on the PWM board. Note that the channels are zero indexed (goes from 0 to 15).

| PWM board channel | Connects to | Notes |
| --- | --- | --- |
| PWM 15 | Steering servo |   |
| PWM 14 | ESC | If the power cable on the ESC is cut, the entire servo connector can be used. Otherwise only the ground and pwm pin should be connected. |
| PWM 13 | Gear servo |   |
| PWM 12 | Front differential servo |   |
| PWM 11 | Rear differential servo |   |
| V+ | A power source between 5 and 6.5 Volts | It is also possible to connect the power through the power terminal (preffered) or any unused power pin on a channel. Due to the high currents required the 5V pin on the Arduno should not be used. |

### Reciever
Required connections for using the remote. If the remote is not going to be used pin D2 and D12 have to be connected to ground. 
Only the pwm pin on the reciver should be connected unless otherwise noted.

| Arduino pin | Connects to | Notes |
| --- | --- | --- |
| D2  | PWM output 1 (Steering) on reciever | Used for detecting the rising edge of the pwm signal |
| D8  | PWM output 1 (Steering) on reciever | There are 2 channel 1 pins on the receiver.  Detects falling edge on channel 1. |
| D9  | PWM output 2 (ESC) on reciever | Detects falling edge on channel 2. |
| D10 | PWM output 3 (Gear) on reciever | Detects falling edge on channel 3. |
| D11 | PWM output 4 (Front ) on reciever | Detects falling edge on channel 4. |
| D12 | PWM output 5 (ESC) on reciever | Detects falling edge on channel 5. |
| 5V  | Any power in pin on the reciever | The VCC pin should be connected to 5V, it does not have to be connected to the 5V pin on the ardunio. |
| Gnd | Any ground pin on the reciever  |  The GND pin should be connected to ground, it does not have to be connected to the ground pin on the ardunio.|

The D2 pin uses external interrupt 0 (INT0) of the arduino. Pin D8 to D12 uses pin change interrupt 0 (PCINT0) and these pins are grouped on the GPIO adress PORTB. 

## Setup
Howto setup all the software inorder to make things work. This section does not include the standard ROS procedures for installing packages.
### Changing the arduino device name
Required for running the default launch scripts.
In the default launch files the Arduino's device name is `/dev/arduinoPWM`. To create this link:
1. Make sure that the control Arduino is the only Arduino connected to the TX2 on the car
2. When in the package base folder, run `sudo python gen_arduino_rule.py`. 
3. If no error messages appear the link should have been created. Check by typing `ls /dev/` and see if `/dev/arduinoPWM` is listed. 

### Seting up arduino libraries
To compile and upload the firmware to the arduino some ROS libraries have to
be created by rosserial and put in the Arduino IDE library folder.
Custom messages requires this procedure everytime their definition is changed.
First run `catkin_make` and make sure that the messages are created correctly. 
Then run `bash make_library.sh` from the `svea_arduino` directory.
(it will run `rm -r ~/Arduino/libraries/ros_lib/` followed by
`rosrun rosserial_arduino make_libraries.py /home/nvidia/Arduino/libraries/`)

## Trouble shooting

### Rosserial reports that the Arduino is not found
1. Make sure that the Arduino is connected to the TX2 throught USB.
2. Type `ls /dev/` and check that `/dev/arduinoPWM` is listed. If not: check if `ttyACM0` is listed.
3. If `ttyACM0` is listed, then see `Changing the arduino device name` under `Setup`.'

### The LED on the ESC flashes with a green light and the car will not drive forward.
The blinking green light indicates that the ESC is not recieving a correct pwm signal.
1. Check that the ESC is connected to the pwm board via the servo cable. (Note that the power cable from the ESC should not be connected to the pwm board).
2. Check that pin 10 on the pwm board is connected to pin D3 on the Arduino. If the pin is disconnected: Connect it and restart the arduino.
3. If the problem persists: Recalibrate the ESC. See the TRX4 manual page 18 under "XL-5 HV Setup Programming" for how to do this.  

### The car makes sudden sharp turns or accelerations 
1. Make sure that the recievers is correctly connected to the Arduino
2. If no reciever is present, connect pin D2 and D12 to ground. 

### Wheel encoders are registering different rates
1. Check that all cables from the sensors, to the sensor boards and 
from the sensor boards to the wheel sensors are connected.
2. Check that the left LED och each sensor board shinse with a steady light. 
If not: Check the connections to the power again. 
3. Try turning the wheels by hand. The right LED on each sensor board should blink.
   If not: Try adjusting the potentiometer on the sensor board. 
4. If both right LEDs blink, run `roslaunch svea_arduino basic_remote_reader.launch`
and start `rosrun rqt_plot rqt_plot`. configure rqt_plot to show 
`left_ticks` and `right_ticks` from the `/lli/encoder` topic. Try tweaking 
the potentiometer on the sensorboard that appears to be giving eroneous readings
while using the remote to control the speed of the wheels. 

