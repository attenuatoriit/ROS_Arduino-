# ROS_Arduino-
FOllowing are the codes to write in terminal of linux. We need to write these only after closing the serial monitor after uploading the code to arduino. ARduino IDE is supposed to be kept open. 
Start the ROS Master - roscore
Run rosserial client on the machine - rosrun rosserial_python serial_node.py /dev/tty<USB# or ACM#>
The serial port is determined at run time for either ttyUSB or ttyACM. The exact port number can be found from the Arduino IDE or using dmesg | grep tty.
Read published data returning if the button is pressed on the Arduino board or not - rostopic echo button_press.After this start rotating the motor to get the data on the terminal.
I f you want to publish data from terminal to arduino folow the link https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros
