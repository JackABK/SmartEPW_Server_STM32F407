SmartEPW
=========

We are now developing a electric-powered wheelchair(EPW) for quadriplegic with high level spinal cord injury. We are going to modify popular EPW whose structure will be changed to fit the subject’s residual functions. 



Face a problem
===============
* For the quadriplegic with high level spinal cord injury case, he cannot use the joystick to control the movement of   EPW.

* Analyzing the signal from the joystick to the driver, we find that we need the                                      [Shark-Dynamic Controls](http://www.dynamiccontrols.com/dealers/products/shark) support to give us the protocol to    control the EPW, the signal input to the driver is a digit signal with duration modulation and it is very easy to    operate if we can get the protocol, but the Shark has yet to anything respond to us.


Resolution
===========

* We could not use the Shark's driver and joystick, just buy the universal driver, and only given the analog        voltage, so we using pwm signal complete the basic control.

* Because our case cannot use the joystick to control EPW, so we use the mouth control switch through morse code       (developed over the past), and display to the tablet PC or smart phone based on android system.

* The way to trasfer control and sensors signal is through the wireless bluetooth, but we prefer to using wifi by      future work. 



STM32F4-Discovery-FreeRTOS
============================

* This project is based on A demo of FreeRTOS running on a STM32F4 Discovery                                           board([STM32F4-FreeRTOS](https://github.com/wangyeee/STM32F4-FreeRTOS)).
