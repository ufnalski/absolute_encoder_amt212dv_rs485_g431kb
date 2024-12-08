# Absolute encoder with RS-485 interface  (STM32G431KB)
An STM32 HAL example of communicating with an absolute encoder over the RS-485 interface. A multi-turn 14-bit encoder from Same Sky (formerly/rebranded CUI Devices) is taken as an example. The relevant evaluation kit is [AMT212D-V (2 Mbps data rate)](https://www.sameskydevices.com/product/motion-and-control/rotary-encoders/absolute/modular/amt21-series). A simple blocking mode is implemented.

![Same Sky AMT21 in action](/Assets/Images/same_sky_rs485_in_action.jpg)

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Hardware
* [RS485 Board (3.3V)](https://www.waveshare.com/wiki/RS485_Board_(3.3V)) (Waveshare)

# Tools
* [DSLogic Plus](https://www.dreamsourcelab.com/product/dslogic-series/) or any other logic analyzer capable of sampling at 16+ MHz.
* [KAmod USB RS485 ISO - Konwerter USB - RS485 z izolacją galwaniczną](https://kamami.pl/konwertery-rs485/1187349-kamod-usb-rs485-iso-konwerter-usb-rs485-z-izolacja-galwaniczna-5906623433391.html) (KAMAMI)
* [YAT - Yet Another Terminal :: Serial Communication :: Engineer/Test/Debug](https://sourceforge.net/projects/y-a-terminal/)
* [Tabby - a terminal for the modern age](https://tabby.sh/)
* [AMT programming cable AMT-PGRM-06C](https://www.sameskydevices.com/product/motion-and-control/rotary-encoders/encoder-accessories/amt-cables/amt-pgrm-06c) (Same Sky)
* [AMT Viewpoint](https://www.sameskydevices.com/amt-viewpoint) (Same Sky)

![DSLogic Plus in action](/Assets/Images/rs485_logic_analyzer.JPG)

![Tabby in action](/Assets/Images/rs485_raw_data_tabby.JPG)

# RS-485
* [RS-485](https://en.wikipedia.org/wiki/RS-485) (Wikipedia)
* [What is RS-485?](https://www.youtube.com/watch?v=bt9Px51eP6s) (Texas Instruments)
* [One Minute RS-485 Introduction](https://www.youtube.com/watch?v=kX3GuNlXKKw) (Texas Instruments)
* [SparkFun According to Pete #54 - How RS-485 Works](https://www.youtube.com/watch?v=9NJVs3_g_PY) (SparkFun Electronics)
* [CAN vs. RS-485: What's the Difference?](https://www.youtube.com/watch?v=lRkIdzsLiFk) (maxim integrated)
* [RS485 - everything you need to know but didn't know who to ask!](https://www.youtube.com/watch?v=1STXlV1CvWs) (tmf mikro)
* [How to Interface RS485 Module with STM32](https://www.youtube.com/watch?v=GyXe9BezQmg) (ControllersTech)
* [RS-485 [RS Elektronika] #62](https://www.youtube.com/watch?v=0M2ZljGHO6Y) (RS Elektronika)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [RS Elektronika](https://www.youtube.com/@RSElektronika), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control in power electronics and drives - do try this at home :exclamation:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
