# FreeRTOS_ATmega238
This demonstration showcases the integration of the FreeRTOS real-time operating system with the ATmega328 microcontroller, 
commonly found in Arduino Uno boards. The demo highlights multitasking capabilities by running multiple concurrent tasks such as:

* LED Blinking Tasks: Multiple tasks toggle LEDs at different intervals to demonstrate task scheduling.

The demo uses preemptive scheduling, allowing tasks with higher priority to interrupt lower-priority ones. 
It is configured using FreeRTOSConfig.h, where parameters like tick rate, heap size, and task priorities are defined.

![image](https://github.com/pcprusti/FreeRTOS_ATmega238/blob/main/vLED1FlashTask_Period.png)

![image](https://github.com/pcprusti/FreeRTOS_ATmega238/blob/main/vLED2FlashTask_Period.png)

![image](https://github.com/pcprusti/FreeRTOS_ATmega238/blob/main/vLED3FlashTask_Period.png)

![image](https://github.com/pcprusti/FreeRTOS_ATmega238/blob/main/vLED4FlashTask_Period.png)

![image](https://github.com/pcprusti/FreeRTOS_ATmega238/blob/main/vLED5FlashTask_Period.png)
