# ch32v003-charlieplex
Drive 12 individual LEDs from an 8-pin 10-cent MCU using 63-levels of software PWM

See http://www.technoblogy.com/show?2H0K for a more detailled description of the technique

My approach uses a slightly higher level of abstraction than the linked article, which offers more flexibility in connecting up LEDs. The target mcu has a lot more resources to work with, compared to something like an attiny. 

Uses the excellent ch32v003fun environment from https://github.com/cnlohr/ch32v003fun
