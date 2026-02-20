GOAL: Control a EZ Kontrol motor using an RC transmitter by directly reading receiver input and generating servo PWM output using arduino, without involving a Raspberry Pi.

System Architecture: 
RC Transmitter
      ↓
RP4TD Receiver
      ↓
Arduino MEGA
      ↓
EZ Kontrol Motor


Component,Wire Color,Arduino Pin,Function
RP3 Receiver TX,Yellow/White,Pin 2,CRSF Signal In
A48 Throttle,Green/White,Pin 9,0-5V Signal Out
A48 Reverse,Blue/Yellow,Pin 11,Direction Toggle
Common GND,Black,GND,Signal Ground Reference
