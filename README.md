GOAL: Control a EZ Kontrol motor using an RC transmitter by directly reading receiver input and generating servo PWM output using arduino, without involving a Raspberry Pi.

System Architecture: 
RC Transmitter
      ↓
RP4TD Receiver
      ↓
Arduino MEGA
      ↓
EZ Kontrol Motor


| Component | Arduino Uno Pin | EZKontrol Wire | Function |
| :--- | :--- | :--- | :--- |
| **Throttle Signal** | Pin 9 (PWM) | Green/White | Speed Control (0-5V) |
| **Reverse Signal** | Pin 11 | Blue/Yellow | Direction Toggle |
| **Receiver (TX)** | Pin 2 | Yellow | CRSF Data In |
| **Common Ground** | GND | Black | Signal Reference |

Power Domains
It is vital to keep the "Logic World" and "Power World" separate to protect your electronics.

Logic (5V): Powers the Arduino and RP3 receiver.

Power (48V+): Feeds the EZKontrol A48 and the motor.

Safety: The high-power motor current does not go through the Arduino.

Setup & Safety Checklist 

Before powering the motor, verify the following:

[ ] RP3 Power: Receiver is powered by the Arduino 5V pin.

[ ] Serial Wires: RP3 TX is connected to the Arduino RX pin.

[ ] Common GND: Arduino GND is connected to the EZKontrol logic ground.

[ ] Throttle Safety: Ensure the RC stick is at zero before powering the A48 to avoid safety lockouts.
