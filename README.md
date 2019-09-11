# Blackout Alert

## Abstract

Sense power loss at home (fluse flipped) and signalize by SMS the blackout to alert someone.

## Connection routing

![alt text](https://raw.githubusercontent.com/RindusIoTJam/Blackout-Alert/master/doc/routing.png)

1. Battery Voltage Level

   On the battery shield, connect the 180k resistor between A0 and the capacitor on the right side of D6.

2. Power Loss Sense

   On the battery shield, connect the 5.1k and 10k resistors in series and connect the 5.1k resistor to the lower diode left of D3, the 10k to GND and the divider middle to D7.

3. Alarm Piezo 

   On the proto shield, connect the piezo between GND and D6.

4. SIM800L

   On the proto shield, connect:

   - SIM800L:TX to proto-shield:D1
   - SIM800L:RX to proto-shield:D2
   - SIM800L:GND to proto-shield:GND
   - SIM800L:VCC to battery-shield:LiPo+

## Setup

Configuration can be done on the hardware serial at 115k2 8N1:

```
INFO: PRESS 'b' (b)oot reason.
            'h' show this (h)elp.
            'm' to set new alert short (m)essage.
                e.g. BLACKOUT AT HOME!
            'n' to set new alert mobile (n)umber for alert message.
                e.g. 612345678 or 00491517346592
            's' for (s)tatus information (Vbat, alert mobile number and msgs).
            't' to set send (t)est alert message.
      All other chars will be send to SIM800L (e.g. AT<CR>).
```

## BOM

1. Wemos D1 mini
2. D1 mini Lithium Battery Shield
3. LiPo Battery 300mAh
4. Resistor  10k Ohm (voltage divider power sense)
5. Resistor 5.1k Ohm (voltage divider power sense)
6. Resistor 180k Ohm (voltage divider battery voltage)
7. D1 mini ProtoBoard
8. SIM800L core breakout
9. Piezo Ceramic Wafer Plate
10. LiPo protection board (if not integrated in LiPo)

## TODO

1. Alert confirm button to turn off beeper