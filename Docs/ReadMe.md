There are 3 XH connector and 1 KF128 terminal on the PCB.
J1 (KF128) is used for power supply. It is suggested to use Litium-ion battery with a boost converter for power supply. Using adapter for power supply, could lead to instability in measuring capacitance operation.
`J4` connector is connected to `PCF8574` module (in the back 1602 LCD). Name of each pin is written beside.
`SW1` connector, connects to the push button. Important notice is middle pin is not used and it should be removed (with a plier) from male connector.
`J5` terminal is connected to measuring probes.

# usage, calibration and first uses

This PCB have 3 modes for measuring:
- Frequency
- Capacitance
- Inductor

After turn on, the deafult mode is Frequency meter. with pressing button, it will go to next mode (Capacitor) and with another press it will go to next (Inductance) and this cycle will will continue.
If in either Capacitor or Inductor mode, if the button is hold for some seconds, PCB goes into calibration mode.

## Calibrating Capacitor Meter
Put the PCB into Capacitance mode. Press and hold the button for some seconds. When LCD is wiped out, you can release the button. the message `Short Probes and Press Button` will appear on the LCD which you should short-circuit the measuring probes and then press the button.
then `Open Probes and Press Button` message will appear which you have to separate the probes and press the buttom again. then the message `Calibration Success` is shown and it means the successfull calibration.

## Calibrating Inductor Meter
Put the PCB into Capacitance mode and short circuit the probes.
Then press and hold the button, untill the message `Calibrated` is shown which means successfully calibrated.​
​
