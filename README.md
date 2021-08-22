# Arduino Smart Bike Trainer

The goal of this project is to develop a smart indoor trainer starting from the Tacx Flow (T1680) trainer combined with a Arduino Nano 33 BLE using the FiTness Machine Service profile.

[https://github.com/krisc-informatica/arduino-csc-bike-trainer](https://github.com/krisc-informatica/arduino-csc-bike-trainer) implements the CSC Profile (Cycling Speed and Cadence) as it appears to be incompatible with the FTMS profile when advertised in the same code. For example, the BKOOL software was working incorrectly when receiving both service data.

Documentation also on [GitHub pages](https://krisc-informatica.github.io/arduino-smart-bike-trainer)

## The trainer cable
According to [https://hackaday.io/project/164276-tacx-flow-ant-conversion](https://hackaday.io/project/164276-tacx-flow-ant-conversion) the cable layout of the RJ10 connector on the Tacx Flow is as follows:
1. Cadence signal to computer (one pulse per crank revolution), 3.3V pulses, pulled high (no pulse => 3.3V)
2. GND/Common.
3. PWM control signal to brake (2.6V pulses).
4. AC power / synchronization signal to computer, ~23V AC signal which appears the +ve part is clipped at ~19.5V (see picture).
5. +18V (~1.5V variation sawtooth profile).
6. Speed signal to computer (4 pulses per brake axle revolution) 3.3V pulses, pulled high (no pulse => 3.3V)

## BLE information that might be useful

* BLE Base UUID: 00000000-0000-1000-8000-00805F9B34FB
  * 16-bit UUID to 128-bit: 0000xxxx-0000-1000-8000-00805F9B34FB
  * 32-bit UUID to 128-bit: xxxxxxxx-0000-1000-8000-00805F9B34FB

* FTMS (Fitness Machine Service): 0x1826
  * Characteristics:
    * Fitness Machine Feature: 0x2ACC
      READ: Fitness Machine Features (32bit) and Target Setting Feature (32bit)
    * Indoor Bike Data: 0x2AD2
      NOTIFY (1/sec)
    * Training Status: 0x2AD3
      NOTIFY (on change) | READ
    * Supported Resistance Level Range: 0x2AD6
      READ
    * Fitness Machine Control Point: 0x2AD9
      WRITE
    * Fitness Machine Status: 0x2ADA
