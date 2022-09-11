# BionX32  stm32 controller for BionX ebike motor

The BionX motor has originally a built-in controller. This project aims to replace embedde electronics by an external controller.
Regular 24V controller with square waves doesn't provide good enough results, this is a new attempt with a FOC implementation.

## Project Content

3 phases output,

3 hall sensors input,

One Shunt current measurement,

Interface for S866 Display.

PAS input,

Brake input


Core code is generated thanks to ST micro graphical tools. (X-CUBE-MCSDK)
then adapted to STM32F103C8Tx

## Hardware description

STM32 module
MOSFET module
Main pcb
S866 display

## Credits  & Resources
STM32CubeIDE

STM32 MC SDK (motor control software development kit)
