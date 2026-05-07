# Self Balancing Cube

This is my self-balancing cube. 

Many people have built self-balancing cubes before. Mine is special
primarily in that it's mine, but there are a few goals I'm going for:

- Entirely 3D printable with the exception of fasteners and electrical
  components.
- All computing will be done on a single microcontroller. No external
  motor driver controllers.

I have been trying to get into embedded development and robotics as a
hobby and have become fascinated with the question of how much can be done
with a single chip. You don't see too many cheap, triple-drive FOC motor 
platforms. This is my attempt at making one.

## Status

A video is worth a thousand words:

https://github.com/user-attachments/assets/a72698bf-f9be-4417-983d-0745a108c254

## Features

- It balances.
- It rejects disturbances and compensates for bias in the physical apparatus.

## Architecture/BOM

- Driven by one STM32G474
- Motors: UAngel 4108 380kV brushless motors
- Motor Driver: Custom motor driver channels based on TI DRV8304 gate driver and TI NexFET MOSFETs
- Encoders: AS5048A
- IMU: MPU6050

## Firmware Design

The firmware uses ArduinoFOC to do commutation, but this is not an Arduino 
project. I wrote a very thing Arduino compatibility layer to allow building
ArduinoFOC, but other than that it's just a plain STM32CubeMX-based C++ project.

I decided not to use ArduinoFOC in its native form because I felt that the G474 
would not be able to perform ~30k FOC calculations per second with the default
ArduinoFOC drivers, especially the SPI driver which requires multiple interrupts
and/or blocking code to communicate with the AS5048A.

I tried to take maximum advantage of the G474's timers and other peripherals to
minimize the number of interrupts and the amount of CPU time spent on FOC. The
peripheral organization works like this:

- The motor is driven by TIM1.
- TIM1's update event triggers ADC1 and ADC2 to simultaneously read the voltage
  levels on the current sensing channels.
- TIM1's update event also triggers TIM2, which times the SPI transaction.
- TIM2's OC1 serves as the CSn output for the AS5048A.
- TIM2's OC2 DMA sends the read commands to the SPI data register.
- The only interrupt or CPU involvement in the entire FOC loop is the SPI completion
  interrupt, during which we perform the FOC calculations and set up for the next
  interrupt.

In addition to the above, I spent some time optimizing ArduinoFOC and got my FOC
loop time down to about 9us, from a starting point of more than 15us. With a 9us
FOC time, we can do commutation for three motors at 10kHZ, and still have plenty
of time left over for the balancing code.

The balancing code follows the original ETH paper closely. It uses LQR and automatic
trimming to calculate the true upright position of the apparatus. Unlike the ETH paper,
we don't use two separate accelerometers, we just use the single IMU to estimate
attitude.

We do not have and do not plan a "stand up" mechanism. I'm afraid the 3D printed parts
will not take the impact.

## Physical Architecture

The current 3D parts can be found in [this OnShape document](https://cad.onshape.com/documents/f8f2dedb655a8f8eadaf2869/w/6735490cac310b34c3954fd4/e/a6eb03da97210c9f0776929d?renderMode=0&uiState=69fcb1fc7f5177c345e15b18).
There's not much to say about these parts except that I had interesting challenges in
the first version of the apparatus. I am not a mechanical engineer and designed a
test bench that could easily rack against itself and introduced tons of IMU noise. The
motor and its opposite side bearing mount need to be rigidly connected to prevent this.
The part I have there currently is overkill and would be too much plastic for the actual
final design, but the principle is sound. Learn from my mistakes!

## Plans

During the next few months, between other projects, I hope to design the final PCB that
incorporates all three motor channels and the final cube apparatus.
