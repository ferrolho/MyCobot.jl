# MyCobot.jl

MyCobot.jl is a Julia package for the [myCobot 280 (for Arduino)][product-en] robotic arm. It provides a high-level interface to control the robotic arm using the [Julia](https://julialang.org/) programming language. The package is built on top of [LibSerialPort.jl](https://github.com/JuliaIO/LibSerialPort.jl) for serial communication with the robot.

> [!WARNING]
> This package is a work in progress and is not yet feature-complete.

## Installation

First, install Julia by following the instructions on the [official website](https://julialang.org/downloads/).

Then, MyCobot.jl can be installed with the Julia package manager.

From the Julia REPL, type `]` to enter the Pkg REPL mode and run:
```julia
(@v1.11) pkg> add https://github.com/ferrolho/MyCobot.jl
```

Or, equivalently, via the Pkg API:
```julia
julia> import Pkg; Pkg.add("https://github.com/ferrolho/MyCobot.jl")
```

## Highlights

https://github.com/user-attachments/assets/d0974d83-21ab-439b-a25e-b0ce1fc81bdb

**Video 1.** Demo showing the real robot on the left and the robot model visualisation on the right. The visualisation is in real time and so when the robot joints are backdriven, the model moves accordingly.

## Resources

- Product page
  - myCobot 280 (for Arduino): [Chinese][product-cn] • [English][product-en]
  - ATOM Matrix ESP32 Dev Kit: [M5Stack Shop][m5stack-atom-matrix]
- Documentation
  - GitBook (Homepage): [Chinese][gitbook-cn] • [English][gitbook-en]
  - Communication Protocol: [Chinese][protocol-cn] • [English][protocol-en]

[product-cn]: https://www.elephantrobotics.com/mycobot-280-arduino-2023/
[product-en]: https://www.elephantrobotics.com/en/mycobot-280arduino-en

[gitbook-cn]: https://docs.elephantrobotics.com/docs/mycobot_280_ar_cn
[gitbook-en]: https://docs.elephantrobotics.com/docs/gitbook-en

[protocol-cn]: https://docs.elephantrobotics.com/docs/mycobot_280_ar_cn/3-FunctionsAndApplications/6.developmentGuide/CommunicationProtocolPackage/18-communication.html
[protocol-en]: https://docs.elephantrobotics.com/docs/gitbook-en/18-communication/18-communication.html

[m5stack-atom-matrix]: https://shop.m5stack.com/products/atom-matrix-esp32-development-kit
