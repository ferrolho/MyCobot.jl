# myCobot 280 Lab

Fast, direct control of the Elephant Robotics [myCobot 280 (for Arduino)][product-en].

- **500 Hz onboard control.** Custom firmware on the ATOM plays plans at a fixed 500 Hz and streams joint state and IMU data over WiFi.
- **Direct servo-bus access.** The laptop talks to the Feetech servos through the FT232R at ~300 Hz.
- **Julia package `MyCobot`.** Kinematics with RigidBodyDynamics.jl, planners, players, iterative learning control, and the ATOM link.
- **Documentation site.** The hardware, the protocols, the firmware and every measured result.

On a 100 mm circle, lag compensation and three runs of learning control reduce the tracking error from 12.5 mm to 0.8–1.0 mm RMS.

> [!WARNING]
> This is a research project and work in progress. There is no emergency stop: read the Safety page first.

## The Julia package

Install Julia from the [official website](https://julialang.org/downloads/). Then add the package:

```julia
julia> import Pkg; Pkg.add(url="https://github.com/ferrolho/mycobot-280-lab")
```

Or work in the repository: `julia --project=.` from the repository root.

## Highlights

https://github.com/user-attachments/assets/d0974d83-21ab-439b-a25e-b0ce1fc81bdb

**Video 1.** Demo showing the real robot on the left and the robot model visualisation on the right. The visualisation is in real time and so when the robot joints are backdriven, the model moves accordingly.

## Documentation

The documentation site in `website/` is the source of truth for this project: the
hardware, the servo bus, the ATOM firmware, the Julia and Python tools, and every
measured result. To browse it locally:

```bash
cd website
npm install
npm run dev     # then open http://localhost:4321/mycobot-280-lab/
```

The site will be published at https://ferrolho.github.io/mycobot-280-lab/.

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
