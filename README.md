<h1 align="center">Feather</h1> 
  <p align="center">
    A lightweight head for ADS-B receivers
    <br/><br/>
    [<a href="https://github.com/Haskili/Feather#Features">Features</a>]
    [<a href="https://github.com/Haskili/Feather#acknowledgements">Acknowledgements</a>]
    [<a href="https://github.com/Haskili/Feather/issues">Issues</a>]
    <br/><br/><br/>
    <img src = "https://static.vecteezy.com/system/resources/previews/020/661/956/non_2x/plane-logo-icon-illustration-design-vector.jpg" alt ="" width="20%" height="20%">
  </p>
</p>

## Overview
Feather is a lightweight [Automatic Dependent Surveillance–Broadcast (ADS-B)](https://en.wikipedia.org/wiki/Automatic_Dependent_Surveillance%E2%80%93Broadcast) message decoding head, originally designed as a "personal education project".

The project is made with resource-constrained systems in mind, providing efficient decoding of Mode-S DF17 messages with minimal resource usage.

## Features

- Compact memory footprint
- Portable C++ implementation
- Suitable for real-time embedded applications
- Prebuilt to listen for backbones such as `dump1090_rs`

## Supported Message Types

Current supported message includes:

- [x] Aircraft Identification
- [x] Airborne Position
- [ ] Surface Position
- [x] Airborne Velocity
- [ ] Operational Status

## Roadmap

- [x] CRC verification
- [ ] Surface Position message handling
- [ ] Operational Status message handling
- [ ] Local CPR
- [x] Refactor for better compartmentalization
- [ ] NIC & NAC
- [x] Aircraft Category implementation
- [ ] Unit tests

## Requirements
In terms of requirements, the only thing needed is an ADS-B AVR source as either a socket or as a file.

If you're brand new and not sure what hardware you should get, all you need is the following:
- SDR Dongle ([example](https://www.amazon.com/RTL-SDR-Blog-RTL2832U-Software-Defined/dp/B0CD7558GT))
- Compatible Antenne ([example](https://www.amazon.com/Flightaware-Fiberglass-Extension-Increase-Catching/dp/B0CWQ68DZP))
- Backbone (e.g. [`dump1090_rs`](https://github.com/rsadsb/dump1090_rs))

<br/><br/>
<img src = "https://m.media-amazon.com/images/I/71ulbD0GdUL._AC_SX466_.jpg" alt ="" width="20%" height="20%">
<br/><br/>

## Installation

Clone the repository:

`git clone https://github.com/haskili/feather.git`

Include the library in your project and build on it using your preferred ADS-B toolchain.

## Acknowledgements

This project is made possible by the many amazing online resources such as:

- ["The 1090 Megahertz Riddle"](https://mode-s.org/1090mhz/misc/preface.html) by Junzi Sun (the "pyModeS" author)
- ["Mode S Packet Decoder"](http://jasonplayne.com:8080/#)
- Live traffic maps like [the one over at theairtraffic.com](https://globe.theairtraffic.com/)
- ["ADS-B Guide"](https://blog.exploit.org/ads-b-guide-demodulation-and-decoding/) by "Sterva"

As a concluding note, this project is a work-in-progress and I am by no means an expert. That said, if you see something incorrect please drop a message in the Issues and I will get to it ASAP.