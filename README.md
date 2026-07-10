# Feather

Feather is a lightweight ADS-B AVR message decoder designed as a small personal projet.

The project is made for resource-constrained systems, providing efficient decoding of Mode-S DF17 messages with minimal resource usage.

### 🚧 Work in progress

## Features

- Compact memory footprint
- Portable C++ implementation
- Suitable for real-time embedded applications
- Prebuilt to listen for backbones such as `dump1090_rs`

## Supported Messages

Current supported message includes:

- [x] Aircraft Identification
- [x] Airborne Position
- [ ] Surface Position
- [x] Airborne Velocity
- [ ] Operational Status

## Installation

Clone the repository:

`git clone https://github.com/haskili/feather.git`

Include the library in your project and build on it using your preferred ADS-B toolchain.

## Acknowledgements

This project is made possible by the many amazing online resources such as:

- ["The 1090 Megahertz Riddle"](https://mode-s.org/1090mhz/misc/preface.html) by Junzi Sun (the "pyModeS" author)
- ["Mode S Packet Decoder"](http://jasonplayne.com:8080/#)
- [Live traffic maps](https://globe.theairtraffic.com/) like the one over at theairtraffic.com
- ["ADS-B Guide"](https://blog.exploit.org/ads-b-guide-demodulation-and-decoding/) by "Sterva"