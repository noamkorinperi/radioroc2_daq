# radioroc2_daq

Development and Implementation of Algorithms for Expanding Sensors Measurement Range  
Fourth Year Engineering Project — Ben-Gurion University of the Negev

## System Overview

Gamma-ray detection system using CsI(Tl) scintillation crystals coupled to SiPM sensors, read out via the RADIOROC2 ASIC, controlled by an STM32F722ZE microcontroller.

The system implements real-time thermal compensation to correct for:
- SiPM gain drift with temperature (hardware feedback loop via InDAC)
- CsI(Tl) light yield variation with temperature (software polynomial correction)

## Repository Structure

```
radioroc2_daq/
├── Core/
│   ├── Inc/
│   │   ├── config.h          — central configuration (edit before build)
│   │   ├── channel_map.h
│   │   ├── radioroc2_i2c.h
│   │   ├── timers.h
│   │   ├── system_init.h
│   │   ├── daq.h
│   │   ├── compensation.h
│   │   └── usb_comm.h
│   └── Src/
│       ├── main.c            — CubeMX generated + app_init/app_run calls
│       ├── channel_map.c
│       ├── radioroc2_i2c.c
│       ├── timers.c
│       ├── system_init.c
│       ├── daq.c
│       ├── compensation.c
│       └── usb_comm.c
└── python/
    ├── main.py               — entry point
    ├── comms.py              — USB CDC communication
    ├── daq.py                — event storage and histogram
    ├── plotter.py            — real-time histogram display
    └── requirements.txt
```

## Hardware

- **MCU:** STM32F722ZE (NUCLEO-F722ZE)
- **ASIC:** RADIOROC2 (W-Health)
- **Detector:** CsI(Tl) scintillation crystal + ARRAYJ-60035-4P SiPM
- **Power:** 3× 9V batteries in series (27V bias)
- **Communication:** USB OTG FS (CDC Virtual COM Port)

## Getting Started

### Firmware

1. Open `radioroc2_daq.ioc` in STM32CubeIDE
2. Generate Code
3. Copy all `.c` files to `Core/Src/` and `.h` files to `Core/Inc/`
4. Edit `config.h` — set active channels, hold mode, and calibration factor
5. Build and flash

### Python Application

```bash
pip install -r python/requirements.txt
python python/main.py --port COM3 --mode count --param 1000
```

**Arguments:**
- `--port`       Serial port (COM3 on Windows, /dev/ttyACM0 on Linux)
- `--mode`       `count` (N events) or `time` (T seconds)
- `--param`      N events or T seconds until histogram display
- `--channels`   Active channel numbers (default: 0 1 2)
- `--calib`      Calibration factor S_T_REF in ADC counts/keV

## Calibration

Before first use, calibrate with a known gamma source (e.g. Cs-137, 662 keV):

1. Run the system at 27°C with the source
2. Identify the photopeak ADC count in the histogram
3. Set `S_T_REF = photopeak_ADC_count / 662.0` in `config.h`
4. Or send via USB: `python main.py --calib <value>`

## Supervisors

- Prof. Adrian Stern
- Dr. Alon Osovizky

**Sponsor:** Negev Nuclear Research Center (NNRC)
