# ESPHome MPU9250 External Component

Vollständige ESPHome External Component für den MPU9250 IMU Sensor.

## Features
- Accelerometer (X/Y/Z)
- Gyroskop (X/Y/Z)
- Magnetometer AK8963 (X/Y/Z)
- Tilt-kompensierter Kompass
- Madgwick Sensor Fusion (stabiler Yaw)
- Auto-Hard-Iron Kalibrierung per Button
- Magnetische Deklination
- ESPHome / Home Assistant kompatibel

## Installation

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/DUspizig/esphome-mpu9250
      ref: main
