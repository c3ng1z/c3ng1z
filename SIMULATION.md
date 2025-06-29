# Spacecraft Sensor Processing Simulation

This repository provides a minimal demonstration of a spacecraft sensor
processing chain written in modern C++17.  Multiple components execute in
parallel and exchange data without any external runtime dependencies.

## Architecture

```
+------+     IMU data     +-----------+
| IMU1 | ---------------> |           |
+------+                  |           |
                          |           |
+------+     IMU data     |           |
| IMU2 | ---------------> | Processing| --> filtered log
+------+                  |           |
                          |           |
+------+     IMU data     |           |
| IMU3 | ---------------> |           |
+------+                  |           |
                          +-----------+
+-------+   GNSS data
| GNSS1 | ---------->
+-------+

+-------+   GNSS data
| GNSS2 | ---------->
+-------+

FDIR monitors all sensors and the processing output.
```

Three IMU components generate attitude-rate measurements at 100 Hz while two
GNSS components produce position data at 20 Hz.  A processing thread combines
the measurements at 50 Hz and an FDIR thread supervises overall health at 10 Hz.

## Building

A C++17 compiler is required. Build with:

```bash
g++ -std=c++17 -pthread src/main.cpp -o sim
```

## Running

The program accepts one argument defining the scenario:

- `nominal` – run for 10 s with all sensors operational.
- `imu_fail` – IMUs fail one after another after 2 s, 4 s and 6 s.
- `gnss_fail` – both GNSS units drop out for 500 ms starting at 5 s.

Example:

```bash
./sim nominal
```

Each run creates two log files named `<scenario>_proc.log` and
`<scenario>_fdir.log` which contain the processed sensor data and any alarms
raised by the FDIR component.

## Example Logs

Below are short excerpts produced when running each scenario on a test machine.

**Nominal run (`nominal_proc.log` and `nominal_fdir.log`):**

```
86.2741,0.94634,1,1000.02,1
86.2943,0.86434,1,1000.02,1
86.3141,1.02767,1,1000.02,1
```

```
86.2739,PROC_ALARM
```

**Sequential IMU failures (`imu_fail_*`):**

```
105.495,0.94634,1,1000.02,1
105.515,0.86434,1,1000.02,1
105.535,1.00933,1,1000.02,1
```

```
107.595,SENSOR_ALARM,IMU1
107.695,SENSOR_ALARM,IMU1
107.795,SENSOR_ALARM,IMU1
```

**GNSS dropout (`gnss_fail_*`):**

```
119.09,0.94634,1,1000.02,1
119.11,0.86434,1,1000.02,1
119.13,1.02867,1,1000.02,1
```

```
124.29,SENSOR_ALARM,GNSS1
124.29,SENSOR_ALARM,GNSS2
124.39,SENSOR_ALARM,GNSS1
```
