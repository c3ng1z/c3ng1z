# C++ Sensor Simulation Explained for C Developers

This guide describes the structure of the sensor processing simulation, focusing on the object-oriented C++ features. It aims to help C programmers understand the code and be ready to discuss it in an interview.

## Program Overview

The program models a simple spacecraft sensor chain. Several sensors run in their own threads, a processing component aggregates their data, and a Fault Detection, Isolation and Recovery (FDIR) component monitors everything. The implementation relies only on the C++17 standard library.

The main executable `sim` is built from `src/main.cpp`.

## Key Classes

The simulation is organized around classes. Each class bundles data and the functions that operate on that data.

- **`SensorBase`** – an abstract base class that manages a sensor thread. It handles periodic measurement generation and supports fault injection. It defines a `generateValue()` virtual function that derived classes override.
- **`IMUSensor`** and **`GnssSensor`** – concrete sensor classes derived from `SensorBase`. Each implements `generateValue()` to produce data representative of an IMU or GNSS receiver.
- **`Processing`** – runs a 50 Hz thread that reads new sensor data, filters it and writes the results to a log file.
- **`FDIR`** – monitors the last output from every sensor and from the processing component. It raises alarms when data is missing.
- **`Simulation`** – orchestrates creation of sensors, processing and FDIR, runs the desired scenario and ensures proper startup and shutdown.

## Object-Oriented Features

### Classes vs. C Structs

In C, data structures and the functions that operate on them are usually separate. In C++, classes combine data and methods in a single type. Each class can control its own construction and destruction, reducing manual bookkeeping.

### Inheritance and Virtual Functions

`SensorBase` declares `generateValue()` as a **virtual** method. Subclasses such as `IMUSensor` override this method to provide sensor‑specific behavior. At runtime, calling `generateValue()` on a `SensorBase` pointer invokes the derived class’s implementation. This is called **polymorphism**.

### Constructors and Destructors

Classes have constructors to initialize members and destructors to clean up. For example, `SensorBase` starts its thread in `start()` and stops it in the destructor, implementing the RAII (Resource Acquisition Is Initialization) idiom. In C you would typically allocate resources and free them manually.

### Smart Pointers

The simulation uses `std::unique_ptr` to manage dynamic allocations for sensors and components. A `unique_ptr` automatically deletes the owned object when it goes out of scope, preventing memory leaks. In C we would need to call `free()` explicitly.

### std::thread and Concurrency

Each sensor object owns a `std::thread` that repeatedly calls its `generateValue()` function. The processing and FDIR components also run in their own threads. Synchronization is done with `std::mutex` and `std::atomic`. In C you could achieve similar concurrency using POSIX threads, but C++ wraps them in higher-level abstractions.

## Program Flow

1. **`main`** parses the scenario argument and constructs a `Simulation` object.
2. The chosen scenario (nominal run or one of the failure tests) calls `setup()` in `Simulation`, which creates sensor objects using `std::make_unique`.
3. `Simulation::startAll()` starts every sensor thread and launches the processing and FDIR threads.
4. The program sleeps for the scenario duration (10 s) while the threads run in the background.
5. `Simulation::stopAll()` stops all threads and cleans up before exiting.

## Reading the Code

- Look for class declarations to see what data and functions each component encapsulates. For example, `class Processing` defines its own thread and log file.
- Notice how `IMUSensor` and `GnssSensor` do not duplicate the thread management code— they simply inherit it from `SensorBase`.
- Pay attention to destructors (`~ClassName()`). They ensure threads are joined and files closed automatically when objects go out of scope.

## Possible Interview Questions

- **"How are sensors implemented?"** – Each sensor class derives from `SensorBase` and overrides `generateValue()`. `SensorBase` handles the threading and queueing.
- **"How is memory managed?"** – Dynamic objects use `std::unique_ptr` so they are deleted automatically. There is no manual `new`/`delete` or `malloc`/`free` in the main logic.
- **"What happens if a sensor stops producing data?"** – `FDIR` checks timestamps of the last outputs. If no data arrives for three cycles of the sensor’s nominal period, it logs a `SENSOR_ALARM`.

## Conclusion

The program demonstrates several object-oriented techniques—inheritance, polymorphism, encapsulation and RAII—while still resembling C in its use of simple data structures and manual control over threads. Understanding these features will help explain the code and its differences from a pure C implementation.

