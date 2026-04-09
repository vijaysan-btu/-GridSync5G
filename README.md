# GridSync5G: Latency-Aware 5G Communication for PHIL-Based Grid Frequency Control

## Overview

GridSync5G is a Power Hardware-in-the-Loop (PHIL) project that investigates the feasibility of using 5G communication for real-time coordination of distributed energy resources (DERs) in low-inertia power systems.

The project implements a centralized grid frequency control architecture where control signals are transmitted across battery, supercapacitor, and fuel cell units using a hybrid UDP–MQTT communication pipeline over a private 5G network.

## Problem Statement

Modern power systems with high renewable penetration lack inherent inertia, making frequency stability a critical challenge. Coordinating distributed energy resources requires:

* Low-latency communication
* High reliability
* Scalable control architecture

While wired communication offers low latency, it is difficult to scale. This project explores whether 5G communication can meet the requirements of real-time grid control.

## System Architecture

The system is built on a PHIL testbed combining real hardware and real-time simulation.

### Components

* **Central Controller** (Battery module)
* **Distributed Energy Resources (DERs):**

  * Supercapacitor → fast frequency support
  * Battery → secondary control
  * Fuel cell → sustained power support
* **Communication Layer:**

  * UDP → real-time control signals
  * MQTT → telemetry and message routing
  * ROS2 → middleware bridge

### Control & Communication Flow

1. Grid frequency deviation is measured
2. Central controller computes power references (P_ref)
3. P_ref signals are sent via UDP to a communication bridge
4. MQTT distributes signals to DER-specific topics
5. DER units subscribe and execute control actions

This hybrid pipeline combines the speed of UDP with the flexibility of MQTT.

## Key Features

* PHIL-based validation with real hardware interaction
* Centralized control of multi-energy DER system
* Hybrid UDP–MQTT communication architecture
* 5G-enabled wireless control infrastructure
* Binary data transmission using struct-based packing
* ROS2-based UDP–MQTT bridge for protocol integration

## Experimental Setup

* Speedgoat real-time controllers running MATLAB/Simulink models
* Raspberry Pi edge devices connected via 5G modules
* Private 5G network with dedicated testbed infrastructure
* Distributed DER units: battery, supercapacitor, fuel cell
* Real-time communication using UDP and MQTT protocols

## Results

* End-to-end communication latency: ~100 ms
* Reliable message delivery across all DER units
* Successful MQTT topic-based routing and filtering
* Stable communication under hybrid UDP–MQTT architecture

## Key Insights

* UDP enables fast transmission but lacks delivery guarantees
* MQTT improves reliability and scalability but introduces latency
* Hybrid architecture balances flexibility and performance
* Observed latency (~100 ms) exceeds requirements for primary frequency control

## Limitations

* Control performance not fully validated (frequency response not measured)
* Communication latency higher than expected for real-time control
* No direct benchmarking against wired communication
* Lack of global time synchronization

## Future Work

* Reduce latency via:

  * Direct UDP control paths
  * MQTT QoS optimization
  * Edge-based/local control strategies

* Implement closed-loop frequency validation

* Compare wired vs. 5G performance under identical conditions

* Extend system to decentralized or distributed control architectures

## Technologies Used

* MATLAB / Simulink (PHIL modeling)
* Speedgoat Real-Time Systems
* Python (communication bridge and clients)
* ROS2 (middleware)
* MQTT (Mosquitto broker)
* UDP communication
* Private 5G network

## Author

* Santhosh Kumar Vijayalakshmi Murugan (https://www.linkedin.com/in/santhosh-vm-24ba681b2/)
* Lubna Basha Mohammed (https://www.linkedin.com/in/lubna-basha-mohammed-48297626a/)

## Summary

This project demonstrates a scalable communication architecture for smart grid control using 5G. While current latency limits real-time applicability, the system establishes a strong foundation for future research in communication-aware energy systems.
