# IoT-Based Remote Health Monitoring System

## Overview
This project presents the design and implementation of an **Internet of Things (IoT)–based remote health monitoring system** capable of measuring and displaying key physiological parameters in real time.

The system integrates:
- **Pulse Oximetry (SpO₂)**
- **Electrocardiogram (ECG)**
- **Body Temperature sensors**

Data is collected using an **Arduino Nano 33 IoT** microcontroller and transmitted via **Wi-Fi** to a **Raspberry Pi 4 Model B**, where it is visualized on a custom **ThingsBoard dashboard**.

This project demonstrates the feasibility of combining **biomedical sensing**, **IoT connectivity**, and **data visualization** into a **low-cost, compact**, and **scalable health monitoring platform** suitable for both personal and clinical applications.

---

## Motivation
Modern health monitoring systems are often **expensive, closed-source, and limited to clinical environments**. This project seeks to address that gap by developing a **low-cost, open, and portable** IoT-based alternative that can continuously monitor vital signs and provide **real-time insights** remotely.

---

## System Architecture

### **Hardware Components**
- **Arduino Nano 33 IoT**
- **Raspberry Pi 4 Model B**
- **MAX30102** — Pulse oximeter and heart rate sensor  
- **AD8232** — ECG sensor  
- **MLX90614** — Infrared temperature sensor  
- **External Power Bank** — For portable operation

### **Software Stack**
| Component | Purpose |
|------------|----------|
| **Arduino IDE** | Sensor data acquisition and preprocessing |
| **Python (Raspberry Pi)** | Data parsing, MQTT transmission |
| **ThingsBoard** | Real-time visualization and dashboarding |
| **Wi-Fi (MQTT Protocol)** | Data transmission between devices |
| **Visual Studio Code** | Raspberry Pi code configuration and debugging |

---

## System Workflow

1. **Data Acquisition**  
   Arduino Nano 33 IoT collects SpO₂, ECG, and temperature data using connected biomedical sensors.

2. **Preprocessing & Transmission**  
   The Arduino formats and transmits data packets to the Raspberry Pi via Wi-Fi.

3. **Data Handling on Raspberry Pi**  
   Python scripts on the Raspberry Pi receive the data, parse it, and forward it to **ThingsBoard** using the MQTT protocol.

4. **Visualization & Monitoring**  
   The ThingsBoard dashboard displays real-time graphs, numerical readouts, and device status indicators accessible from any browser.

---

## Performance Evaluation

### **Sensor Accuracy**
Sensor readings are compared with reference medical instruments to verify accuracy and consistency.  
Preliminary testing observed an **average latency of 1–2 seconds** from acquisition to display.

### **Power Consumption**
Power draw is monitored using a USB Power Meter to evaluate performance under portable conditions and optimize energy efficiency for battery-powered operation.

---

## Features
- Real-time measurement and display of SpO₂, ECG, and temperature
- Wireless data transmission via Wi-Fi (MQTT protocol)  
- Interactive, customizable **ThingsBoard dashboard**  
- Portable design powered by rechargeable power banks  
- Modular and open-source for easy modification or scaling  

---

### **Hardware Setup**
1. Connect the **MAX30102**, **MLX90614**, and **AD8232** sensors to the **Arduino Nano 33 IoT**.
2. Configure the **Arduino Nano 33 IoT** to connect to your Wi-Fi network.
3. Connect the **Arduino** and **Raspberry Pi 4** to the same local network.
4. Power the system using USB or an external power bank.
