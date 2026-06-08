# An Embedded System for SIDS Prevention: Newborns Signals Monitoring (NICU)

This repository contains the full embedded firmware, hardware architecture, and graduation thesis for a low-cost, home-based infant monitoring device designed for the early detection of physiological abnormalities and apnea events.

## 📺 Project Demonstration
Click the media player below to watch the complete hardware walk-through, system architecture presentation, and live demonstration:

[![Watch the Project Video](https://img.youtube.com/vi/_RN2-44NaQc/0.jpg)](https://youtu.be/_RN2-44NaQc)

*Note: The hardware prototype links seamlessly with a mobile application ("Vital Monitor") for real-time biometric tracking.*

---

## 📄 Abstract
This study presents the design and conceptual development of a low-cost, home-based infant monitoring device aimed at early detection of physiological abnormalities and apnea events. The proposed system continuously monitors key vital parameters including heart rate (HR), oxygen saturation ($SpO_2$), body temperature, estimated blood pressure (BP), and apnea episodes using non-invasive sensing techniques. 

The device is designed specifically to support caregivers at home following an infant's discharge from the Neonatal Intensive Care Unit (NICU), providing a supportive monitoring tool rather than a replacement for professional medical diagnosis.

## 🛠️ System Architecture & Hardware Stack
The project features a decentralized, dual-microcontroller architecture to isolate real-time data streaming from safety-critical alarm processing:

* **Main System Controller (ESP32):** Receives data packages, executes precise threshold-checking logic, and manages emergency notification routines.
* **AD8232 ECG Module:** Utilizes a standard 3-electrode configuration (RA, LA, RL) for primary cardiac monitoring and heart rate estimation.
* **MAX30102 Sensor:** Evaluates photoplethysmography (PPG) waveforms to extract $SpO_2$ percentages and acts as a signal backup for heart rate verification.
* **MPU6050 IMU:** Tracks respiration-driven movement patterns to classify sleep apnea conditions.
* **MCP9808 Sensor:** High-precision skin-contact sensor for continuous body temperature logging.

---

## 📊 Process Flow & Decision Logic
The system evaluates risk levels by continuously updating and categorizing parameters into **Normal**, **Warning**, or **Critical** states:

1. **Signal Acquisition:** Raw biometric streams are gathered from the sensor cluster.
2. **Preprocessing:** Inputs undergo digital filtering, resampling, and timestamp synchronization to eliminate motion noise.
3. **Feature Extraction:** Calculates final $HR$, $SpO_2$, breathing cessation anomalies, and skin temperature values.
4. **State Classification:** Compares real-time values with safe thresholds to trigger visual LED alerts or audible caregiver notifications if abnormal parameters are caught.

---

## 📂 Repository Layout
* `/code/esp32_firmware` - Control firmware managing logic trees, multi-state alert conditions, and serial output logging.
* `/thesis` - Contains the final comprehensive academic PDF text (`NICU_Graduation_Thesis.pdf`).

## 🎓 Academic Credit
* **Author:** Mohamed Elsayed Masri Mayhoub (Yıldız Technical University)
* **Advisor:** Assoc. Prof. İsmail Cantürk
* **Department:** Department of Biomedical Engineering
* **Year:** 2026
