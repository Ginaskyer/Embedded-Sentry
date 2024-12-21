# Embedded Sentry: Gesture-Based Unlock System

## Overview
Embedded Sentry is a gesture-based unlock system developed as the final project for the **ECE-GY 6483 Embedded System** course. The system leverages gyroscope data to recognize predefined hand gestures for device unlocking, enhancing security and interactivity.

### Team Members
- Xian Wu, Xingyu Liu, Yimeng Zhang

---

## Features
- **Train Mode(Gesture Training Mode)**: Record and save a user-defined gesture.
- **Test Mode(Gesture Recognition Mode)**: Compare a live gesture against the saved one for verification.
- **SPI Communication**: Interfaces with sensors for precise motion data collection.
- **LCD Feedback**: Displays the system's status and results.

---

## Components and Tools
- **Hardware**:
  - STM32F429ZI Discovery Board
  - Gyroscope sensor (SPI communication)
- **Software**:
  - [Mbed OS](https://os.mbed.com/)
  - PlatformIO (for project management)
- **IDE**: Visual Studio Code

---

## System Modes
### 1. Initialization Mode
The default state where the system waits for user input:
- Short button press (<1 second): Enters **Train Mode**.
- Long button press (>1 second): Enters **Test Mode**.

### 2. Train Mode
The system records a sequence of gesture data using the gyroscope:
- Gyroscope data is processed with a moving average filter.
- Gesture data is normalized for consistency.

### 3. Test Mode
The system captures a live gesture and compares it with the saved one:
- Uses DTW to calculate the similarity between gestures.
- Displays the result (`Success` or `Failure`) on the LCD screen.

---

## Code Overview
### Key Modules
1. **SPI Communication**:
   - Reads gyroscope data (`X`, `Y`, `Z` axes).
   - Configured with a clock frequency of 1 MHz.

2. **Gesture Recognition**:
   - Moving Average Filter: Reduces noise in the sensor data.
   - DTW Algorithm: Matches gestures based on their spatial and temporal characteristics.

3. **LCD Control**:
   - Displays user instructions, progress, and results.

4. **Button Handling**:
   - Detects button presses with debounce and duration logic.
   - Switches between system modes based on input.

---

## Usage Instructions
### Steps
1. Install Visual Studio Code and the **PlatformIO** extension:
   - Download and install [Visual Studio Code](https://code.visualstudio.com/).
   - Install the **PlatformIO IDE** extension from the VS Code Marketplace.
  
2. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/embedded-sentry.git

3. Upload and Test
