# Embedded Ethernet Communication with Nucleo-H723ZG

This project demonstrates Ethernet communication using the Nucleo-H723ZG development board. It captures analog voltage samples from the onboard ADC, transmits them over Ethernet, and uses a UART interface for logging received packets. The transmitted packets can be monitored and analyzed in Wireshark.

---

## Features

- **Microcontroller**: Nucleo-H723ZG (STM32H7 series)  
- **Communication Protocol**: Ethernet  
- **Data Sampling**: 12-bit ADC captures voltage samples  
- **Data Transmission**: Data is transmitted as packets using Ethernet frames  
- **Data Monitoring**: Packets are analyzed using Wireshark  
- **Data Logging**: Packets received are displayed via UART  
- **Outlier Detection (Optional)**: Real-time inference using a Flask-based anomaly detection server

---

## Hardware Setup

- **Nucleo-H723ZG**: Ensure that the board is properly powered and connected to a network.
- **UART Connection**: Set up UART to capture and display incoming packets. Use a USB-to-UART converter if needed.
- **Wireshark**: Use Wireshark on the PC to capture and monitor outgoing Ethernet packets.

---

## Software Requirements

- **STM32CubeIDE**: For project compilation and programming  
- **Wireshark**: For packet capture and analysis  
- **Minicom/Tera Term**: To monitor UART communication (optional)  
- **Python 3 (Linux)**: For hosting the anomaly detection server  
- **Required Python Libraries**: `Flask`, `TensorFlow`, `NumPy`

---

## Project Overview

This project involves setting up an Ethernet communication protocol using the Nucleo-H723ZG board. The application samples analog voltage signals using the onboard ADC, packages them into Ethernet frames, and transmits them over the network. The transmitted data can be captured in Wireshark for verification.

The project uses the `HAL` library for configuring the Ethernet and ADC peripherals on the STM32H723ZG board.

---

### Packet Structure

- Each Ethernet packet contains a sequence of 32-bit floating-point ADC voltage samples.
- These packets can be parsed and exported from Wireshark for visualization, training, or inference.

---

## Outlier Detection Server (`outlier_server`)

An optional Python-based Flask server enables real-time anomaly detection using the STM32 data stream.

### Setup

1. Create a Python virtual environment on a Linux machine:

   ```bash
   python3 -m venv ekg_env
   source ekg_env/bin/activate
   pip install flask tensorflow numpy
   ```

2. Place `voltage.keras`, `mean.npy`, and `std.npy` in the same directory as `serve_voltage.py`.

3. Launch the server:

   ```bash
   python serve_voltage.py
   ```

   The server listens on port `5000` and exposes a `/predict` endpoint for inference.

> Note: The `voltage.keras` model was trained on real-world ADC data from the STM32H723ZG using its built-in Ethernet MAC. If you're using the same board and setup, retraining is likely unnecessary.

---

### Packet Sniffing + Inference Pipeline

Use the `sniff_mac.c` application to capture Ethernet packets filtered by the target MAC address. The app has two operating modes:

- **Training Mode** (`TRAIN_MODE` defined):  
  Captures a continuous stream of packets and writes them into a raw training file.
  
- **Inference Mode** (default):  
  Captures packets and extracts floating-point payloads for real-time JSON inference via POST to the Flask server.

To configure the server address, edit:

```c
#define SERVER_URL "http://<your-localhost-or-remote>:5000/predict"
```

This should point to your Flask server endpoint.

---

## Usage

1. **Compile and Flash**: Compile in STM32CubeIDE and flash the board.
2. **Build Sniffer App**: Compile `sniff_mac.c` on your Linux box.
3. **Set Mode**: Define or undefine `TRAIN_MODE` depending on whether you're building a dataset or running inference.
4. **Run Flask Server**: Launch `serve_voltage.py` to host the model for inference.
5. **Start Packet Capture**: Run `sniff_mac` to gather data and send it to the server.
6. **Monitor Dashboard**: Visit `http://localhost:5000` to see inference results and anomaly flags.

---

## Future Enhancements

- Implement FFT analysis on the received data  
- Real-time waveform and anomaly visualization UI  
- Cross-platform support for packet sniffer  
- Broader STM32 family compatibility for training/inference

---

## Repository Structure

- `Src/` – STM32 source files for ADC, Ethernet, UART
- `Inc/` – Header files for peripheral configuration
- `sniff_mac.c` – Raw Ethernet sniffer client for training/inference
- `serve_voltage.py` – Flask server that performs anomaly detection
- `index.html` – Dashboard UI for monitoring incoming predictions
- `readme.md` – This documentation

---

## License

This project is licensed under the GPL 2 License.

