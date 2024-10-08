
# Embedded Ethernet Communication with Nucleo-H723ZG

This project demonstrates Ethernet communication using the Nucleo-H723ZG development board. It captures analog voltage samples from the onboard ADC, transmits them over Ethernet, and uses a UART interface for logging received packets. The transmitted packets can be monitored and analyzed in Wireshark.

## Features
- **Microcontroller**: Nucleo-H723ZG (STM32H7 series)  
- **Communication Protocol**: Ethernet  
- **Data Sampling**: 12-bit ADC captures voltage samples  
- **Data Transmission**: Data is transmitted as packets using Ethernet frames  
- **Data Monitoring**: Packets are analyzed using Wireshark  
- **Data Logging**: Packets received are displayed via UART

## Hardware Setup
- **Nucleo-H723ZG**: Ensure that the board is properly powered and connected to a network.
- **UART Connection**: Set up UART to capture and display incoming packets. Use a USB-to-UART converter if needed.
- **Wireshark**: Use Wireshark on the PC to capture and monitor outgoing Ethernet packets.

## Software Requirements
- **STM32CubeIDE**: For project compilation and programming
- **Wireshark**: For packet capture and analysis
- **Minicom/Tera Term**: To monitor UART communication (optional)

## Project Overview
This project involves setting up an Ethernet communication protocol using the Nucleo-H723ZG board. The application samples analog voltage signals using the onboard ADC, packages them into Ethernet frames, and transmits them over the network. The transmitted data can be captured in Wireshark for verification.

The project uses the `HAL` library for configuring the Ethernet and ADC peripherals on the STM32H723ZG board.

### Packet Structure
- Each Ethernet packet is structured to include a sequence of ADC voltage samples, transmitted as a series of 32-bit floating-point values.
- The received packets can be analyzed using Wireshark by exporting the data as C arrays for further processing.

### Usage
1. **Compile and Flash**: Compile the code in STM32CubeIDE and flash it onto the Nucleo-H723ZG board.
2. **Monitor UART**: Open a terminal program to view the UART output for any received packets.
3. **Start Wireshark**: Begin capturing on the relevant network interface in Wireshark.
4. **Analyze Packets**: Review the transmitted packets in Wireshark to verify data integrity and transmission format.

## Future Enhancements
- Implement FFT analysis on the received data.
- Create a client application to visualize the received voltage samples in real time.
- Add support for other microcontrollers in the STM32 family.

## Repository Structure
- `Src/` - Contains the main source files for ADC configuration, Ethernet communication, and UART logging.
- `Inc/` - Header files for peripheral configuration and project definitions.
- `readme.md` - This readme file with project overview and usage instructions.

## License
This project is licensed under the GPL 2 License
