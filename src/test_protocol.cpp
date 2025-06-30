#include "protocol.h"
#include "serial_port.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <map>
#include <conio.h>

// Global variables
std::atomic<bool> g_running(true);
ProtocolHandler g_protocolHandler;
SerialPort g_serialPort;
std::string g_currentPortName;
int g_currentBaudRate = 115200;

// Command handler function
void handleCommand(const std::string& cmd);

// Display help information
void showHelp() {
    std::cout << "\nAvailable commands:\n";
    std::cout << "  help         - Show this help information\n";
    std::cout << "  ports        - List available serial ports\n";
    std::cout << "  open <port>  - Open specified serial port (e.g.: open COM1)\n";
    std::cout << "  close        - Close current serial port\n";
    std::cout << "  baud <rate>  - Set baud rate (e.g.: baud 115200)\n";
    std::cout << "  stats        - Show protocol statistics\n";
    std::cout << "  reset        - Reset statistics\n";
    std::cout << "  debug on/off - Enable/disable debug mode\n";
    std::cout << "  send info    - Send info type test frame\n";
    std::cout << "  send control - Send control type test frame\n";
    std::cout << "  send param   - Send parameter type test frame\n";
    std::cout << "  send special - Send special type test frame\n";
    std::cout << "  speedtest <count> <size> - Test maximum transmission speed\n";
    std::cout << "  errortest <type>  - Generate error test frames\n";
    std::cout << "                     Types: crc, length, invalid, escape, overflow\n";
    std::cout << "  exit         - Exit program\n";
}

// List available serial ports
void listPorts() {
    std::vector<std::string> ports = SerialPort::getAvailablePorts();
    
    if (ports.empty()) {
        std::cout << "No available serial ports found\n";
        return;
    }
    
    std::cout << "Available serial ports:\n";
    for (const auto& port : ports) {
        std::cout << "  " << port << "\n";
    }
}

// Open serial port
void openPort(const std::string& portName) {
    if (g_serialPort.isOpen()) {
        g_serialPort.close();
    }
    
    if (g_serialPort.open(portName, g_currentBaudRate)) {
        g_currentPortName = portName;
        
        // Set receive callback
        g_serialPort.setReceiveCallback([](const std::vector<uint8_t>& data) {
            g_protocolHandler.processData(data.data(), data.size());
        });
    }
}

// Close serial port
void closePort() {
    g_serialPort.close();
    g_currentPortName = "";
}

// Set baud rate
void setBaudRate(int baudRate) {
    g_currentBaudRate = baudRate;
    std::cout << "Baud rate set to: " << baudRate << std::endl;
    
    if (g_serialPort.isOpen()) {
        std::cout << "Reopen serial port to apply new baud rate settings\n";
        std::string currentPort = g_currentPortName;
        closePort();
        openPort(currentPort);
    }
}

// Show protocol statistics
void showStats() {
    const ProtocolStats& stats = g_protocolHandler.getStats();
    std::cout << stats.toString() << std::endl;
}

// Reset statistics
void resetStats() {
    g_protocolHandler.resetStats();
    std::cout << "Statistics reset\n";
}

// Set debug mode
void setDebugMode(bool enabled) {
    g_protocolHandler.setDebugMode(enabled);
    std::cout << "Debug Mode: " << (enabled ? "ON" : "OFF") << std::endl;
}

// Send test frame
void sendTestFrame(const std::string& frameType) {
    if (!g_serialPort.isOpen()) {
        std::cout << "Error: Serial port not open, cannot send data\n";
        return;
    }
    
    std::vector<uint8_t> frameData;
    
    if (frameType == "info") {
        // Send version info command
        std::vector<uint8_t> versionData = {1, 0, 0}; // v1.0.0
        frameData = ProtocolHandler::createInfoCommand(0x01, 0x02, InfoCommand::VERSION, versionData);
        std::cout << "Sending info type test frame (version info)\n";
    } else if (frameType == "control") {
        // Send start command
        frameData = ProtocolHandler::createControlCommand(0x01, 0x02, ControlCommand::START);
        std::cout << "Sending control type test frame (start command)\n";
    } else if (frameType == "param") {
        // Send parameter query command
        frameData = ProtocolHandler::createParameterCommand(0x01, 0x02, 0x01, false);
        std::cout << "Sending parameter type test frame (parameter query)\n";
    } else if (frameType == "special") {
        // Send echo test command
        std::vector<uint8_t> echoData = {'H', 'e', 'l', 'l', 'o'};
        frameData = ProtocolHandler::createSpecialCommand(0x01, 0x02, SpecialCommand::ECHO, echoData);
        std::cout << "Sending special type test frame (echo test)\n";
    } else {
        std::cout << "Unknown frame type: " << frameType << std::endl;
        return;
    }
    
    // Print frame content
    std::cout << "Frame data: ";
    for (uint8_t byte : frameData) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
    
    // Send data
    if (g_serialPort.send(frameData)) {
        std::cout << "Send successful, total " << frameData.size() << " bytes\n";
        
        // 手动处理发送的数据（模拟回环）
        g_protocolHandler.processData(frameData.data(), frameData.size());
    } else {
        std::cout << "Send failed\n";
    }
}

// Run maximum speed test
void runSpeedTest(int frameCount, int dataSize) {
    if (!g_serialPort.isOpen()) {
        std::cout << "Error: Serial port not open, cannot run speed test\n";
        return;
    }
    
    if (frameCount <= 0 || dataSize <= 0) {
        std::cout << "Error: Invalid parameters. Count and size must be positive.\n";
        return;
    }
    
    // Keep current debug mode setting
    bool oldDebugMode = g_protocolHandler.isDebugMode();
    
    // Prepare test data
    std::vector<uint8_t> testData(dataSize);
    for (int i = 0; i < dataSize; i++) {
        testData[i] = static_cast<uint8_t>(i & 0xFF);
    }
    
    std::cout << "Starting speed test: " << frameCount << " frames, " << dataSize << " bytes per frame\n";
    
    // Reset statistics
    resetStats();
    
    // Record start time
    auto startTime = std::chrono::steady_clock::now();
    
    // Send frames
    int successCount = 0;
    for (int i = 0; i < frameCount; i++) {
        // Create special command with test data
        std::vector<uint8_t> frameData = ProtocolHandler::createSpecialCommand(
            0x01, 0x02, SpecialCommand::ECHO, testData);
        
        std::cout << "Frame " << (i+1) << " size: " << frameData.size() << " bytes\n";
        
        // Send data
        if (g_serialPort.send(frameData)) {
            successCount++;
            
            // 手动处理发送的数据（模拟回环）
            g_protocolHandler.processData(frameData.data(), frameData.size());
        }
        
        // Show progress every 10%
        if (frameCount >= 10 && i % (frameCount / 10) == 0 || i == frameCount - 1) {
            std::cout << "Progress: " << (i + 1) << "/" << frameCount << " frames sent\r";
            std::cout.flush();
        }
    }
    
    // Record end time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    
    // Calculate statistics
    double totalBytes = successCount * (dataSize + 10); // Add ~10 bytes for frame overhead
    double bytesPerSecond = (elapsedTime > 0) ? (totalBytes * 1000 / elapsedTime) : 0;
    double framesPerSecond = (elapsedTime > 0) ? (successCount * 1000.0 / elapsedTime) : 0;
    
    // Force update protocol statistics before displaying them
    // Call multiple times to ensure all frames are processed
    for (int i = 0; i < 3; i++) {
        g_protocolHandler.processData(nullptr, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "\nSpeed test completed:\n";
    std::cout << "  Frames sent: " << successCount << " of " << frameCount << "\n";
    std::cout << "  Total bytes: " << std::fixed << std::setprecision(2) << totalBytes << " bytes\n";
    std::cout << "  Elapsed time: " << elapsedTime << " ms\n";
    std::cout << "  Throughput: " << std::fixed << std::setprecision(2) << bytesPerSecond << " bytes/sec\n";
    std::cout << "  Frame rate: " << std::fixed << std::setprecision(2) << framesPerSecond << " frames/sec\n";
    
    // Show protocol statistics
    showStats();
}

// Create and send error test frame
void sendErrorTestFrame(const std::string& errorType) {
    if (!g_serialPort.isOpen()) {
        std::cout << "Error: Serial port not open, cannot send data\n";
        return;
    }
    
    // Reset statistics before the test
    resetStats();
    std::cout << "Statistics reset before error test\n";
    
    std::vector<uint8_t> frameData;
    
    if (errorType == "crc") {
        try {
            // Create a valid frame first
            frameData = ProtocolHandler::createInfoCommand(0x01, 0x02, InfoCommand::VERSION, {1, 0, 0});
            
            // Print the original frame for debugging
            std::cout << "Original frame before CRC modification: ";
            for (uint8_t byte : frameData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
            
            // Build raw data for CRC calculation
            // This should match exactly how the ProtocolHandler::createFrame does it
            std::vector<uint8_t> crcData;
            
            // From field
            crcData.push_back(0x01);
            
            // To field
            crcData.push_back(0x02);
            
            // Flags field
            crcData.push_back(0x00);
            
            // Command byte
            crcData.push_back((static_cast<uint8_t>(CommandType::INFO) << 6) | static_cast<uint8_t>(InfoCommand::VERSION)); // cmd
            
            // Length (little-endian) - 3 bytes data length
            crcData.push_back(0x03);  // len (low byte)
            crcData.push_back(0x00);  // len (high byte)
            
            // Data
            crcData.insert(crcData.end(), {1, 0, 0});
            
            uint8_t correctCrc = ProtocolHandler::calculateCRC8(crcData);
            
            // Display original CRC value
            uint8_t originalCrc = frameData[frameData.size() - 2];
            std::cout << "Original CRC: 0x" << std::hex << static_cast<int>(originalCrc) << std::dec << std::endl;
            std::cout << "Calculated CRC: 0x" << std::hex << static_cast<int>(correctCrc) << std::dec << std::endl;
            
            // Corrupt the CRC (second to last byte)
            if (frameData.size() >= 2) {
                frameData[frameData.size() - 2] = (originalCrc == 0x00) ? 0xFF : 0x00; // Ensure CRC value changes
                std::cout << "Modified CRC: 0x" << std::hex << static_cast<int>(frameData[frameData.size() - 2]) << std::dec << std::endl;
            }
            
            // Output frame data in hexadecimal, including modified CRC
            std::cout << "Frame with invalid CRC: ";
            for (uint8_t byte : frameData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
            
            std::cout << "Sending frame with invalid CRC\n";
        } catch (const std::exception& e) {
            std::cerr << "Exception during CRC error test creation: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown exception during CRC error test creation" << std::endl;
        }
    } 
    else if (errorType == "length") {
        // Create a frame with length exceeding MAX_FRAME_SIZE
        frameData.push_back(FRAME_START);
        frameData.push_back(0x01); // From
        frameData.push_back(0x02); // To
        frameData.push_back(0x00); // Flags
        frameData.push_back(0x01); // Command
        frameData.push_back(0x00); // Length LSB (claim 2048 bytes)
        frameData.push_back(0x08); // Length MSB (0x0800 = 2048)
        
        // Add some data (much less than the claimed length)
        for (int i = 0; i < 10; i++) {
            frameData.push_back(static_cast<uint8_t>(i));
        }
        
        // Calculate CRC
        std::vector<uint8_t> crcData = {0x01, 0x02, 0x00, 0x01, 0x00, 0x08};
        for (int i = 0; i < 10; i++) {
            crcData.push_back(static_cast<uint8_t>(i));
        }
        uint8_t crc = ProtocolHandler::calculateCRC8(crcData);
        frameData.push_back(crc);
        
        // Add stop byte
        frameData.push_back(FRAME_STOP);
        std::cout << "Sending frame with length error (claimed 2048 bytes, actual 10 bytes)\n";
    } 
    else if (errorType == "invalid") {
        // Create a frame with invalid stop byte
        frameData = ProtocolHandler::createInfoCommand(0x01, 0x02, InfoCommand::VERSION, {1, 0, 0});
        
        // Print the original frame for debugging
        std::cout << "Original frame before modification: ";
        for (uint8_t byte : frameData) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
        
        // Replace stop byte with invalid value
        if (!frameData.empty()) {
            frameData[frameData.size() - 1] = 0xAA; // Not the FRAME_STOP value
        }
        std::cout << "Sending frame with invalid stop byte\n";
    } 
    else if (errorType == "escape") {
        // Create a frame with invalid escape sequence
        frameData.push_back(FRAME_START);
        frameData.push_back(0x01); // From
        frameData.push_back(0x02); // To
        frameData.push_back(0x00); // Flags
        
        // Add an escape character followed by an invalid escape sequence
        frameData.push_back(ESCAPE_CHAR);
        frameData.push_back(0x00); // Invalid escaped byte
        
        // Add some valid data
        frameData.push_back(0x01); // Command
        frameData.push_back(0x03); // Length LSB
        frameData.push_back(0x00); // Length MSB
        frameData.push_back('A');
        frameData.push_back('B');
        frameData.push_back('C');
        
        // Add CRC and stop (these will be ignored due to earlier error)
        frameData.push_back(0x00); // Dummy CRC
        frameData.push_back(FRAME_STOP);
        std::cout << "Sending frame with invalid escape sequence\n";
    } 
    else if (errorType == "overflow") {
        // Create a frame that's too large
        frameData.push_back(FRAME_START);
        frameData.push_back(0x01); // From
        frameData.push_back(0x02); // To
        frameData.push_back(0x00); // Flags
        frameData.push_back(0x01); // Command
        frameData.push_back(0xFF); // Length LSB (claim 1023 bytes)
        frameData.push_back(0x03); // Length MSB
        
        // Add some data (not the full 1023 bytes)
        for (int i = 0; i < 100; i++) {
            frameData.push_back(static_cast<uint8_t>(i & 0xFF));
        }
        
        // Add CRC and stop
        frameData.push_back(0x00); // Dummy CRC
        frameData.push_back(FRAME_STOP);
        std::cout << "Sending frame with buffer overflow potential\n";
    } 
    else {
        std::cout << "Unknown error test type: " << errorType << std::endl;
        std::cout << "Available types: crc, length, invalid, escape, overflow\n";
        return;
    }
    
    // Print frame content
    std::cout << "Error frame data: ";
    for (uint8_t byte : frameData) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
    
    // Send data
    if (g_serialPort.send(frameData)) {
        std::cout << "Error test frame sent successfully, total " << frameData.size() << " bytes\n";
        
        // 手动处理发送的数据（模拟回环）
        g_protocolHandler.processData(frameData.data(), frameData.size());
        
        // Wait for a moment to ensure the error frame is processed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Force update statistics multiple times to ensure all errors are processed
        for (int i = 0; i < 3; i++) {
            g_protocolHandler.processData(nullptr, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Show statistics to check if the error was properly recorded
        std::cout << "\nCheck error statistics:\n";
        showStats();
    } else {
        std::cout << "Error test frame send failed\n";
    }
}

// Command handler function
void handleCommand(const std::string& cmdLine) {
    std::istringstream iss(cmdLine);
    std::string cmd;
    iss >> cmd;
    
    if (cmd == "help" || cmd == "h") {
        showHelp();
    } else if (cmd == "ports") {
        listPorts();
    } else if (cmd == "open") {
        std::string portName;
        iss >> portName;
        if (portName.empty()) {
            std::cout << "Error: Please specify port name\n";
        } else {
            openPort(portName);
        }
    } else if (cmd == "close") {
        closePort();
    } else if (cmd == "baud") {
        int baudRate;
        iss >> baudRate;
        if (baudRate <= 0) {
            std::cout << "Error: Invalid baud rate\n";
        } else {
            setBaudRate(baudRate);
        }
    } else if (cmd == "stats") {
        showStats();
    } else if (cmd == "reset") {
        resetStats();
    } else if (cmd == "debug") {
        std::string mode;
        iss >> mode;
        if (mode == "on") {
            setDebugMode(true);
        } else if (mode == "off") {
            setDebugMode(false);
        } else {
            std::cout << "Error: Please specify 'on' or 'off'\n";
        }
    } else if (cmd == "send") {
        std::string frameType;
        iss >> frameType;
        if (frameType.empty()) {
            std::cout << "Error: Please specify frame type (info, control, param, special)\n";
        } else {
            sendTestFrame(frameType);
        }
    } else if (cmd == "speedtest") {
        int count = 100;  // Default count
        int size = 64;    // Default size
        
        iss >> count;
        iss >> size;
        
        runSpeedTest(count, size);
    } else if (cmd == "errortest") {
        std::string errorType;
        iss >> errorType;
        
        if (errorType.empty()) {
            std::cout << "Error: Please specify error type\n";
            std::cout << "Available types: crc, length, invalid, escape, overflow\n";
        } else {
            sendErrorTestFrame(errorType);
        }
    } else if (cmd == "exit" || cmd == "quit" || cmd == "q") {
        g_running = false;
    } else if (!cmd.empty()) {
        std::cout << "Unknown command: " << cmd << "\n";
        std::cout << "Type 'help' for help\n";
    }
}

int main() {
    std::cout << "UART Protocol Parser Test Program\n";
    std::cout << "Type 'help' for command list\n";
    
    // Set frame receive callback
    g_protocolHandler.setFrameReceivedCallback([](const ProtocolFrame& frame, const std::string& info) {
        std::cout << "\nReceived frame: " << info << std::endl;
    });
    
    // Configure protocol parser
    g_protocolHandler.configureFieldLengths(1, 1, 1);
    
    // Command line loop
    std::string cmdLine;
    while (g_running) {
        std::cout << "\n> ";
        std::getline(std::cin, cmdLine);
        handleCommand(cmdLine);
    }
    
    // Close serial port
    if (g_serialPort.isOpen()) {
        g_serialPort.close();
    }
    
    std::cout << "Program exited\n";
    return 0;
} 