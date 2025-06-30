#pragma once

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <windows.h>

class SerialPort {
public:
    // Constructor
    SerialPort();
    
    // Destructor
    ~SerialPort();
    
    // Open serial port
    bool open(const std::string& portName, int baudRate = 9600);
    
    // Close serial port
    void close();
    
    // Send data
    bool send(const std::vector<uint8_t>& data);
    
    // Set receive callback
    void setReceiveCallback(std::function<void(const std::vector<uint8_t>&)> callback);
    
    // Check if serial port is open
    bool isOpen() const;
    
    // Get available serial ports
    static std::vector<std::string> getAvailablePorts();
    
private:
    // Receive thread function
    void receiveThread();
    
    HANDLE hSerial;
    std::atomic<bool> isRunning;
    std::thread receiveThreadHandle;
    std::function<void(const std::vector<uint8_t>&)> receiveCallback;
    std::mutex mutex;
}; 