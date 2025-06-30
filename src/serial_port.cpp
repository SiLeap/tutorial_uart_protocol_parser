#include "serial_port.h"
#include <iostream>
#include <sstream>

SerialPort::SerialPort() : hSerial(INVALID_HANDLE_VALUE), isRunning(false) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& portName, int baudRate) {
    // Close already opened serial port
    if (isOpen()) {
        close();
    }
    
    // Open serial port
    std::string fullPortName = "\\\\.\\" + portName;
    hSerial = CreateFileA(
        fullPortName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open serial port " << portName << ", error code: " << GetLastError() << std::endl;
        return false;
    }
    
    // Set serial port parameters
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to get serial port state, error code: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Set baud rate
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Failed to set serial port parameters, error code: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Failed to set serial port timeout parameters, error code: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Start receive thread
    isRunning = true;
    receiveThreadHandle = std::thread(&SerialPort::receiveThread, this);
    
    std::cout << "Successfully opened serial port " << portName << " baud rate: " << baudRate << std::endl;
    return true;
}

void SerialPort::close() {
    if (isOpen()) {
        // Stop receive thread
        isRunning = false;
        if (receiveThreadHandle.joinable()) {
            receiveThreadHandle.join();
        }
        
        // Close serial port handle
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        
        std::cout << "Serial port closed" << std::endl;
    }
}

bool SerialPort::send(const std::vector<uint8_t>& data) {
    if (!isOpen()) {
        std::cerr << "Serial port not open, cannot send data" << std::endl;
        return false;
    }
    
    DWORD bytesWritten = 0;
    bool result = WriteFile(
        hSerial,
        data.data(),
        static_cast<DWORD>(data.size()),
        &bytesWritten,
        NULL
    );
    
    if (!result || bytesWritten != data.size()) {
        std::cerr << "Failed to send data, error code: " << GetLastError() << std::endl;
        return false;
    }
    
    return true;
}

void SerialPort::setReceiveCallback(std::function<void(const std::vector<uint8_t>&)> callback) {
    std::lock_guard<std::mutex> lock(mutex);
    receiveCallback = callback;
}

bool SerialPort::isOpen() const {
    return hSerial != INVALID_HANDLE_VALUE;
}

std::vector<std::string> SerialPort::getAvailablePorts() {
    std::vector<std::string> ports;
    
    // Try to open COM1-COM256
    for (int i = 1; i <= 256; i++) {
        std::stringstream ss;
        ss << "COM" << i;
        std::string portName = ss.str();
        
        // Try to open serial port
        std::string fullPortName = "\\\\.\\" + portName;
        HANDLE hTestSerial = CreateFileA(
            fullPortName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
        );
        
        if (hTestSerial != INVALID_HANDLE_VALUE) {
            // Serial port is available
            ports.push_back(portName);
            CloseHandle(hTestSerial);
        }
    }
    
    return ports;
}

void SerialPort::receiveThread() {
    const int bufferSize = 1024;
    uint8_t buffer[bufferSize];
    
    while (isRunning) {
        if (!isOpen()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        DWORD bytesRead = 0;
        bool result = ReadFile(
            hSerial,
            buffer,
            bufferSize,
            &bytesRead,
            NULL
        );
        
        if (result && bytesRead > 0) {
            std::vector<uint8_t> data(buffer, buffer + bytesRead);
            
            // Call callback function
            std::lock_guard<std::mutex> lock(mutex);
            if (receiveCallback) {
                receiveCallback(data);
            }
        }
        
        // Short sleep to avoid high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
} 