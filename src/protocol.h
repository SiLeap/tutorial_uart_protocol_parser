#pragma once

#include <cstdint>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <map>
#include <thread>
#include <atomic>

// Protocol constants
constexpr uint8_t FRAME_START = 0xAA;
constexpr uint8_t FRAME_STOP = 0x55;
constexpr uint8_t ESCAPE_CHAR = 0x7D;
constexpr uint8_t ESCAPE_MASK = 0x20;
constexpr size_t MAX_FRAME_SIZE = 1024*1024;
constexpr size_t RING_BUFFER_SIZE = 65536;
constexpr uint32_t PARSE_TIMEOUT_MS = 500;

// Error codes
enum class ProtocolError {
    NONE,
    CRC_ERROR,
    LENGTH_ERROR,
    TIMEOUT_ERROR,
    BUFFER_OVERFLOW,
    INVALID_FRAME,
    ESCAPED_SEQUENCE_ERROR
};

// Command types
enum class CommandType : uint8_t {
    INFO = 0,      // Information commands
    CONTROL = 1,   // Control commands
    PARAMETER = 2, // Parameter commands
    SPECIAL = 3    // Special commands
};

// Info sub-commands
enum class InfoCommand : uint8_t {
    STATUS = 0,    // Status information
    VERSION = 1,   // Version information
    DEVICE_ID = 2, // Device ID
    STATS = 3      // Statistics
};

// Control sub-commands
enum class ControlCommand : uint8_t {
    RESET = 0,     // Reset
    START = 1,     // Start
    STOP = 2,      // Stop
    CALIBRATE = 3  // Calibrate
};

// Parameter sub-commands
enum class ParamCommand : uint8_t {
    GET_ALL = 0x1F,  // Get all parameters
    SET_FLAG = 0x20  // Set parameter flag
};

// Special sub-commands
enum class SpecialCommand : uint8_t {
    ECHO = 0,      // Echo test
    LOOPBACK = 1,  // Loopback test
    DEBUG = 2      // Debug command
};

// Statistics structure
struct ProtocolStats {
    uint32_t totalFrames;
    uint32_t validFrames;
    uint32_t crcErrors;
    uint32_t lengthErrors;
    uint32_t timeoutErrors;
    uint32_t bufferOverflows;
    uint32_t invalidFrames;
    uint32_t escapedSequenceErrors;
    double bytesPerSecond;
    double framesPerSecond;
    double errorRate;
    
    ProtocolStats() : 
        totalFrames(0), validFrames(0), crcErrors(0), lengthErrors(0),
        timeoutErrors(0), bufferOverflows(0), invalidFrames(0), 
        escapedSequenceErrors(0), bytesPerSecond(0), framesPerSecond(0),
        errorRate(0) {}
        
    // Convert statistics to string
    std::string toString() const;
};

// Protocol frame structure
struct ProtocolFrame {
    uint8_t start;
    std::vector<uint8_t> from;
    std::vector<uint8_t> to;
    std::vector<uint8_t> flags;
    uint8_t cmd;
    uint16_t len;
    std::vector<uint8_t> data;
    uint8_t crc;
    uint8_t stop;
    
    ProtocolFrame();
    
    // Convert frame content to string (for debugging)
    std::string toString() const;
};

// Ring buffer class
class RingBuffer {
private:
    std::vector<uint8_t> buffer;
    size_t head;
    size_t tail;
    size_t size;
    std::mutex mutex;
    
public:
    RingBuffer(size_t size = RING_BUFFER_SIZE);
    
    // Write a single byte
    bool write(uint8_t byte);
    
    // Write multiple bytes
    size_t write(const uint8_t* data, size_t length);
    
    // Read a single byte
    bool read(uint8_t& byte);
    
    // Read multiple bytes
    size_t read(uint8_t* data, size_t length);
    
    // Peek without removing byte
    bool peek(uint8_t& byte, size_t offset = 0) const;
    
    // Get available bytes
    size_t available() const;
    
    // Clear buffer
    void clear();
    
    // Skip specified number of bytes
    void skip(size_t count);
    
    // Find specific byte position
    int findByte(uint8_t target) const;
};

// Protocol parser class
class ProtocolParser {
private:
    enum class ParseState {
        WAIT_START,
        PARSE_FROM,
        PARSE_TO,
        PARSE_FLAGS,
        PARSE_CMD,
        PARSE_LEN,
        PARSE_DATA,
        PARSE_CRC,
        PARSE_STOP
    };
    
    RingBuffer rxBuffer;
    ParseState state;
    ProtocolFrame currentFrame;
    std::vector<uint8_t> tempBuffer;
    std::chrono::steady_clock::time_point lastActivityTime;
    ProtocolStats stats;
    bool useEscaping;
    bool escapingActive;
    size_t expectedFromLen;
    size_t expectedToLen;
    size_t expectedFlagsLen;
    size_t dataIndex;
    
    std::function<void(const ProtocolFrame&)> frameCallback;
    std::function<void(ProtocolError, const std::vector<uint8_t>&)> errorCallback;
    
    // Calculate CRC8 checksum
    uint8_t calculateCRC8(const std::vector<uint8_t>& data);
    
    // Reset parser state
    void resetParser();
    
    // Handle escape character
    bool handleEscaping(uint8_t& byte);
    
    // Calculate frame CRC
    uint8_t calculateFrameCRC();
    
    // Check for timeout
    bool checkTimeout();
    
    // Report error
    void reportError(ProtocolError error, const std::vector<uint8_t>& errorData);
    
    // Frame complete handler
    void frameComplete();
    
    // Resynchronize
    void resynchronize();
    
public:
    ProtocolParser(bool useEscaping = true);
    
    // Set frame receive callback
    void setFrameCallback(std::function<void(const ProtocolFrame&)> callback);
    
    // Set error callback
    void setErrorCallback(std::function<void(ProtocolError, const std::vector<uint8_t>&)> callback);
    
    // Configure field lengths
    void configureFieldLengths(size_t fromLen, size_t toLen, size_t flagsLen);
    
    // Reset statistics
    void resetStats();
    
    // Update bandwidth statistics
    void updateBandwidthStats(uint32_t bytesProcessed, uint32_t framesProcessed, double elapsedSeconds);
    
    // Add data to buffer
    void addData(const uint8_t* data, size_t length);
    
    // Process buffer data
    void processBuffer();
    
    // Get statistics
    const ProtocolStats& getStats() const { return stats; }
};

// Protocol handler class
class ProtocolHandler {
private:
    ProtocolParser parser;
    std::chrono::steady_clock::time_point lastStatsUpdate;
    uint32_t bytesSinceLastUpdate;
    uint32_t framesSinceLastUpdate;
    bool debugMode;
    std::map<uint8_t, std::string> deviceNames;
    std::function<void(const ProtocolFrame&, const std::string&)> frameReceivedCallback;
    
public:
    ProtocolHandler(bool useEscaping = true);
    
    // Configure field lengths
    void configureFieldLengths(size_t fromLen, size_t toLen, size_t flagsLen);
    
    // Data input interface
    void processData(const uint8_t* data, size_t length);
    
    // Reset statistics
    void resetStats();
    
    // Set frame received callback
    void setFrameReceivedCallback(std::function<void(const ProtocolFrame&, const std::string&)> callback);
    
    // Set debug mode
    void setDebugMode(bool enabled);
    
    // Get debug mode status
    bool isDebugMode() const { return debugMode; }
    
    // Get device name
    std::string getDeviceName(uint8_t deviceId) const;
    
    // Get statistics
    const ProtocolStats& getStats() const { return parser.getStats(); }
    
    // Create a frame
    static std::vector<uint8_t> createFrame(
        uint8_t fromDevice,
        uint8_t toDevice,
        uint8_t cmd,
        const std::vector<uint8_t>& data = {},
        const std::vector<uint8_t>& flags = {0x00},
        bool useEscaping = true
    );
    
    // Calculate CRC8 (static version for frame creation)
    static uint8_t calculateCRC8(const std::vector<uint8_t>& data);
    
    // Create info command
    static std::vector<uint8_t> createInfoCommand(
        uint8_t fromDevice,
        uint8_t toDevice,
        InfoCommand subCmd,
        const std::vector<uint8_t>& data = {}
    );
    
    // Create control command
    static std::vector<uint8_t> createControlCommand(
        uint8_t fromDevice,
        uint8_t toDevice,
        ControlCommand subCmd,
        const std::vector<uint8_t>& data = {}
    );
    
    // Create parameter command
    static std::vector<uint8_t> createParameterCommand(
        uint8_t fromDevice,
        uint8_t toDevice,
        uint8_t paramId,
        bool isSet,
        const std::vector<uint8_t>& data = {}
    );
    
    // Create special command
    static std::vector<uint8_t> createSpecialCommand(
        uint8_t fromDevice,
        uint8_t toDevice,
        SpecialCommand subCmd,
        const std::vector<uint8_t>& data = {}
    );
    
private:
    // Handle valid frame
    void handleFrame(const ProtocolFrame& frame);
    
    // Handle info command
    std::string handleInfoCommand(const ProtocolFrame& frame, uint8_t subType);
    
    // Handle control command
    std::string handleControlCommand(const ProtocolFrame& frame, uint8_t subType);
    
    // Handle parameter command
    std::string handleParameterCommand(const ProtocolFrame& frame, uint8_t subType);
    
    // Handle special command
    std::string handleSpecialCommand(const ProtocolFrame& frame, uint8_t subType);
    
    // Handle error
    void handleError(ProtocolError error, const std::vector<uint8_t>& errorData);
    
    // Update statistics
    void updateStats();
    
    // Add byte with escaping to frame
    static void addByteWithEscaping(std::vector<uint8_t>& frame, uint8_t byte, bool useEscaping);
}; 