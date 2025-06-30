#include "protocol.h"
#include <algorithm>  // For std::max

// Convert statistics to string
std::string ProtocolStats::toString() const {
    std::stringstream ss;
    ss << "Protocol Statistics:\n";
    ss << "  Total Frames: " << totalFrames << "\n";
    ss << "  Valid Frames: " << validFrames << "\n";
    ss << "  CRC Errors: " << crcErrors << "\n";
    ss << "  Length Errors: " << lengthErrors << "\n";
    ss << "  Timeout Errors: " << timeoutErrors << "\n";
    ss << "  Buffer Overflows: " << bufferOverflows << "\n";
    ss << "  Invalid Frames: " << invalidFrames << "\n";
    ss << "  Escaped Sequence Errors: " << escapedSequenceErrors << "\n";
    ss << "  Bytes Rate: " << std::fixed << std::setprecision(2) << bytesPerSecond << " bytes/sec\n";
    ss << "  Frame Rate: " << std::fixed << std::setprecision(2) << framesPerSecond << " frames/sec\n";
    ss << "  Error Rate: " << std::fixed << std::setprecision(2) << (errorRate * 100) << "%";
    return ss.str();
}

// ProtocolFrame constructor
ProtocolFrame::ProtocolFrame() : start(FRAME_START), cmd(0), len(0), crc(0), stop(FRAME_STOP) {}

// Convert frame content to string (for debugging)
std::string ProtocolFrame::toString() const {
    std::stringstream ss;
    
    ss << "Frame Content:\n";
    ss << "  Start Byte: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(start) << "\n";
    
    ss << "  From: ";
    for (auto b : from) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    ss << "\n";
    
    ss << "  To: ";
    for (auto b : to) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    ss << "\n";
    
    ss << "  Flags: ";
    for (auto b : flags) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    ss << "\n";
    
    ss << "  Command: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd) << "\n";
    ss << "  Length: " << std::dec << len << "\n";
    
    ss << "  Data: ";
    for (auto b : data) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    ss << "\n";
    
    ss << "  CRC: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crc) << "\n";
    ss << "  Stop Byte: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(stop);
    
    return ss.str();
}

// RingBuffer constructor
RingBuffer::RingBuffer(size_t size) : 
    buffer(size), head(0), tail(0), size(size) {}

// Write a single byte
bool RingBuffer::write(uint8_t byte) {
    std::lock_guard<std::mutex> lock(mutex);
    size_t nextHead = (head + 1) % size;
    if (nextHead == tail) {
        return false; // Buffer is full
    }
    buffer[head] = byte;
    head = nextHead;
    return true;
}

// Write multiple bytes
size_t RingBuffer::write(const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(mutex);
    size_t count = 0;
    for (size_t i = 0; i < length; i++) {
        size_t nextHead = (head + 1) % size;
        if (nextHead == tail) {
            break; // Buffer is full
        }
        buffer[head] = data[i];
        head = nextHead;
        count++;
    }
    return count;
}

// Read a single byte
bool RingBuffer::read(uint8_t& byte) {
    std::lock_guard<std::mutex> lock(mutex);
    if (head == tail) {
        return false; // Buffer is empty
    }
    byte = buffer[tail];
    tail = (tail + 1) % size;
    return true;
}

// Read multiple bytes
size_t RingBuffer::read(uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(mutex);
    size_t count = 0;
    for (size_t i = 0; i < length; i++) {
        if (head == tail) {
            break; // Buffer is empty
        }
        data[i] = buffer[tail];
        tail = (tail + 1) % size;
        count++;
    }
    return count;
}

// Peek without removing data
bool RingBuffer::peek(uint8_t& byte, size_t offset) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex));
    // Calculate available data directly instead of calling available() to avoid nested locks
    size_t avail;
    if (head >= tail) {
        avail = head - tail;
    } else {
        avail = size - (tail - head);
    }
    
    if (avail <= offset) {
        return false;
    }
    byte = buffer[(tail + offset) % size];
    return true;
}

// Get available data amount
size_t RingBuffer::available() const {
    // Use a short lock to minimize contention
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex));
    // Calculate directly to avoid potential recursive locks
    if (head >= tail) {
        return head - tail;
    } else {
        return size - (tail - head);
    }
}

// Clear buffer
void RingBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex);
    head = tail = 0;
}

// Skip specified number of bytes
void RingBuffer::skip(size_t count) {
    std::lock_guard<std::mutex> lock(mutex);
    count = std::min(count, available());
    tail = (tail + count) % size;
}

// Find specified byte
int RingBuffer::findByte(uint8_t target) const {
    // First get a snapshot of the buffer to avoid nested locks
    std::vector<uint8_t> bufferSnapshot;
    size_t avail;
    
    {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(mutex));
        // Calculate available data directly
        if (head >= tail) {
            avail = head - tail;
        } else {
            avail = size - (tail - head);
        }
        
        // Create a snapshot of the buffer
        if (avail > 0) {
            bufferSnapshot.resize(avail);
            for (size_t i = 0; i < avail; i++) {
                bufferSnapshot[i] = buffer[(tail + i) % size];
            }
        }
    }
    
    // Now search in the snapshot without holding the lock
    for (size_t i = 0; i < bufferSnapshot.size(); i++) {
        if (bufferSnapshot[i] == target) {
            return static_cast<int>(i);
        }
    }
    
    return -1; // Not found
}

// ProtocolParser constructor
ProtocolParser::ProtocolParser(bool useEscaping) :
    state(ParseState::WAIT_START),
    lastActivityTime(std::chrono::steady_clock::now()),
    useEscaping(useEscaping),
    escapingActive(false),
    expectedFromLen(1),  // Default value, can be modified via configuration
    expectedToLen(1),    // Default value, can be modified via configuration
    expectedFlagsLen(1), // Default value, can be modified via configuration
    dataIndex(0) {}

// Set frame receive callback
void ProtocolParser::setFrameCallback(std::function<void(const ProtocolFrame&)> callback) {
    frameCallback = callback;
}

// Set error callback
void ProtocolParser::setErrorCallback(std::function<void(ProtocolError, const std::vector<uint8_t>&)> callback) {
    errorCallback = callback;
}

// Configure variable length field lengths
void ProtocolParser::configureFieldLengths(size_t fromLen, size_t toLen, size_t flagsLen) {
    expectedFromLen = fromLen;
    expectedToLen = toLen;
    expectedFlagsLen = flagsLen;
}

// Reset statistics
void ProtocolParser::resetStats() {
    stats = ProtocolStats();
}

// Update bandwidth statistics
void ProtocolParser::updateBandwidthStats(uint32_t bytesProcessed, uint32_t framesProcessed, double elapsedSeconds) {
    if (elapsedSeconds > 0) {
        // Update bytes per second rate
        stats.bytesPerSecond = bytesProcessed / elapsedSeconds;
        
        // Update frames per second rate
        stats.framesPerSecond = framesProcessed / elapsedSeconds;
    }
    
    // Update error rate based on current statistics
    if (stats.totalFrames > 0) {
        stats.errorRate = 1.0 - (static_cast<double>(stats.validFrames) / stats.totalFrames);
    }
}

// CRC-8 calculation
uint8_t ProtocolParser::calculateCRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xFF; // ensure result is within 0-255 range
            } else {
                crc = (crc << 1) & 0xFF; // ensure result is within 0-255 range
            }
        }
    }
    
    return crc;
}

// Reset parsing state
void ProtocolParser::resetParser() {
    state = ParseState::WAIT_START;
    currentFrame = ProtocolFrame();
    tempBuffer.clear();
    escapingActive = false;
    dataIndex = 0;
    lastActivityTime = std::chrono::steady_clock::now();
}

// Handle escaping character
bool ProtocolParser::handleEscaping(uint8_t& byte) {
    // If escaping is not enabled, just pass the byte through
    if (!useEscaping) {
        return true;
    }
    
    if (escapingActive) {
        // We've already seen an escape character, now check if this is a valid escaped byte
        if ((byte ^ ESCAPE_MASK) == FRAME_START || 
            (byte ^ ESCAPE_MASK) == FRAME_STOP || 
            (byte ^ ESCAPE_MASK) == ESCAPE_CHAR) {
            // Valid escaped byte - unescape it by applying XOR mask
            byte ^= ESCAPE_MASK;
            escapingActive = false;
            return true;
        } else {
            // Invalid escape sequence - report error
            std::vector<uint8_t> errorData = tempBuffer;
            errorData.push_back(byte);
            reportError(ProtocolError::ESCAPED_SEQUENCE_ERROR, errorData);
            // Instead of calling resynchronize here, return false to let caller handle it
            escapingActive = false; // Reset escaping state
            return false;
        }
    } else if (byte == ESCAPE_CHAR) {
        // Found an escape character - set flag and wait for next byte
        escapingActive = true;
        return false;  // Don't process this byte yet, need to see the next one
    }
    
    // Regular byte, no special processing needed
    return true;
}

// Calculate current frame CRC
uint8_t ProtocolParser::calculateFrameCRC() {
    try {
        // Prepare the CRC calculation data using the same order as in frame creation
        std::vector<uint8_t> crcData;
        
        // From field - already unescaped during parsing
        crcData.insert(crcData.end(), currentFrame.from.begin(), currentFrame.from.end());
        
        // To field - already unescaped during parsing
        crcData.insert(crcData.end(), currentFrame.to.begin(), currentFrame.to.end());
        
        // Flags field - already unescaped during parsing
        crcData.insert(crcData.end(), currentFrame.flags.begin(), currentFrame.flags.end());
        
        // Command byte - already unescaped during parsing
        crcData.push_back(currentFrame.cmd);
        
        // Length (little-endian)
        crcData.push_back(currentFrame.len & 0xFF);
        crcData.push_back((currentFrame.len >> 8) & 0xFF);
        
        // Data field - already unescaped during parsing
        crcData.insert(crcData.end(), currentFrame.data.begin(), currentFrame.data.end());
        
        // For debugging, we can add a function to print CRC input data
        // We'll leave this as a simple implementation that can be expanded later
        if (currentFrame.data.size() < 100) {  // Only print for small frames to avoid excessive output
            std::cout << "CRC calculation data: ";
            for (size_t i = 0; i < crcData.size(); ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crcData[i]) << " ";
            }
            std::cout << std::dec << std::endl;
        }
        
        // Calculate CRC on unescaped data
        uint8_t calculatedCRC = calculateCRC8(crcData);
        
        return calculatedCRC;
    } catch (const std::exception& e) {
        std::cerr << "Exception during CRC calculation: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "Unknown exception during CRC calculation" << std::endl;
        return 0;
    }
}

// Check timeout
bool ProtocolParser::checkTimeout() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActivityTime).count();
    if (elapsed > PARSE_TIMEOUT_MS && state != ParseState::WAIT_START) {
        reportError(ProtocolError::TIMEOUT_ERROR, tempBuffer);
        resetParser();
        return true;
    }
    return false;
}

// Report error
void ProtocolParser::reportError(ProtocolError error, const std::vector<uint8_t>& errorData) {
    // Error statistics are now updated in ProtocolHandler::handleError
    
    // Ensure callback exists and is called safely
    if (errorCallback) {
        try {
            errorCallback(error, errorData);
        } catch (...) {
            // Catch any possible exceptions to prevent crashes
            // In a real application, logging could be added here
        }
    }
}

// Successfully process a frame
void ProtocolParser::frameComplete() {
    // Frame statistics are now updated in ProtocolHandler::updateStats
    
    if (frameCallback) {
        frameCallback(currentFrame);
    }
    
    resetParser();
}

// Synchronously re-find frame header
void ProtocolParser::resynchronize() {
    try {
        // Reset parser state first
        resetParser();
        
        // Simple approach: just read and discard bytes until we find a start byte
        // This avoids complex buffer manipulation that might cause deadlocks
        uint8_t byte;
        bool foundStart = false;
        
        // Read up to RING_BUFFER_SIZE bytes to find a start byte
        for (size_t i = 0; i < RING_BUFFER_SIZE && !foundStart; i++) {
            if (rxBuffer.read(byte)) {
                if (byte == FRAME_START) {
                    // Found a start byte - put it back and return
                    // This will be processed in the next processBuffer call
                    rxBuffer.write(byte);
                    foundStart = true;
                }
                // Otherwise just discard the byte
            } else {
                // No more data to read
                break;
            }
        }
        
        // If we couldn't find a start byte, the buffer is now empty
        if (!foundStart) {
            rxBuffer.clear();
        }
    } catch (const std::exception& e) {
        // Catch exceptions to prevent crashes
        std::cerr << "Exception in resynchronize: " << e.what() << std::endl;
        rxBuffer.clear();
    } catch (...) {
        // Catch any other type of exception
        std::cerr << "Unknown exception in resynchronize" << std::endl;
        rxBuffer.clear();
    }
}

// Data input interface
void ProtocolParser::addData(const uint8_t* data, size_t length) {
    // Allow null data for forced processing of existing buffer
    if (data == nullptr && length == 0) {
        processBuffer();
        return;
    }
    
    if (data == nullptr || length == 0) {
        return;
    }
    
    size_t written = rxBuffer.write(data, length);
    if (written < length) {
        // Buffer overflow error - report it but don't increment counter here
        // The counter will be incremented in handleError
        std::vector<uint8_t> overflowInfo;
        // Add only one byte to indicate overflow, avoiding access to potentially invalid memory
        overflowInfo.push_back(static_cast<uint8_t>(length - written));
        reportError(ProtocolError::BUFFER_OVERFLOW, overflowInfo);
    }
    processBuffer();
}

// Process data in buffer
void ProtocolParser::processBuffer() {
    checkTimeout();
    
    while (rxBuffer.available() > 0) {
        uint8_t byte;
        lastActivityTime = std::chrono::steady_clock::now();
        
        switch (state) {
            case ParseState::WAIT_START:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (byte == FRAME_START) {
                    currentFrame.start = byte;
                    tempBuffer.push_back(byte);
                    state = ParseState::PARSE_FROM;
                }
                break;
                
            case ParseState::PARSE_FROM:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    currentFrame.from.push_back(byte);
                    
                    if (currentFrame.from.size() >= expectedFromLen) {
                        state = ParseState::PARSE_TO;
                    }
                } else {
                    // Escaping error occurred, need to resynchronize
                    resynchronize();
                    return;
                }
                break;
                
            case ParseState::PARSE_TO:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    currentFrame.to.push_back(byte);
                    
                    if (currentFrame.to.size() >= expectedToLen) {
                        state = ParseState::PARSE_FLAGS;
                    }
                } else {
                    // Escaping error occurred, need to resynchronize
                    resynchronize();
                    return;
                }
                break;
                
            case ParseState::PARSE_FLAGS:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    currentFrame.flags.push_back(byte);
                    
                    if (currentFrame.flags.size() >= expectedFlagsLen) {
                        state = ParseState::PARSE_CMD;
                    }
                } else {
                    // Escaping error occurred, need to resynchronize
                    resynchronize();
                    return;
                }
                break;
                
            case ParseState::PARSE_CMD:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    currentFrame.cmd = byte;
                    state = ParseState::PARSE_LEN;
                } else {
                    // Escaping error occurred, need to resynchronize
                    resynchronize();
                    return;
                }
                break;
                
            case ParseState::PARSE_LEN:
                // Need to handle each byte separately to properly handle potential escaping
                // First byte of length (low byte)
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    uint8_t lenLow = byte;
                    
                    // Second byte of length (high byte)
                    if (!rxBuffer.read(byte)) {
                        return;
                }
                
                    if (handleEscaping(byte)) {
                        tempBuffer.push_back(byte);
                        uint8_t lenHigh = byte;
                
                        // Calculate the length from both bytes
                currentFrame.len = (static_cast<uint16_t>(lenHigh) << 8) | lenLow;
                
                // Check length is reasonable - use the constant from protocol.h
                if (currentFrame.len > MAX_FRAME_SIZE) {
                    std::cerr << "Length error: received length " << currentFrame.len 
                              << " exceeds maximum frame size " << MAX_FRAME_SIZE << std::endl;
                    reportError(ProtocolError::LENGTH_ERROR, tempBuffer);
                    // Reset parser first to avoid lock issues during resynchronization
                    resetParser();
                    resynchronize();
                    return;
                }
                
                currentFrame.data.reserve(currentFrame.len);
                dataIndex = 0;
                state = ParseState::PARSE_DATA;
                                    } else {
                    // Escaping error in high byte of length
                    // Reset parser first to avoid lock issues
                    resetParser();
                    resynchronize();
                    return;
                }
                } else {
                    // Escaping error in low byte of length
                    // Reset parser first to avoid lock issues
                    resetParser();
                    resynchronize();
                    return;
                }
                break;
                
            case ParseState::PARSE_DATA:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                        tempBuffer.push_back(byte);
                        
                        // Only add to data field if we haven't reached the end of data
                        if (dataIndex < currentFrame.len) {
                            currentFrame.data.push_back(byte);
                            dataIndex++;
                            
                            if (dataIndex >= currentFrame.len) {
                                state = ParseState::PARSE_CRC;
                            }
                        }
                    } else {
                        // Escaping error occurred, need to resynchronize
                        // Reset parser first to avoid lock issues
                        resetParser();
                        resynchronize();
                        return;
                    }
                break;
                
            case ParseState::PARSE_CRC:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                if (handleEscaping(byte)) {
                    tempBuffer.push_back(byte);
                    currentFrame.crc = byte;
                    

                    
                    try {
                        // Verify CRC
                        uint8_t calculatedCRC = calculateFrameCRC();
                        if (calculatedCRC != currentFrame.crc) {
                            // Always print basic CRC error information
                            std::cerr << "CRC error: expected 0x" << std::hex << static_cast<int>(calculatedCRC) 
                                      << ", got 0x" << static_cast<int>(currentFrame.crc) << std::dec << std::endl;
                            
                            reportError(ProtocolError::CRC_ERROR, tempBuffer);
                            // Perform resynchronization after CRC error
                            resetParser(); // Reset parser state first to avoid potential lock issues
                            resynchronize();
                            return;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception during CRC verification: " << e.what() << std::endl;
                        reportError(ProtocolError::CRC_ERROR, tempBuffer);
                        resetParser(); // Reset parser state first to avoid potential lock issues
                        resynchronize();
                        return;
                    } catch (...) {
                        std::cerr << "Unknown exception during CRC verification" << std::endl;
                        reportError(ProtocolError::CRC_ERROR, tempBuffer);
                        resetParser(); // Reset parser state first to avoid potential lock issues
                        resynchronize();
                        return;
                    }
                    
                    state = ParseState::PARSE_STOP;
                }
                break;
                
            case ParseState::PARSE_STOP:
                if (!rxBuffer.read(byte)) {
                    return;
                }
                
                tempBuffer.push_back(byte);
                currentFrame.stop = byte;
                
                if (byte == FRAME_STOP) {
                    frameComplete();
                } else {
                    reportError(ProtocolError::INVALID_FRAME, tempBuffer);
                    // Reset parser first to avoid lock issues
                    resetParser();
                    resynchronize();
                }
                break;
        }
    }
}

// ProtocolHandler constructor
ProtocolHandler::ProtocolHandler(bool useEscaping) : 
    parser(useEscaping), 
    lastStatsUpdate(std::chrono::steady_clock::now()),
    bytesSinceLastUpdate(0),
    framesSinceLastUpdate(0),
    debugMode(false) {
    
    // Set frame receive callback
    parser.setFrameCallback([this](const ProtocolFrame& frame) {
        handleFrame(frame);
        framesSinceLastUpdate++;
    });
    
    // Set error callback
    parser.setErrorCallback([this](ProtocolError error, const std::vector<uint8_t>& errorData) {
        handleError(error, errorData);
    });
    
    // Initialize some example device names
    deviceNames[0x01] = "Main Controller";
    deviceNames[0x02] = "Sensor Node 1";
    deviceNames[0x03] = "Sensor Node 2";
    deviceNames[0x04] = "Actuator Node";
    deviceNames[0xFF] = "Broadcast";
}

// Configure variable length field lengths
void ProtocolHandler::configureFieldLengths(size_t fromLen, size_t toLen, size_t flagsLen) {
    parser.configureFieldLengths(fromLen, toLen, flagsLen);
}

// Data input interface
void ProtocolHandler::processData(const uint8_t* data, size_t length) {
    // Minimal debug output - only print first few bytes
    if (debugMode && data != nullptr && length > 0) {
        std::cout << "Processing " << length << " bytes" << std::endl;
    }
    
    // Only add data to buffer if we have actual data
    if (data != nullptr && length > 0) {
        bytesSinceLastUpdate += length;
        parser.addData(data, length);
    } else {
        // Force update statistics even with empty data
        parser.addData(nullptr, 0);
    }
    
    // Always update statistics
    updateStats();
}

// Reset statistics
void ProtocolHandler::resetStats() {
    parser.resetStats();
    bytesSinceLastUpdate = 0;
    framesSinceLastUpdate = 0;
    lastStatsUpdate = std::chrono::steady_clock::now();
}

// Set frame receive callback
void ProtocolHandler::setFrameReceivedCallback(std::function<void(const ProtocolFrame&, const std::string&)> callback) {
    frameReceivedCallback = callback;
}

// Set debug mode
void ProtocolHandler::setDebugMode(bool enabled) {
    debugMode = enabled;
    
    // Simple notification without excessive details
    if (enabled) {
        std::cout << "Debug mode enabled" << std::endl;
    } else {
        std::cout << "Debug mode disabled" << std::endl;
    }
}

// Get device name
std::string ProtocolHandler::getDeviceName(uint8_t deviceId) const {
    auto it = deviceNames.find(deviceId);
    if (it != deviceNames.end()) {
        return it->second;
    }
    return "Unknown Device(" + std::to_string(deviceId) + ")";
}

// Create a frame
std::vector<uint8_t> ProtocolHandler::createFrame(
    uint8_t fromDevice,
    uint8_t toDevice,
    uint8_t cmd,
    const std::vector<uint8_t>& data,
    const std::vector<uint8_t>& flags,
    bool useEscaping
) {
    try {
        // First prepare the raw data for CRC calculation (before any escaping)
        std::vector<uint8_t> crcData;
        crcData.push_back(fromDevice);  // From field
        crcData.push_back(toDevice);    // To field
        crcData.insert(crcData.end(), flags.begin(), flags.end());  // Flags field
        crcData.push_back(cmd);         // Command byte
        
        // Length (little-endian)
        uint16_t len = static_cast<uint16_t>(data.size());
        crcData.push_back(len & 0xFF);
        crcData.push_back((len >> 8) & 0xFF);
        
        // Data
        crcData.insert(crcData.end(), data.begin(), data.end());
        
                                // Calculate CRC on raw data before escaping
                        uint8_t crc = calculateCRC8(crcData);
                        
                        // Print CRC calculation data in debug mode
                        // We can't access debugMode in a static function, so we'll leave this commented out
                        // In a real implementation, we would pass the debug flag as a parameter or use a global setting
                        /*
                        if (debugEnabled) {
                            std::cout << "Frame creation - CRC input data: ";
                            for (size_t i = 0; i < crcData.size(); ++i) {
                                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crcData[i]) << " ";
                            }
                            std::cout << std::dec << std::endl;
                            std::cout << "Frame creation - Calculated CRC: 0x" << std::hex << static_cast<int>(crc) << std::dec << std::endl;
                        }
                        */
        
        // Now build the frame with escaping
        std::vector<uint8_t> frame;
        frame.push_back(FRAME_START);  // Start byte is never escaped
        
        // Add From field with escaping
        addByteWithEscaping(frame, fromDevice, useEscaping);
        
        // Add To field with escaping
        addByteWithEscaping(frame, toDevice, useEscaping);
        
        // Add Flags field with escaping
        for (uint8_t b : flags) {
            addByteWithEscaping(frame, b, useEscaping);
        }
        
        // Add command byte with escaping
        addByteWithEscaping(frame, cmd, useEscaping);
        
        // Add length (little-endian) with escaping
        addByteWithEscaping(frame, len & 0xFF, useEscaping);
        addByteWithEscaping(frame, (len >> 8) & 0xFF, useEscaping);
        
        // Add data with escaping
        for (uint8_t b : data) {
            addByteWithEscaping(frame, b, useEscaping);
        }
        
        // Add CRC with escaping
        addByteWithEscaping(frame, crc, useEscaping);
        
        // Add stop byte (never escaped)
        frame.push_back(FRAME_STOP);
        
        return frame;
    } catch (const std::exception& e) {
        // Catch exceptions to prevent crashes
        std::cerr << "Exception in createFrame: " << e.what() << std::endl;
        return {};
    } catch (...) {
        // Catch any other type of exception
        std::cerr << "Unknown exception in createFrame" << std::endl;
        return {};
    }
}

// CRC-8 calculation (static version, for frame creation)
uint8_t ProtocolHandler::calculateCRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xFF; // ensure result is within 0-255 range
            } else {
                crc = (crc << 1) & 0xFF; // ensure result is within 0-255 range
            }
        }
    }
    
    return crc;
}

// Create information class command
std::vector<uint8_t> ProtocolHandler::createInfoCommand(
    uint8_t fromDevice,
    uint8_t toDevice,
    InfoCommand subCmd,
    const std::vector<uint8_t>& data
) {
    uint8_t cmd = (static_cast<uint8_t>(CommandType::INFO) << 6) | static_cast<uint8_t>(subCmd);
    return createFrame(fromDevice, toDevice, cmd, data);
}

// Create control class command
std::vector<uint8_t> ProtocolHandler::createControlCommand(
    uint8_t fromDevice,
    uint8_t toDevice,
    ControlCommand subCmd,
    const std::vector<uint8_t>& data
) {
    uint8_t cmd = (static_cast<uint8_t>(CommandType::CONTROL) << 6) | static_cast<uint8_t>(subCmd);
    return createFrame(fromDevice, toDevice, cmd, data);
}

// Create parameter class command
std::vector<uint8_t> ProtocolHandler::createParameterCommand(
    uint8_t fromDevice,
    uint8_t toDevice,
    uint8_t paramId,
    bool isSet,
    const std::vector<uint8_t>& data
) {
    uint8_t subCmd = paramId & 0x1F;
    if (isSet) {
        subCmd |= static_cast<uint8_t>(ParamCommand::SET_FLAG);
    }
    uint8_t cmd = (static_cast<uint8_t>(CommandType::PARAMETER) << 6) | subCmd;
    return createFrame(fromDevice, toDevice, cmd, data);
}

// Create special class command
std::vector<uint8_t> ProtocolHandler::createSpecialCommand(
    uint8_t fromDevice,
    uint8_t toDevice,
    SpecialCommand subCmd,
    const std::vector<uint8_t>& data
) {
    uint8_t cmd = (static_cast<uint8_t>(CommandType::SPECIAL) << 6) | static_cast<uint8_t>(subCmd);
    return createFrame(fromDevice, toDevice, cmd, data);
}

// Handle valid frame
void ProtocolHandler::handleFrame(const ProtocolFrame& frame) {
    // Extract command type and subtype
    uint8_t cmdType = (frame.cmd >> 6) & 0x03;
    uint8_t cmdSubType = frame.cmd & 0x3F;
    
    std::string messageInfo;
    
    switch (cmdType) {
        case 0: // Information class
            messageInfo = handleInfoCommand(frame, cmdSubType);
            break;
        case 1: // Command class
            messageInfo = handleControlCommand(frame, cmdSubType);
            break;
        case 2: // Parameter class
            messageInfo = handleParameterCommand(frame, cmdSubType);
            break;
        case 3: // Special class
            messageInfo = handleSpecialCommand(frame, cmdSubType);
            break;
        default:
            messageInfo = "Unknown Command Type";
            break;
    }
    
    // Increment frame counter for statistics
    framesSinceLastUpdate++;
    
    if (debugMode) {
        std::cout << "Received Frame: " << messageInfo << std::endl;
        std::cout << frame.toString() << std::endl;
    }
    
    if (frameReceivedCallback) {
        frameReceivedCallback(frame, messageInfo);
    }
}

// Handle information class command
std::string ProtocolHandler::handleInfoCommand(const ProtocolFrame& frame, uint8_t subType) {
    std::stringstream ss;
    ss << "Info Command: ";
    
    switch (subType) {
        case static_cast<uint8_t>(InfoCommand::STATUS):
            ss << "Status Info";
            break;
        case static_cast<uint8_t>(InfoCommand::VERSION):
            ss << "Version Info";
            if (frame.data.size() >= 3) {
                ss << " v" << static_cast<int>(frame.data[0]) << "."
                   << static_cast<int>(frame.data[1]) << "."
                   << static_cast<int>(frame.data[2]);
            }
            break;
        case static_cast<uint8_t>(InfoCommand::DEVICE_ID):
            ss << "Device ID";
            break;
        case static_cast<uint8_t>(InfoCommand::STATS):
            ss << "Statistics";
            break;
        default:
            ss << "Unknown Sub-command(0x" << std::hex << static_cast<int>(subType) << ")";
            break;
    }
    
    return ss.str();
}

// Handle command class command
std::string ProtocolHandler::handleControlCommand(const ProtocolFrame& frame, uint8_t subType) {
    std::stringstream ss;
    ss << "Control Command: ";
    
    switch (subType) {
        case static_cast<uint8_t>(ControlCommand::RESET):
            ss << "Reset";
            break;
        case static_cast<uint8_t>(ControlCommand::START):
            ss << "Start";
            break;
        case static_cast<uint8_t>(ControlCommand::STOP):
            ss << "Stop";
            break;
        case static_cast<uint8_t>(ControlCommand::CALIBRATE):
            ss << "Calibrate";
            break;
        default:
            ss << "Unknown Sub-command(0x" << std::hex << static_cast<int>(subType) << ")";
            break;
    }
    
    return ss.str();
}

// Handle parameter class command
std::string ProtocolHandler::handleParameterCommand(const ProtocolFrame& frame, uint8_t subType) {
    std::stringstream ss;
    ss << "Parameter Command: ";
    
    // Check if it's a query or modify parameter
    bool isSet = (subType & static_cast<uint8_t>(ParamCommand::SET_FLAG)) != 0;
    uint8_t paramId = subType & 0x1F;
    
    if (isSet) {
        ss << "Set Parameter #" << static_cast<int>(paramId);
    } else {
        if (paramId == static_cast<uint8_t>(ParamCommand::GET_ALL)) {
            ss << "Query All Parameters";
        } else {
            ss << "Query Parameter #" << static_cast<int>(paramId);
        }
    }
    
    return ss.str();
}

// Handle special class command
std::string ProtocolHandler::handleSpecialCommand(const ProtocolFrame& frame, uint8_t subType) {
    std::stringstream ss;
    ss << "Special Command: ";
    
    switch (subType) {
        case static_cast<uint8_t>(SpecialCommand::ECHO):
            ss << "Echo Test";
            break;
        case static_cast<uint8_t>(SpecialCommand::LOOPBACK):
            ss << "Loopback Test";
            break;
        case static_cast<uint8_t>(SpecialCommand::DEBUG):
            ss << "Debug Command";
            break;
        default:
            ss << "Unknown Sub-command(0x" << std::hex << static_cast<int>(subType) << ")";
            break;
    }
    
    return ss.str();
}

// Handle error
void ProtocolHandler::handleError(ProtocolError error, const std::vector<uint8_t>& errorData) {
    // Update total frames count for error rate calculation
    ProtocolStats& stats = const_cast<ProtocolStats&>(parser.getStats());
    stats.totalFrames++;
    
    // Update specific error counters
    switch (error) {
        case ProtocolError::CRC_ERROR:
            stats.crcErrors++;
            break;
        case ProtocolError::LENGTH_ERROR:
            stats.lengthErrors++;
            break;
        case ProtocolError::TIMEOUT_ERROR:
            stats.timeoutErrors++;
            break;
        case ProtocolError::BUFFER_OVERFLOW:
            stats.bufferOverflows++;
            break;
        case ProtocolError::INVALID_FRAME:
            stats.invalidFrames++;
            break;
        case ProtocolError::ESCAPED_SEQUENCE_ERROR:
            stats.escapedSequenceErrors++;
            break;
        default:
            break;
    }
    
    // Update error rate
    if (stats.totalFrames > 0) {
        stats.errorRate = 1.0 - (static_cast<double>(stats.validFrames) / stats.totalFrames);
    }
    
    if (debugMode) {
        std::cout << "Protocol Error: ";
        switch (error) {
            case ProtocolError::CRC_ERROR:
                std::cout << "CRC Check Error";
                break;
            case ProtocolError::LENGTH_ERROR:
                std::cout << "Length Error";
                break;
            case ProtocolError::TIMEOUT_ERROR:
                std::cout << "Timeout Error";
                break;
            case ProtocolError::BUFFER_OVERFLOW:
                std::cout << "Buffer Overflow";
                break;
            case ProtocolError::INVALID_FRAME:
                std::cout << "Invalid Frame";
                break;
            case ProtocolError::ESCAPED_SEQUENCE_ERROR:
                std::cout << "Escaped Sequence Error";
                break;
            default:
                std::cout << "Unknown Error";
                break;
        }
        std::cout << std::endl;
        
        // Safely print error data
        std::cout << "Error Data: ";
        try {
            for (size_t i = 0; i < errorData.size() && i < 20; ++i) { // Limit print quantity to avoid excessive output
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(errorData[i]) << " ";
            }
            if (errorData.size() > 20) {
                std::cout << "... (truncated)";
            }
        } catch (...) {
            std::cout << "(Error accessing error data)";
        }
        std::cout << std::endl;
    }
}

// Update statistics
void ProtocolHandler::updateStats() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastStatsUpdate).count();
    
    // Always update statistics if we have new frames or sufficient time has passed
    if (elapsed >= 1 || framesSinceLastUpdate > 0) {
        // If elapsed time is too small, use a minimum value to avoid division by zero
        double elapsedTime = (elapsed < 1) ? 1 : elapsed;
        
        // Update bandwidth statistics
        parser.updateBandwidthStats(bytesSinceLastUpdate, framesSinceLastUpdate, elapsedTime);
        
        // Directly update frame counts in parser stats to ensure they're updated
        if (framesSinceLastUpdate > 0) {
            ProtocolStats& stats = const_cast<ProtocolStats&>(parser.getStats());
            stats.totalFrames += framesSinceLastUpdate;
            stats.validFrames += framesSinceLastUpdate;
        }
        
        // Reset counters and update timestamp if sufficient time has passed
        if (elapsed >= 1) {
            bytesSinceLastUpdate = 0;
            framesSinceLastUpdate = 0;
            lastStatsUpdate = now;
        }
    }
}

// Add byte with escaping to frame
void ProtocolHandler::addByteWithEscaping(std::vector<uint8_t>& frame, uint8_t byte, bool useEscaping) {
    try {
        if (useEscaping && (byte == FRAME_START || byte == FRAME_STOP || byte == ESCAPE_CHAR)) {
            frame.push_back(ESCAPE_CHAR);
            frame.push_back(byte ^ ESCAPE_MASK);
        } else {
            frame.push_back(byte);
        }
    } catch (const std::exception& e) {
        // Catch exceptions to prevent crashes
        std::cerr << "Exception in addByteWithEscaping: " << e.what() << std::endl;
    } catch (...) {
        // Catch any other type of exception
        std::cerr << "Unknown exception in addByteWithEscaping" << std::endl;
    }
}
