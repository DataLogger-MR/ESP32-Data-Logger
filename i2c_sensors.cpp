#include "i2c_sensors.h"
#include "globals.h"
#include "session_manager.h"
#include "file_manager.h"
#include "utils.h"
#include "i2c_config.h"
#include <map>
#include <memory>   // for std::unique_ptr (optional, we'll use raw pointers)

// ================ GLOBALS ================
I2CSensorData sensorData;

// I2C device instances - These are on main bus, NOT on multiplexer
RTC_DS3231 rtc;
Adafruit_MCP23X17 mcp23017;
Adafruit_ADS1115 ads1115_1;  // Address 0x48
Adafruit_ADS1115 ads1115_2;  // Address 0x49

// I2C multiplexer flags
bool useMultiplexer = false;
bool hasSecondaryMux = false;

// MCP9600 address
#define MCP9600_ADDR 0x60

// Structure to define MCP9600 mapping
struct HardcodedMCP9600 {
    uint8_t mainChannel;
    int8_t secondaryChannel;
    uint8_t address;
    String name;
    int instanceIndex;
    bool valid;
};

// Configure based on your test results - MCP9600 on Main Ch0, Secondary Ch2
HardcodedMCP9600 hardcodedMCP9600s[] = {
    // Format: {mainChannel, secondaryChannel, address, "SignalName", instanceIndex, valid}
    
    // First 8: direct on main channels 0-7
    //{0, -1, 0x60, "TC1", 0, false},
    {1, -1, 0x60, "TC2", 1, false},
    {2, -1, 0x60, "TC3", 2, false},
    {3, -1, 0x60, "TC4", 3, false},
    {4, -1, 0x60, "TC5", 4, false},
    {5, -1, 0x60, "TC6", 5, false},
    {6, -1, 0x60, "TC7", 6, false},
    {7, -1, 0x60, "TC8", 7, false},
    // Next 4: secondary mux on main channel 0, secondary channels 0-3
    {0, 0, 0x60, "TC9", 8, false},
    {0, 1, 0x60, "TC10", 9, false},
    {0, 2, 0x60, "TC11", 10, false},
    {0, 3, 0x60, "TC12", 11, false},
    {0, 4, 0x60, "TC1", 0, false},
};

const int HARDCODED_MCP9600_COUNT = sizeof(hardcodedMCP9600s) / sizeof(hardcodedMCP9600s[0]);

// ================ HELPER FUNCTIONS FOR MCP9600 ================
// I2C transmission with timeout (milliseconds)
static bool i2cWriteWithTimeout(uint8_t addr, const uint8_t* data, size_t len, int timeoutMs) {
    unsigned long start = millis();
    Wire.beginTransmission(addr);
    if (data && len) {
        size_t written = Wire.write(data, len);
        if (written != len) return false;
    }
    uint8_t error = Wire.endTransmission();
    if (error == 0) return true;
    // Wait a bit and retry once
    while (millis() - start < (unsigned long)timeoutMs) {
        delay(1);
        Wire.beginTransmission(addr);
        if (data && len) Wire.write(data, len);
        error = Wire.endTransmission();
        if (error == 0) return true;
    }
    return false;
}

// ================ SIMPLIFIED MUX CONTROL FUNCTIONS (Based on working standalone code) ================

// Simple function to select a channel on a specific mux
static bool selectMuxChannel(uint8_t muxAddr, uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(muxAddr);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

// Simple function to disable a specific mux (close all channels)
static void disableMux(uint8_t muxAddr) {
    Wire.beginTransmission(muxAddr);
    Wire.write(0x00);
    Wire.endTransmission();
}

// Select main mux channel (and disable secondary mux if needed)
bool selectMainMuxChannel(uint8_t channel) {
    if (!useMultiplexer || channel > 7) return false;
    
    // First, disable secondary mux if it exists to ensure isolation
    if (hasSecondaryMux) {
        disableMux(PCA9548A_ADDR_SECONDARY);
    }
    
    // Select the main channel
    return selectMuxChannel(PCA9548A_ADDR_MAIN, channel);
}

// Select secondary mux channel (main mux must already be on channel 0)
bool selectSecondaryMuxChannel(uint8_t channel) {
    if (!hasSecondaryMux || channel > 7) return false;
    
    // Select secondary channel
    return selectMuxChannel(PCA9548A_ADDR_SECONDARY, channel);
}

// Select complete I2C path (main + optional secondary)
bool selectI2CPath(uint8_t mainChannel, int8_t secondaryChannel) {
    // Select main channel first
    if (!selectMainMuxChannel(mainChannel)) return false;
    
    // If secondary channel specified, select it
    if (secondaryChannel >= 0 && hasSecondaryMux) {
        delayMicroseconds(50);
        if (!selectSecondaryMuxChannel(secondaryChannel)) return false;
    }
    return true;
}

// Reset I2C path - disable both muxes
void resetI2CPath() {
    if (useMultiplexer) {
        disableMux(PCA9548A_ADDR_MAIN);
        if (hasSecondaryMux) {
            disableMux(PCA9548A_ADDR_SECONDARY);
        }
    }
}

// Check if MCP9600 is present on current path (based on working standalone code)
static bool isMCP9600Present() {
    Wire.beginTransmission(MCP9600_ADDR);
    Wire.write(0x20);  // Read device ID register
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MCP9600_ADDR, (uint8_t)2) != 2) return false;
    uint8_t highByte = Wire.read();
    Wire.read();
    return (highByte == 0x40);  // MCP9600 device ID
}

// Read temperature from MCP9600 (based on working standalone code)
static float readMCP9600Temperature() {
    Wire.beginTransmission(MCP9600_ADDR);
    Wire.write(0x00);  // Temperature register
    if (Wire.endTransmission(false) != 0) return NAN;
    if (Wire.requestFrom((uint8_t)MCP9600_ADDR, (uint8_t)2) != 2) return NAN;
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    return raw * 0.0625f;  // Each count = 0.0625°C
}

// Select the correct path for a thermocouple (simplified)
static bool selectMCP9600Path(const HardcodedMCP9600& tc) {
    if (tc.secondaryChannel >= 0) {
        // For secondary mux: main channel 0, then secondary channel
        if (!selectMainMuxChannel(tc.mainChannel)) {
            return false;
        }
        delayMicroseconds(50);
        if (!selectSecondaryMuxChannel(tc.secondaryChannel)) {
            return false;
        }
    } else {
        // Direct on main channel
        if (!selectMainMuxChannel(tc.mainChannel)) {
            return false;
        }
    }
    return true;
}

// ================ INITIALIZATION ================
void initI2CSensors() {

    // IMPORTANT: Reset I2C bus first
    Wire.end();
    delay(100);
    Wire.begin();
    Wire.setClock(100000);
    delay(100);

    // Add pull-up enable for I2C pins
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    delay(100);

    // Quick bus test
    Serial.println("Testing I2C bus...");
    uint8_t dummy = 0;
    if (i2cWriteWithTimeout(0x70, &dummy, 0, 10)) {
        Serial.println("✅ I2C bus OK (0x70 responded)");
    } else {
      //  Serial.println("⚠️ I2C bus error: no device at 0x70 (check wiring)");
    }

    // Clear all data structures
    memset(&sensorData, 0, sizeof(I2CSensorData));
    sensorData.lastScanTime = millis();

    // CRITICAL: First, check for multiplexer directly without any other interference
    Serial.println("\n--- Direct Multiplexer Detection ---");
    
    // Reset I2C bus
    Wire.end();
    delay(50);
    Wire.begin();
    delay(50);
    
    // Check for main multiplexer at 0x70
    if (i2cWriteWithTimeout(0x70, nullptr, 0, 10)) {
        Serial.println("  ✅ Main PCA9548A Multiplexer found at 0x70");
        useMultiplexer = true;
        sensorData.pca9548a_present = true;
        
        // Now check for secondary multiplexer on main channel 0
        if (selectMainMuxChannel(0)) {
            delay(10);
            if (i2cWriteWithTimeout(0x74, nullptr, 0, 10)) {
                Serial.println("  ✅ Secondary PCA9548A Multiplexer found at 0x74 (on Main Ch0)");
                hasSecondaryMux = true;
                
                // Check for MCP9600 on secondary channel 2
                if (selectSecondaryMuxChannel(2)) {
                    delay(10);
                    if (isMCP9600Present()) {
                        Serial.println("  ✅ MCP9600 found at 0x60 (Main Ch0, Sec Ch2)");
                    } else {
                        Serial.println("  ⚠️ MCP9600 NOT found at expected path");
                    }
                }
            } else {
                Serial.println("  ℹ️ No secondary multiplexer detected");
            }
        }
        resetI2CPath();
        delay(10);
    } else {
        Serial.println("  ❌ No PCA9548A multiplexer detected at 0x70");
        useMultiplexer = false;
        hasSecondaryMux = false;
    }
    
    // Now do the full scan for all devices
    scanI2CBus();

    // ========== INITIALIZE DEVICES ON MAIN BUS (NO MULTIPLEXER) ==========
    Serial.println("\n--- Initializing Main Bus Devices ---");
    
    // MCP23017 on main bus (address 0x20)
    Serial.print("  MCP23017 GPIO (0x20): ");
    if (mcp23017.begin_I2C(0x20)) {
        for (int i = 0; i < 16; i++) {
            mcp23017.pinMode(i, INPUT_PULLUP);
        }
        sensorData.mcp23017_present = true;
        Serial.println("✅ FOUND");
    } else {
        sensorData.mcp23017_present = false;
        Serial.println("❌ NOT FOUND");
    }
    
    // ADS1115 #1 (0x48) on main bus
    Serial.print("  ADS1115 (0x48) ADC: ");
    if (ads1115_1.begin(0x48)) {
        ads1115_1.setGain(GAIN_ONE);
        ads1115_1.setDataRate(RATE_ADS1115_128SPS);
        sensorData.ads1115_present = true;
        Serial.println("✅ FOUND");
    } else {
        Serial.println("❌ NOT FOUND");
    }
    
    // ADS1115 #2 (0x49) on main bus
    Serial.print("  ADS1115 (0x49) ADC: ");
    if (ads1115_2.begin(0x49)) {
        ads1115_2.setGain(GAIN_ONE);
        ads1115_2.setDataRate(RATE_ADS1115_128SPS);
        Serial.println("✅ FOUND");
    } else {
        Serial.println("❌ NOT FOUND");
    }
    
    // DS3231 RTC on main bus
    Serial.print("  DS3231 RTC (0x68): ");
    if (rtc.begin()) {
        Serial.println("✅ FOUND");
        sensorData.rtc.valid = true;
    } else {
        Serial.println("❌ NOT FOUND");
    }
    
    // ========== INITIALIZE MCP9600 THERMOCOUPLES ON MULTIPLEXER ==========
    if (useMultiplexer) {
        Serial.println("\n--- Initializing MCP9600 Thermocouples ---");
        int foundCount = 0;
        for (int i = 0; i < HARDCODED_MCP9600_COUNT; i++) {
            uint8_t mainChannel = hardcodedMCP9600s[i].mainChannel;
            int8_t secondaryChannel = hardcodedMCP9600s[i].secondaryChannel;
            uint8_t address = hardcodedMCP9600s[i].address;

            Serial.printf("  [%d] %s: Main Ch%d", i + 1, hardcodedMCP9600s[i].name.c_str(), mainChannel);
            if (secondaryChannel >= 0) {
                Serial.printf(", Sec Ch%d", secondaryChannel);
            }
            Serial.printf(", Addr 0x%02X: ", address);

            // --- PATH SELECTION ---
            if (!selectMCP9600Path(hardcodedMCP9600s[i])) {
                Serial.println("❌ PATH FAILED");
                hardcodedMCP9600s[i].valid = false;
                sensorData.mcp9600_present[i] = false;
                continue;
            }

            delay(5);  // Small delay for mux to settle

            // Check if MCP9600 is present and read temperature
            if (isMCP9600Present()) {
                float temp = readMCP9600Temperature();
                Serial.printf(" Temp = %.2f°C", temp);
                if (!isnan(temp) && temp > -50 && temp < 500) {
                    hardcodedMCP9600s[i].valid = true;
                    hardcodedMCP9600s[i].instanceIndex = i;
                    sensorData.mcp9600_present[i] = true;
                    sensorData.thermocouples[i].valid = false;   // will be updated later
                    Serial.println(" ✅ OK");
                    foundCount++;
                } else {
                    hardcodedMCP9600s[i].valid = false;
                    sensorData.mcp9600_present[i] = false;
                    Serial.println(" ❌ Invalid temp reading");
                }
            } else {
                hardcodedMCP9600s[i].valid = false;
                sensorData.mcp9600_present[i] = false;
                Serial.println(" ❌ NOT FOUND");
            }
            
            // Disable mux after each test (important!)
            resetI2CPath();
            delay(5);
        }
        Serial.printf("\n  ✅ Found %d of %d MCP9600 devices\n", foundCount, HARDCODED_MCP9600_COUNT);
        resetI2CPath();
    } else {
        Serial.println("\n⚠️ No multiplexer detected! MCP9600 thermocouples will not be available.");
    }

    // Load I2C configuration for other devices (MCP23017, ADS1115, DS3231)
    loadI2CConfig();

    Serial.println("✅ I2C Sensors initialization complete");
}

void scanI2CBus() {
    Serial.println("\n🔍 Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    int timeout = 10; // ms per address

    // Check for main PCA9548A multiplexer at 0x70
    if (i2cWriteWithTimeout(PCA9548A_ADDR_MAIN, nullptr, 0, 10)) {
        Serial.printf("  ✅ Main PCA9548A Multiplexer found at 0x%02X\n", PCA9548A_ADDR_MAIN);
        useMultiplexer = true;
        sensorData.pca9548a_present = true;
        nDevices++;
        
        // Check for secondary multiplexer on main channel 0
        if (selectMainMuxChannel(0)) {
            if (i2cWriteWithTimeout(PCA9548A_ADDR_SECONDARY, nullptr, 0, 10)) {
                Serial.printf("  ✅ Secondary PCA9548A Multiplexer found at 0x%02X (on Main Ch0)\n", 
                             PCA9548A_ADDR_SECONDARY);
                hasSecondaryMux = true;
                nDevices++;
                
                // Also check for MCP9600 on secondary channel 2
                if (selectSecondaryMuxChannel(2)) {
                    if (isMCP9600Present()) {
                        Serial.println("      ✅ MCP9600 found on Sec Ch2 at 0x60");
                    }
                }
            }
            resetI2CPath();
        }
    } else {
        Serial.println("  ℹ️ No PCA9548A multiplexer detected");
        useMultiplexer = false;
        hasSecondaryMux = false;
    }
    
    // Scan main bus for other devices
    Serial.println("\n  Scanning main I2C bus for other devices:");
    for (address = 1; address < 127; address++) {
        if (address == PCA9548A_ADDR_MAIN || address == PCA9548A_ADDR_SECONDARY) continue;
        
        unsigned long start = millis();
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        unsigned long elapsed = millis() - start;
        if (elapsed > timeout) {
            Serial.printf("    Timeout at 0x%02X\n", address);
        }
        
        if (error == 0) {
            Serial.print("    Device found at 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            
            if (address == 0x68) Serial.print(" (DS3231 RTC)");
            else if (address == 0x20) Serial.print(" (MCP23017 GPIO)");
            else if (address == 0x48) Serial.print(" (ADS1115 ADC #1)");
            else if (address == 0x49) Serial.print(" (ADS1115 ADC #2)");
            else if (address == 0x60) Serial.print(" (MCP9600 - on mux path)");
            
            Serial.println();
            nDevices++;
        }
        delay(5);
    }

    if (nDevices == 0 && !useMultiplexer) {
        Serial.println("  No I2C devices found");
    } else {
        Serial.printf("\n  Total devices detected: %d\n", nDevices);
    }
}

// ================ UPDATE FUNCTIONS ================

void updateThermocouples() {
    if (!useMultiplexer) {
        static bool warned = false;
        if (!warned) {
            Serial.println("⚠️ No multiplexer, thermocouples skipped.");
            warned = true;
        }
        return;
    }
    
    unsigned long now = millis();
    
    for (int i = 0; i < HARDCODED_MCP9600_COUNT; i++) {
        HardcodedMCP9600& tc = hardcodedMCP9600s[i];
        if (!tc.valid) {
            sensorData.thermocouples[i].valid = false;
            continue;
        }

        // Select the correct path
        if (!selectMCP9600Path(tc)) {
            sensorData.thermocouples[i].valid = false;
            resetI2CPath();
            continue;
        }

        delay(5);  // Allow mux to settle

        // Read temperature
        float temp = readMCP9600Temperature();
        
        // Read ambient temperature (optional)
        float ambient = NAN;
        if (!isnan(temp)) {
            Wire.beginTransmission(MCP9600_ADDR);
            Wire.write(0x01);
            if (Wire.endTransmission(false) == 0) {
                if (Wire.requestFrom(MCP9600_ADDR, (uint8_t)2) == 2) {
                    int16_t rawAmbient = ((int16_t)Wire.read() << 8) | Wire.read();
                    ambient = rawAmbient * 0.0625f;
                }
            }
        }

        if (!isnan(temp) && temp > -50 && temp < 500) {
            sensorData.thermocouples[i].temperature = temp;
            sensorData.thermocouples[i].ambientTemp = ambient;
            sensorData.thermocouples[i].fault = 0;
            sensorData.thermocouples[i].valid = true;
            sensorData.thermocouples[i].lastUpdate = now;
        } else {
            sensorData.thermocouples[i].valid = false;
            sensorData.thermocouples[i].fault = 1;
        }
        
        // IMPORTANT: Disable mux after each read (like your standalone code)
        resetI2CPath();
    }
}

void updateRTC() {
    if (!rtc.begin()) return;
    
    DateTime now = rtc.now();
    
    sensorData.rtc.year = now.year();
    sensorData.rtc.month = now.month();
    sensorData.rtc.day = now.day();
    sensorData.rtc.hour = now.hour();
    sensorData.rtc.minute = now.minute();
    sensorData.rtc.second = now.second();
    sensorData.rtc.temperature = rtc.getTemperature();
    sensorData.rtc.valid = true;
    sensorData.rtc.lastUpdate = millis();
}

void updateMCP23017() {
    if (!sensorData.mcp23017_present) return;
    
    uint16_t gpioA = mcp23017.readGPIOAB();
    sensorData.gpio.gpio_state = gpioA;
    
    for (int i = 0; i < 16; i++) {
        sensorData.gpio.input_pins[i] = (gpioA >> i) & 1;
    }
    
    sensorData.gpio.valid = true;
    sensorData.gpio.lastUpdate = millis();
}

void updateADS1115() {
    unsigned long now = millis();
    bool any_valid = false;

    // Clear previous readings
    sensorData.ads1115_data.clear();

    // Static map to hold persistent ADS1115 objects (one per address)
    static std::map<uint8_t, Adafruit_ADS1115*> adsObjects;

    // Iterate over all ADS1115 devices from the dynamic configuration
    for (const auto& dev : i2cConfig.devices) {
        if (dev.type != "ADS1115") continue;
        uint8_t addr = dev.address;

        // Get or create the ADS1115 object for this address
        Adafruit_ADS1115* ads = nullptr;
        auto it = adsObjects.find(addr);
        if (it == adsObjects.end()) {
            // Create new object
            ads = new Adafruit_ADS1115();
            if (!ads->begin(addr)) {
                // Only print error once per address
                static std::set<uint8_t> printedErrors;
                if (printedErrors.find(addr) == printedErrors.end()) {
                    Serial.printf("ADS1115 @0x%02X not found or init failed\n", addr);
                    printedErrors.insert(addr);
                }
                delete ads;
                continue;
            }
            ads->setGain(GAIN_ONE);
            ads->setDataRate(RATE_ADS1115_128SPS);
            adsObjects[addr] = ads;
            Serial.printf("ADS1115 @0x%02X initialized\n", addr);
        } else {
            ads = it->second;
        }

        // Read all 4 channels
        ADS1115Data data;
        data.valid = true;
        data.lastUpdate = now;
        for (int ch = 0; ch < 4; ch++) {
            int16_t raw = ads->readADC_SingleEnded(ch);
            data.raw_values[ch] = raw;
            data.voltages[ch] = raw * 0.125 / 1000.0;
        }
        sensorData.ads1115_data[addr] = data;
        any_valid = true;

        // Verbose print removed – no more ADC lines every loop
    }

    sensorData.ads1115_present = any_valid;
}

// ================ POPULATE i2cValues ================
void populateI2CValues() {
    i2cValues.clear();
    unsigned long now = millis();

    // ========== HARDCODED: Add MCP9600 thermocouple values ==========
    for (int i = 0; i < HARDCODED_MCP9600_COUNT; i++) {
        if (hardcodedMCP9600s[i].valid && 
            sensorData.thermocouples[hardcodedMCP9600s[i].instanceIndex].valid &&
            (now - sensorData.thermocouples[hardcodedMCP9600s[i].instanceIndex].lastUpdate) <= 5000) {
            i2cValues[hardcodedMCP9600s[i].name] = 
                sensorData.thermocouples[hardcodedMCP9600s[i].instanceIndex].temperature;
        }
    }

    // ========== DYNAMIC CONFIG: For other devices ==========
    for (const auto& dev : i2cConfig.devices) {
        if (dev.type == "MCP23017") {
            if (!sensorData.gpio.valid) continue;
            for (const auto& sig : dev.signals) {
                if (!sig.enabled) continue;
                uint16_t raw = (sig.bitMask == 0) ? sensorData.gpio.gpio_state : 
                               (sensorData.gpio.gpio_state & sig.bitMask);
                if (sig.bitMask != 0 && (sig.bitMask & (sig.bitMask - 1)) == 0) {
                    int bitPos = 0;
                    uint16_t mask = sig.bitMask;
                    while ((mask & 1) == 0) {
                        mask >>= 1;
                        bitPos++;
                    }
                    raw >>= bitPos;
                    if (sig.invert) raw = raw ? 0 : 1;
                }
                double phys = raw * sig.factor + sig.offset;
                i2cValues[sig.name] = phys;
            }
        }
        else if (dev.type == "ADS1115") {
            uint8_t addr = dev.address;
            auto it = sensorData.ads1115_data.find(addr);
            if (it == sensorData.ads1115_data.end() || !it->second.valid) continue;
            const ADS1115Data& adcData = it->second;
            for (const auto& sig : dev.signals) {
                if (!sig.enabled) continue;
                if (sig.channel >= 0 && sig.channel < 4) {
                    double phys = adcData.voltages[sig.channel] * sig.factor + sig.offset;
                    i2cValues[sig.name] = phys;
                }
            }
        }
        else if (dev.type == "DS3231") {
            if (!sensorData.rtc.valid) continue;
            for (const auto& sig : dev.signals) {
                if (!sig.enabled) continue;
                if (sig.name == "RTC_Temperature") {
                    double phys = sensorData.rtc.temperature * sig.factor + sig.offset;
                    i2cValues[sig.name] = phys;
                }
            }
        }
    }
}

// ================ MAIN UPDATE FUNCTION ================
void updateI2CSensors() {
    static unsigned long lastI2CUpdate = 0;
    unsigned long now = millis();
    
    if (now - lastI2CUpdate < I2C_UPDATE_INTERVAL_MS) return;
    lastI2CUpdate = now;
    
    updateRTC();
    updateMCP23017();
    updateADS1115();
    updateThermocouples();
    
    sensorData.lastScanTime = now;
    populateI2CValues();
    resetI2CPath();
}

// ================ CSV FORMATTING ================
void formatI2CData(char* buffer, size_t bufferSize, unsigned long currentTime) {
    int pos = 0;
    
    // RTC Data
    if (sensorData.rtc.valid && isValid(sensorData.rtc.lastUpdate, 5000, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%04d-%02d-%02d,%02d:%02d:%02d,%.1f",
                        sensorData.rtc.year, sensorData.rtc.month, sensorData.rtc.day,
                        sensorData.rtc.hour, sensorData.rtc.minute, sensorData.rtc.second,
                        sensorData.rtc.temperature);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // Thermocouples
    for (int i = 0; i < HARDCODED_MCP9600_COUNT; i++) {
        int idx = hardcodedMCP9600s[i].instanceIndex;
        if (sensorData.thermocouples[idx].valid && 
            isValid(sensorData.thermocouples[idx].lastUpdate, 5000, currentTime)) {
            pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%d",
                            sensorData.thermocouples[idx].temperature,
                            sensorData.thermocouples[idx].ambientTemp,
                            sensorData.thermocouples[idx].fault);
        } else {
            pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
        }
    }
    
    for (int i = HARDCODED_MCP9600_COUNT; i < THERMOCOUPLE_COUNT; i++) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // GPIO
    if (sensorData.gpio.valid && isValid(sensorData.gpio.lastUpdate, 5000, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",0x%04X", sensorData.gpio.gpio_state);
        for (int i = 0; i < 16; i++) {
            pos += snprintf(buffer + pos, bufferSize - pos, ",%d", sensorData.gpio.input_pins[i]);
        }
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
        for (int i = 0; i < 16; i++) pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // ADS1115 Data
    if (!sensorData.ads1115_data.empty()) {
        // Take the first ADS1115 (lowest address) for static logging
        auto it = sensorData.ads1115_data.begin();
        const ADS1115Data& adc = it->second;
        for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
            pos += snprintf(buffer + pos, bufferSize - pos, ",%.3f,%d",
                            adc.voltages[i], adc.raw_values[i]);
        }
    } else {
        for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
            pos += snprintf(buffer + pos, bufferSize - pos, ",,");
        }
    }
}