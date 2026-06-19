#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BluetoothSerial.h>

// ============================================================================
// SECTION 1: CONFIGURATION - PINS, CALIBRATION, TIMING
// ============================================================================

#define FIRMWARE_VERSION        "1.0.0"
#define DEVICE_NAME              "SmartBMS-ESP32"
#define SERIAL_BAUD_RATE          115200

// --- Pin assignments (ESP32 DevKit-1 / Wokwi "esp32" board) ---
#define PIN_VOLTAGE_SENSE         34   // Voltage divider output -> battery voltage
#define PIN_CURRENT_SENSE         35   // ACS712 OUT pin -> current sensing
#define PIN_TEMP_SENSE            32   // LM35 OUT pin -> temperature sensing
#define PIN_FAN_CONTROL           25   // Drives fan via NPN transistor / MOSFET gate
#define PIN_STATUS_LED_OK         26   // Green LED - system healthy
#define PIN_STATUS_LED_FAULT      27   // Red LED - fault present
#define PIN_BATTERY_DISCONNECT    33   // Drives relay/MOSFET to disconnect battery on fault
#define PIN_BUZZER                14   // Reserved for future audible alarm

// --- ADC configuration ---
#define ADC_RESOLUTION_BITS       12
#define ADC_MAX_VALUE             4095.0f
#define ADC_REF_VOLTAGE           3.3f

// --- Voltage divider calibration (CALIBRATE) ---
// Divider: Vbat -> R1 -> ADC_PIN -> R2 -> GND
// NOTE: For a 24V max system, ensure (R1+R2)/R2 keeps ADC pin <= 3.3V at worst case.
// Example for 24V max: R1=120k, R2=15k -> ratio 9.0 -> 24V gives ~2.67V at the pin.
#define VOLTAGE_DIVIDER_R1         120000.0f   // Ohms (CALIBRATE: measure actual resistor)
#define VOLTAGE_DIVIDER_R2          15000.0f   // Ohms (CALIBRATE: measure actual resistor)
#define VOLTAGE_DIVIDER_RATIO      ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)
#define VOLTAGE_CALIBRATION_OFFSET  0.0f        // Volts (CALIBRATE)

// --- ACS712 calibration (CALIBRATE) ---
// Variants: ACS712-05B (185mV/A), ACS712-20A (100mV/A), ACS712-30A (66mV/A)
#define ACS712_SENSITIVITY_MV_PER_A   100.0f    // mV per Amp (CALIBRATE: depends on module variant)
#define ACS712_ZERO_CURRENT_VOLTAGE    1.65f    // Volts at 0A, ideally ADC_REF_VOLTAGE/2 (CALIBRATE)
#define CURRENT_CALIBRATION_OFFSET     0.0f     // Amps (CALIBRATE)
#define CURRENT_FILTER_SAMPLES         20

// --- LM35 calibration (CALIBRATE) ---
#define LM35_MV_PER_DEGREE_C       10.0f
#define TEMPERATURE_CALIBRATION_OFFSET 0.0f     // Degrees C (CALIBRATE)

// --- Sampling / timing intervals (ms) ---
#define SENSOR_SAMPLE_INTERVAL_MS      500
#define PROTECTION_CHECK_INTERVAL_MS   500
#define STATUS_REPORT_INTERVAL_MS      2000
#define FAN_CONTROL_INTERVAL_MS        1000

// --- Fan control thresholds ---
#define FAN_ON_TEMP_C               40.0f
#define FAN_OFF_TEMP_C               35.0f

// --- Wi-Fi configuration (EDIT BEFORE DEPLOYMENT) ---
#define WIFI_SSID                  "YOUR_WIFI_SSID"
#define WIFI_PASSWORD                "YOUR_WIFI_PASSWORD"
#define WIFI_CONNECT_TIMEOUT_MS      15000
#define WIFI_RECONNECT_INTERVAL_MS   30000
#define HTTP_SERVER_PORT              80

// --- Bluetooth configuration ---
#define BLUETOOTH_DEVICE_NAME        "SmartBMS_ESP32"

// --- Safety / fault latching ---
#define ENABLE_BATTERY_DISCONNECT_ON_FAULT  true
#define FAULT_LATCH_REQUIRES_MANUAL_CLEAR   true

// ============================================================================
// SECTION 2: DATA STRUCTURES & ENUMERATIONS
// ============================================================================

// --- Battery type selector ---
enum class BatteryType : uint8_t {
    MOTORCYCLE = 0,
    TUKTUK     = 1,
    CAR        = 2,
    TRUCK      = 3
};

// --- Per-battery-type protection/SOC thresholds ---
struct BatteryProfile {
    const char* name;
    float nominalVoltage;
    float fullChargeVoltage;
    float emptyVoltage;
    float overVoltageCutoff;
    float underVoltageCutoff;
    float overCurrentCutoff;
    float overTemperatureCutoff;
    float capacityAh;
};

// --- Snapshot of live sensor data ---
struct SensorReadings {
    float voltage;
    float current;
    float temperature;
    unsigned long timestampMs;
};

// --- Fault bitmask flags ---
enum FaultFlag : uint16_t {
    FAULT_NONE             = 0,
    FAULT_OVER_VOLTAGE     = 1 << 0,
    FAULT_UNDER_VOLTAGE    = 1 << 1,
    FAULT_OVER_CURRENT     = 1 << 2,
    FAULT_OVER_TEMPERATURE = 1 << 3,
    FAULT_SENSOR_ERROR     = 1 << 4
};

// --- Battery health classification ---
enum class BatteryHealth : uint8_t {
    GOOD = 0,
    DEGRADED = 1,
    POOR = 2
};

// ============================================================================
// SECTION 3: BATTERY PROFILE MANAGER
// Realistic lead-acid thresholds for each supported battery type.
// ============================================================================
class BatteryProfileManager {
public:
    BatteryProfileManager() : activeType(BatteryType::CAR) {}   // Default to Car profile

    // Switch the active battery profile to the given type.
    void setBatteryType(BatteryType type) { activeType = type; }

    // Return the enum of the currently active battery type.
    BatteryType getBatteryType() const { return activeType; }

    // Return a reference to the full threshold struct for the active battery type.
    const BatteryProfile& getActiveProfile() const { return profiles[static_cast<uint8_t>(activeType)]; }

    // Return the human-readable name of the active battery type.
    const char* getBatteryTypeName() const { return profiles[static_cast<uint8_t>(activeType)].name; }

    // Parse a command string (e.g. "motorcycle") into a BatteryType enum; returns false if unknown.
    static bool parseBatteryTypeFromString(const String& str, BatteryType& outType) {
        String s = str;
        s.toLowerCase();
        s.trim();
        if (s == "motorcycle" || s == "bike") { outType = BatteryType::MOTORCYCLE; return true; }
        if (s == "tuktuk" || s == "tuk-tuk" || s == "tuk_tuk") { outType = BatteryType::TUKTUK; return true; }
        if (s == "car") { outType = BatteryType::CAR; return true; }
        if (s == "truck") { outType = BatteryType::TRUCK; return true; }
        return false;
    }

private:
    BatteryType activeType;

    // Lookup table: name, nominal, full, empty, OV, UV, OC(A), OT(C), capacity(Ah)
    static constexpr BatteryProfile profiles[4] = {
        { "Motorcycle", 12.0f, 13.0f, 11.5f, 14.8f, 11.0f,  30.0f, 50.0f,  9.0f  },
        { "Tuk-Tuk",     12.0f, 13.0f, 11.5f, 14.8f, 11.0f,  60.0f, 50.0f,  35.0f },
        { "Car",          12.0f, 12.9f, 11.5f, 15.0f, 11.2f, 120.0f, 55.0f,  60.0f },
        { "Truck",         24.0f, 25.8f, 23.0f, 29.5f, 22.0f, 200.0f, 55.0f, 120.0f }
    };
};

constexpr BatteryProfile BatteryProfileManager::profiles[4]; // Out-of-class definition for static constexpr array

// ============================================================================
// SECTION 4: SENSOR MANAGER
// Reads and converts raw ADC data from the voltage divider, ACS712, and LM35.
// ============================================================================
class SensorManager {
public:
    SensorManager() { lastReading = { 0.0f, 0.0f, 0.0f, 0 }; }

    // Configure ADC resolution and pin modes for all analog inputs.
    void begin() {
        analogReadResolution(ADC_RESOLUTION_BITS);
        pinMode(PIN_VOLTAGE_SENSE, INPUT);
        pinMode(PIN_CURRENT_SENSE, INPUT);
        pinMode(PIN_TEMP_SENSE, INPUT);
    }

    // Read the voltage divider pin and scale it back up to true battery voltage.
    float readBatteryVoltage() {
        int raw = oversampleAnalogRead(PIN_VOLTAGE_SENSE, 10);
        float pinVoltage = rawAdcToVoltage(raw);
        return (pinVoltage * VOLTAGE_DIVIDER_RATIO) + VOLTAGE_CALIBRATION_OFFSET;
    }

    // Read the ACS712 output pin and convert it to a current value in Amps.
    float readBatteryCurrent() {
        int raw = oversampleAnalogRead(PIN_CURRENT_SENSE, CURRENT_FILTER_SAMPLES);
        float pinVoltage = rawAdcToVoltage(raw);
        float deltaV_mV = (pinVoltage - ACS712_ZERO_CURRENT_VOLTAGE) * 1000.0f;
        return (deltaV_mV / ACS712_SENSITIVITY_MV_PER_A) + CURRENT_CALIBRATION_OFFSET;
    }

    // Read the LM35 output pin and convert it to a temperature value in Celsius.
    float readBatteryTemperature() {
        int raw = oversampleAnalogRead(PIN_TEMP_SENSE, 10);
        float pinVoltage = rawAdcToVoltage(raw);
        return (pinVoltage * 1000.0f / LM35_MV_PER_DEGREE_C) + TEMPERATURE_CALIBRATION_OFFSET;
    }

    // Read all three sensors together and return a timestamped snapshot.
    SensorReadings readAll() {
        SensorReadings r;
        r.voltage = readBatteryVoltage();
        r.current = readBatteryCurrent();
        r.temperature = readBatteryTemperature();
        r.timestampMs = millis();
        lastReading = r;
        return r;
    }

private:
    // Convert a raw ADC count (0-4095) into the voltage seen at the ESP32 ADC pin.
    float rawAdcToVoltage(int rawAdc) {
        return (static_cast<float>(rawAdc) / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    }

    // Take multiple ADC samples and return their average to reduce sensor noise.
    int oversampleAnalogRead(uint8_t pin, uint8_t samples) {
        long sum = 0;
        for (uint8_t i = 0; i < samples; i++) {
            sum += analogRead(pin);
            delayMicroseconds(200);
        }
        return static_cast<int>(sum / samples);
    }

    SensorReadings lastReading;
};

// ============================================================================
// SECTION 5: PROTECTION MANAGER
// Fault detection, SOC estimation, battery health, and protective outputs.
// ============================================================================
class ProtectionManager {
public:
    ProtectionManager()
        : activeFaultBits(FAULT_NONE), lastSocPercent(0), lastHealth(BatteryHealth::GOOD), disconnected(false) {}

    // Configure the battery-disconnect and status LED output pins.
    void begin() {
        pinMode(PIN_BATTERY_DISCONNECT, OUTPUT);
        pinMode(PIN_STATUS_LED_OK, OUTPUT);
        pinMode(PIN_STATUS_LED_FAULT, OUTPUT);
        digitalWrite(PIN_BATTERY_DISCONNECT, LOW); // LOW = battery connected (safe default)
        digitalWrite(PIN_STATUS_LED_OK, HIGH);
        digitalWrite(PIN_STATUS_LED_FAULT, LOW);
    }

    // Run all protection checks against the latest sensor readings and active battery profile.
    void evaluate(const SensorReadings& readings, const BatteryProfile& profile) {
        setFault(FAULT_OVER_VOLTAGE, readings.voltage > profile.overVoltageCutoff);
        setFault(FAULT_UNDER_VOLTAGE, readings.voltage < profile.underVoltageCutoff);
        setFault(FAULT_OVER_CURRENT, fabs(readings.current) > profile.overCurrentCutoff);
        setFault(FAULT_OVER_TEMPERATURE, readings.temperature > profile.overTemperatureCutoff);

        bool sensorError = (readings.voltage < 0.0f || readings.voltage > 60.0f) ||
                            (readings.temperature < -40.0f || readings.temperature > 150.0f);
        setFault(FAULT_SENSOR_ERROR, sensorError);

        lastSocPercent = estimateStateOfCharge(readings.voltage, profile);
        lastHealth = assessHealth(readings.voltage, readings.current, profile);

        if (ENABLE_BATTERY_DISCONNECT_ON_FAULT) {
            disconnected = (activeFaultBits != FAULT_NONE);
        }
        updateOutputs();
    }

    uint16_t getActiveFaults() const { return activeFaultBits; }                 // Get current fault bitmask
    bool isBatteryDisconnected() const { return disconnected; }                   // Get disconnect relay state
    uint8_t getStateOfCharge() const { return lastSocPercent; }                    // Get last computed SOC
    BatteryHealth getBatteryHealth() const { return lastHealth; }                   // Get last health classification

    // Build a comma-separated human-readable string of all active faults.
    String getActiveFaultsAsString() const {
        if (activeFaultBits == FAULT_NONE) return "None";
        String result = "";
        if (activeFaultBits & FAULT_OVER_VOLTAGE)     result += "OVER_VOLTAGE, ";
        if (activeFaultBits & FAULT_UNDER_VOLTAGE)    result += "UNDER_VOLTAGE, ";
        if (activeFaultBits & FAULT_OVER_CURRENT)     result += "OVER_CURRENT, ";
        if (activeFaultBits & FAULT_OVER_TEMPERATURE) result += "OVER_TEMPERATURE, ";
        if (activeFaultBits & FAULT_SENSOR_ERROR)     result += "SENSOR_ERROR, ";
        result.remove(result.length() - 2);
        return result;
    }

    // Build a JSON array string of active fault names for the API.
    String getActiveFaultsAsJsonArray() const {
        if (activeFaultBits == FAULT_NONE) return "[]";
        String result = "[";
        bool first = true;
        auto append = [&](const char* name) {
            if (!first) result += ",";
            result += "\"";
            result += name;
            result += "\"";
            first = false;
        };
        if (activeFaultBits & FAULT_OVER_VOLTAGE)     append("OVER_VOLTAGE");
        if (activeFaultBits & FAULT_UNDER_VOLTAGE)    append("UNDER_VOLTAGE");
        if (activeFaultBits & FAULT_OVER_CURRENT)     append("OVER_CURRENT");
        if (activeFaultBits & FAULT_OVER_TEMPERATURE) append("OVER_TEMPERATURE");
        if (activeFaultBits & FAULT_SENSOR_ERROR)     append("SENSOR_ERROR");
        result += "]";
        return result;
    }

    // Manually clear all latched faults and re-enable the battery connection.
    void clearFaults() {
        activeFaultBits = FAULT_NONE;
        disconnected = false;
        updateOutputs();
    }

private:
    // Set or clear an individual fault bit in the active fault bitmask.
    void setFault(FaultFlag fault, bool active) {
        if (active) {
            activeFaultBits |= fault;
        } else if (!FAULT_LATCH_REQUIRES_MANUAL_CLEAR) {
            activeFaultBits &= ~fault;
        }
    }

    // Estimate State of Charge using linear interpolation between empty and full resting voltage.
    uint8_t estimateStateOfCharge(float voltage, const BatteryProfile& profile) {
        if (voltage <= profile.emptyVoltage) return 0;
        if (voltage >= profile.fullChargeVoltage) return 100;
        float range = profile.fullChargeVoltage - profile.emptyVoltage;
        float position = voltage - profile.emptyVoltage;
        float percent = (position / range) * 100.0f;
        return static_cast<uint8_t>(constrain(percent, 0.0f, 100.0f));
    }

    // Assess battery health based on voltage sag relative to current draw (simple heuristic).
    BatteryHealth assessHealth(float voltage, float current, const BatteryProfile& profile) {
        float sagMargin = voltage - profile.underVoltageCutoff;
        if (current > (profile.overCurrentCutoff * 0.5f) && sagMargin < 0.3f) return BatteryHealth::POOR;
        if (current > (profile.overCurrentCutoff * 0.3f) && sagMargin < 0.6f) return BatteryHealth::DEGRADED;
        return BatteryHealth::GOOD;
    }

    // Drive the disconnect relay and status LEDs to reflect current fault state.
    void updateOutputs() {
        digitalWrite(PIN_BATTERY_DISCONNECT, disconnected ? HIGH : LOW);
        digitalWrite(PIN_STATUS_LED_FAULT, (activeFaultBits != FAULT_NONE) ? HIGH : LOW);
        digitalWrite(PIN_STATUS_LED_OK, (activeFaultBits == FAULT_NONE) ? HIGH : LOW);
    }

    uint16_t activeFaultBits;
    uint8_t lastSocPercent;
    BatteryHealth lastHealth;
    bool disconnected;
};

// ============================================================================
// SECTION 6: FAN CONTROL
// Hysteresis-based cooling fan control driven by battery temperature.
// ============================================================================
class FanControl {
public:
    FanControl() : fanOn(false) {}

    // Configure the fan control output pin.
    void begin() {
        pinMode(PIN_FAN_CONTROL, OUTPUT);
        digitalWrite(PIN_FAN_CONTROL, LOW);
    }

    // Apply hysteresis: ON above FAN_ON_TEMP_C, OFF below FAN_OFF_TEMP_C, hold otherwise.
    void update(float temperatureC) {
        if (temperatureC >= FAN_ON_TEMP_C) fanOn = true;
        else if (temperatureC <= FAN_OFF_TEMP_C) fanOn = false;
        digitalWrite(PIN_FAN_CONTROL, fanOn ? HIGH : LOW);
    }

    bool isFanOn() const { return fanOn; }                                  // Get current fan state
    const char* getFanStatusString() const { return fanOn ? "ON" : "OFF"; }   // Get "ON"/"OFF" string

private:
    bool fanOn;
};

// ============================================================================
// SECTION 7: COMMUNICATION MANAGER
// Bluetooth Serial command interface + Wi-Fi HTTP/JSON status API.
// ============================================================================
class CommunicationManager {
public:
    // Constructor wires up references to all subsystems needed for status/commands.
    CommunicationManager(BatteryProfileManager& profileMgr, ProtectionManager& protectionMgr, FanControl& fanCtrl)
        : profileManager(profileMgr), protectionManager(protectionMgr), fanControl(fanCtrl),
          httpServer(HTTP_SERVER_PORT), lastWiFiAttemptMs(0), wifiWasConnected(false) {
        latestReadings = { 0.0f, 0.0f, 0.0f, 0 };
    }

    // Start Bluetooth Serial, connect to Wi-Fi, and start the HTTP server.
    void begin() {
        btSerial.begin(BLUETOOTH_DEVICE_NAME);
        Serial.println("[BT] Bluetooth Serial started as: " + String(BLUETOOTH_DEVICE_NAME));
        connectToWiFi();
        setupHttpRoutes();
        httpServer.begin();
        Serial.println("[HTTP] Web server started on port " + String(HTTP_SERVER_PORT));
    }

    // Poll Bluetooth and HTTP clients; should be called once per main loop() iteration.
    void update(const SensorReadings& readings) {
        latestReadings = readings;
        handleBluetoothCommands();
        maintainWiFiConnection();
        httpServer.handleClient();
    }

    // Build the /status JSON payload used by both the HTTP API and the BT get_status command.
    String buildStatusJson(const SensorReadings& readings) const {
        String json = "{";
        json += "\"battery_type\":\"" + String(profileManager.getBatteryTypeName()) + "\",";
        json += "\"voltage\":" + String(readings.voltage, 2) + ",";
        json += "\"current\":" + String(readings.current, 2) + ",";
        json += "\"temperature\":" + String(readings.temperature, 1) + ",";
        json += "\"soc\":" + String(protectionManager.getStateOfCharge()) + ",";
        json += "\"fan\":\"" + String(fanControl.getFanStatusString()) + "\",";
        json += "\"disconnected\":" + String(protectionManager.isBatteryDisconnected() ? "true" : "false") + ",";
        json += "\"faults\":" + protectionManager.getActiveFaultsAsJsonArray();
        json += "}";
        return json;
    }

private:
    // Attempt to connect to Wi-Fi in station mode, waiting up to WIFI_CONNECT_TIMEOUT_MS.
    void connectToWiFi() {
        Serial.println("[WiFi] Connecting to SSID: " + String(WIFI_SSID));
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        unsigned long startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT_MS) {
            delay(250);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n[WiFi] Connected. IP address: " + WiFi.localIP().toString());
            wifiWasConnected = true;
        } else {
            Serial.println("\n[WiFi] Connection failed. Will retry in background.");
            wifiWasConnected = false;
        }
        lastWiFiAttemptMs = millis();
    }

    // Check Wi-Fi status periodically and reconnect if the connection has dropped.
    void maintainWiFiConnection() {
        if (WiFi.status() != WL_CONNECTED) {
            if (wifiWasConnected) { Serial.println("[WiFi] Connection lost."); wifiWasConnected = false; }
            if (millis() - lastWiFiAttemptMs > WIFI_RECONNECT_INTERVAL_MS) {
                Serial.println("[WiFi] Attempting reconnect...");
                connectToWiFi();
            }
        } else if (!wifiWasConnected) {
            Serial.println("[WiFi] Reconnected. IP address: " + WiFi.localIP().toString());
            wifiWasConnected = true;
        }
    }

    // Register all HTTP server endpoint handlers.
    void setupHttpRoutes() {
        httpServer.on("/", HTTP_GET, [this]() { handleRootEndpoint(); });
        httpServer.on("/status", HTTP_GET, [this]() { handleStatusEndpoint(); });
        httpServer.onNotFound([this]() { handleNotFound(); });
    }

    // Simple landing page confirming the device is online and listing available endpoints.
    void handleRootEndpoint() {
        String html = "<html><body><h2>" + String(DEVICE_NAME) + "</h2>";
        html += "<p>Firmware v" + String(FIRMWARE_VERSION) + "</p>";
        html += "<p>Available endpoint: <a href=\"/status\">/status</a></p></body></html>";
        httpServer.send(200, "text/html", html);
    }

    // GET /status — returns the full battery status as a JSON object.
    void handleStatusEndpoint() {
        httpServer.send(200, "application/json", buildStatusJson(latestReadings));
    }

    // Fallback handler for any route that is not registered.
    void handleNotFound() {
        httpServer.send(404, "application/json", "{\"error\":\"not_found\"}");
    }

    // Check for and read any pending Bluetooth Serial command, then dispatch it.
    void handleBluetoothCommands() {
        if (btSerial.available()) {
            String command = btSerial.readStringUntil('\n');
            command.trim();
            if (command.length() > 0) processCommand(command);
        }
    }

    // Parse a single text command and execute the corresponding action.
    void processCommand(const String& command) {
        String cmd = command;
        cmd.toLowerCase();
        cmd.trim();

        if (cmd.startsWith("set_type")) {
            int spaceIndex = cmd.indexOf(' ');
            if (spaceIndex == -1) {
                sendBluetoothReply("ERROR: usage 'set_type <motorcycle|tuktuk|car|truck>'");
                return;
            }
            String typeArg = cmd.substring(spaceIndex + 1);
            BatteryType parsedType;
            if (BatteryProfileManager::parseBatteryTypeFromString(typeArg, parsedType)) {
                profileManager.setBatteryType(parsedType);
                sendBluetoothReply("OK: battery type set to " + String(profileManager.getBatteryTypeName()));
            } else {
                sendBluetoothReply("ERROR: unknown battery type '" + typeArg + "'");
            }
        } else if (cmd == "get_status") {
            sendBluetoothReply(buildStatusJson(latestReadings));
        } else if (cmd == "get_faults") {
            sendBluetoothReply("Faults: " + protectionManager.getActiveFaultsAsString());
        } else if (cmd == "clear_faults") {
            protectionManager.clearFaults();
            sendBluetoothReply("OK: faults cleared");
        } else {
            sendBluetoothReply("ERROR: unknown command '" + command + "'");
        }
    }

    // Send a text reply back to the connected Bluetooth client, followed by a newline.
    void sendBluetoothReply(const String& message) { btSerial.println(message); }

    BatteryProfileManager& profileManager;
    ProtectionManager& protectionManager;
    FanControl& fanControl;

    BluetoothSerial btSerial;
    WebServer httpServer;

    SensorReadings latestReadings;
    unsigned long lastWiFiAttemptMs;
    bool wifiWasConnected;
};

// ============================================================================
// SECTION 8: GLOBAL INSTANCES & APPLICATION ENTRY POINT
// ============================================================================
SensorManager sensorManager;
BatteryProfileManager profileManager;
ProtectionManager protectionManager;
FanControl fanControl;
CommunicationManager commManager(profileManager, protectionManager, fanControl);

unsigned long lastSensorReadMs = 0;
unsigned long lastProtectionCheckMs = 0;
unsigned long lastFanUpdateMs = 0;
unsigned long lastStatusReportMs = 0;

SensorReadings currentReadings;

void printStatusToSerial(); // Forward declaration

// --- setup() - runs once at boot ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);
    Serial.println("\n=================================================");
    Serial.println(String(DEVICE_NAME) + " | Firmware v" + String(FIRMWARE_VERSION));
    Serial.println("=================================================");

    sensorManager.begin();
    protectionManager.begin();
    fanControl.begin();
    commManager.begin();

    currentReadings = sensorManager.readAll();
    Serial.println("[SYSTEM] Initialization complete. Default battery type: " +
                    String(profileManager.getBatteryTypeName()));
    Serial.println("=================================================\n");
}

// --- loop() - runs continuously ---
void loop() {
    unsigned long now = millis();

    if (now - lastSensorReadMs >= SENSOR_SAMPLE_INTERVAL_MS) {
        currentReadings = sensorManager.readAll();
        lastSensorReadMs = now;
    }

    if (now - lastProtectionCheckMs >= PROTECTION_CHECK_INTERVAL_MS) {
        protectionManager.evaluate(currentReadings, profileManager.getActiveProfile());
        lastProtectionCheckMs = now;
    }

    if (now - lastFanUpdateMs >= FAN_CONTROL_INTERVAL_MS) {
        fanControl.update(currentReadings.temperature);
        lastFanUpdateMs = now;
    }

    commManager.update(currentReadings);

    if (now - lastStatusReportMs >= STATUS_REPORT_INTERVAL_MS) {
        printStatusToSerial();
        lastStatusReportMs = now;
    }
}

// --- Print a concise, human-readable status summary to the USB serial console ---
void printStatusToSerial() {
    Serial.println("---------------------------------------------------");
    Serial.println("Battery Type : " + String(profileManager.getBatteryTypeName()));
    Serial.printf("Voltage      : %.2f V\n", currentReadings.voltage);
    Serial.printf("Current      : %.2f A\n", currentReadings.current);
    Serial.printf("Temperature  : %.1f C\n", currentReadings.temperature);
    Serial.println("SOC          : " + String(protectionManager.getStateOfCharge()) + " %");
    Serial.println("Fan          : " + String(fanControl.getFanStatusString()));
    Serial.println("Disconnected : " + String(protectionManager.isBatteryDisconnected() ? "YES" : "NO"));
    Serial.println("Faults       : " + protectionManager.getActiveFaultsAsString());
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("HTTP API     : http://" + WiFi.localIP().toString() + "/status");
    }
    Serial.println("---------------------------------------------------");
}
