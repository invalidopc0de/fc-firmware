
#include <i2c_t3.h>
//#include <Wire.h>
#include <TinyGPS++.h>
#include <SPIFlash.h>
#include <SPI.h>
#include "cobs.h"
#include <time.h>
#include <EEPROM.h>

//#define DEBUG 0

#define GPS_REPORT_PERIOD 3000
#define SENSOR_LOG_PERIOD 1000

// FC Pins
const int ACCEL_INT1 = 0;
const int ACCEL_INT2 = 1;
const int ALT_INT1   = 3;
const int ALT_INT2   = 4;
const int GPS_RESET  = 5;
const int GPS_ANT    = 6;
const int GPS_OUT    = 7;
const int GPS_IN     = 8;
const int RADIO_OUT  = 9;
const int RADIO_IN   = 10;
const int FLASH_MOSI = 11;
const int FLASH_MISO = 12;
const int FLASH_SCK  = 13;
const int FLASH_WP   = 14;
const int FLASH_CS   = 15;
const int FLASH_HOLD = 16;
const int FIRE1      = 17;
const int FIRE2      = 18;
const int SENSOR_SCL = 19;
const int SENSOR_SDA = 20;
const int RADIO_EN   = 21;
const int RADIO_SET = 22;
const int RADIO_AUX  = 23;


// Size: 42 bytes
typedef struct __attribute__((__packed__)) TelemetryValues_t {
    uint32_t timestamp;
    double latitude;    // 8 
    double longitude;   // 8
    uint8_t day;        // 1
    uint8_t month;      // 1
    uint8_t hour;       // 1
    uint8_t minute;     // 1
    uint8_t second;     // 1
    uint8_t centisecond;// 1
    double altitude;    // 8
    double speed;       // 8
    uint32_t num_sats;  // 4
} TelemetryValues;

typedef struct __attribute__((__packed__)) LogHeader_t {
    uint32_t log_addr1;
    uint32_t log_addr2;
} LogHeader;

#define LOG_HDR_ADDR  0
#define LOG_START   (LOG_HDR_ADDR + sizeof(LogHeader))

uint32_t log_address = LOG_START;
bool confirm_reset = false;

// Devices

TinyGPSPlus gps;

SPIFlash flash(15);

const char* GPS_5HZ_NMEA = "$PSRF103,00,6,00,0*23";

TelemetryValues telemetry_values;
uint8_t cobs_buffer[sizeof(TelemetryValues) + 1];

LogHeader log_header;


// set this to the hardware serial port you wish to use
#define GPSSERIAL Serial3
#define RADIOSERIAL Serial2

void writeBytes(uint32_t address, uint8_t* data, int len)
{
    int current_index = 0;
    for (current_index = 0; current_index < len; current_index++)
    {
        flash.writeByte(address + current_index, data[current_index]);
        while (flash.busy());
    }
}

void readBytes(uint32_t address, uint8_t* data, int len)
{
    int current_index = 0;
    for (current_index = 0; current_index < len; current_index++)
    {
        data[current_index] = flash.readByte(address + current_index);
        while (flash.busy());
    }
}

void writeEEBytes(uint32_t address, uint8_t* data, int len)
{
    int current_index = 0;
    for (current_index = 0; current_index < len; current_index++)
    {
        EEPROM.write(address + current_index, data[current_index]);
        while (flash.busy());
    }
}

void readEEBytes(uint32_t address, uint8_t* data, int len)
{
    int current_index = 0;
    for (current_index = 0; current_index < len; current_index++)
    {
        data[current_index] = EEPROM.read(address + current_index);
        while (flash.busy());
    }
}


void updateTelemetry()
{
    // Update telemetry values
    telemetry_values.timestamp = millis();
    telemetry_values.latitude = gps.location.lat();
    telemetry_values.longitude = gps.location.lng();
    telemetry_values.day = gps.date.day();
    telemetry_values.month = gps.date.month();
    telemetry_values.hour = gps.time.hour();
    telemetry_values.minute = gps.time.minute();
    telemetry_values.second = gps.time.second();
    telemetry_values.centisecond = gps.time.centisecond();
    telemetry_values.altitude = gps.altitude.feet();
    telemetry_values.speed = gps.speed.mph();
    telemetry_values.num_sats = gps.satellites.value();
}

void updateTestTelemetry()
{
    // Update telemetry values
    telemetry_values.timestamp = millis();
    telemetry_values.latitude = 33.980289;
    telemetry_values.longitude = -118.451745;
    telemetry_values.day = 14;
    telemetry_values.month = 4;
    telemetry_values.hour = 11;
    telemetry_values.minute = 57;
    telemetry_values.second = 13;
    telemetry_values.centisecond = 10;
    telemetry_values.altitude = 30.47;
    telemetry_values.speed = 15.2;
    telemetry_values.num_sats = 5;
}

void sendTelemetry(HardwareSerial* serialDevice)
{   
    // Update COBS buffer
    cobs_encode(cobs_buffer, sizeof(cobs_buffer), (uint8_t*) &telemetry_values, sizeof(TelemetryValues));

    // Send buffer
    serialDevice->write(cobs_buffer, sizeof(cobs_buffer));
    serialDevice->write(0x00);
}

void logTelemetry()
{
    // Store the address of the last log entry in flash
    log_header.log_addr1 = log_address;
    log_header.log_addr2 = log_address;
    writeEEBytes(LOG_HDR_ADDR, (uint8_t*) &log_header, sizeof(LogHeader_t));

    //flash.writeBytes(LOG_CNTR_ADDR1, &log_address, sizeof(log_address));
    //flash.writeBytes(LOG_CNTR_ADDR2, &log_address, sizeof(log_address));

    // Write log entry
    writeBytes(log_address, (uint8_t*) &telemetry_values, sizeof(TelemetryValues_t));
    log_address += sizeof(TelemetryValues);

    readEEBytes(LOG_HDR_ADDR, (uint8_t*) &log_header, sizeof(LogHeader_t));
    //flash.readBytes(LOG_HDR_ADDR, &log_header, sizeof(LogHeader));

    Serial.print("Log address:");
    Serial.print(log_address, 10);
    Serial.print(" Value 1:");
    Serial.print(log_header.log_addr1, 10);
    Serial.print(" Value 2:");
    Serial.println(log_header.log_addr2, 10);
}

void printLog()
{
    TelemetryValues entry;

    uint32_t address = LOG_START;

    Serial.println("");

    for (address = LOG_START; address < log_address;)
    {
        // Read entry from flash
        readBytes(address, (uint8_t *) &entry, sizeof(TelemetryValues_t));

        // TODO: Print out entry
        Serial.print(address, 10); Serial.print(",");
        Serial.print(entry.timestamp, 10); Serial.print(",");
        Serial.print(entry.latitude, 6); Serial.print(",");
        Serial.print(entry.longitude, 6); Serial.print(",");
        Serial.print(entry.day, 10); Serial.print(",");
        Serial.print(entry.month, 10); Serial.print(",");
        Serial.print(entry.hour, 10); Serial.print(",");
        Serial.print(entry.minute, 10); Serial.print(",");
        Serial.print(entry.second, 10); Serial.print(",");
        Serial.print(entry.centisecond, 10); Serial.print(",");
        Serial.print(entry.altitude, 10); Serial.print(",");
        Serial.print(entry.speed, 10); Serial.print(",");
        Serial.print(entry.num_sats, 10); Serial.println();

        address += sizeof(TelemetryValues);
    }
}

void displayInfo(HardwareSerial* serialDevice)
{
    serialDevice->print(F("Location: "));
    if (gps.location.isValid())
    {
        serialDevice->print(gps.location.lat(), 6);
        serialDevice->print(F(","));
        RADIOSERIAL.print(gps.location.lng(), 6);
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }

    serialDevice->print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        serialDevice->print(gps.date.month());
        serialDevice->print(F("/"));
        serialDevice->print(gps.date.day());
        serialDevice->print(F("/"));
        serialDevice->print(gps.date.year());
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }

    serialDevice->print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10) serialDevice->print(F("0"));
        serialDevice->print(gps.time.hour());
        serialDevice->print(F(":"));
        if (gps.time.minute() < 10) serialDevice->print(F("0"));
        serialDevice->print(gps.time.minute());
        serialDevice->print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        serialDevice->print(gps.time.second());
        serialDevice->print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        serialDevice->print(gps.time.centisecond());
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }

    serialDevice->print(F(" Altitude: "));
    if (gps.altitude.isValid())
    {
        serialDevice->print(gps.altitude.feet(), 6);
        serialDevice->print(F("ft "));
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }

   
    serialDevice->print(F(" Speed: "));
    if (gps.speed.isValid())
    {
        serialDevice->print(gps.speed.mph(), 6);
        serialDevice->print(F("mph "));
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }

    serialDevice->print(F(" Sats: "));
    if (gps.satellites.isValid())
    {
        serialDevice->print(gps.satellites.value(), 6);
        serialDevice->print(F(" "));
    }
    else
    {
        serialDevice->print(F("INVALID"));
    }


    serialDevice->println();
}

// the setup() method runs once, when the sketch starts
void setup() {
    // initialize the digital pin as an output.
    //pinMode(ledPin, OUTPUT);
    Serial.begin(9600);

    // Radio setup
    pinMode(RADIO_EN, OUTPUT);
    pinMode(RADIO_SET, OUTPUT);
    pinMode(RADIO_AUX, INPUT);

    digitalWrite(RADIO_EN, HIGH);
    digitalWrite(RADIO_SET, HIGH);

    RADIOSERIAL.begin(9600, SERIAL_8N1);
    RADIOSERIAL.write(0xFF);

    digitalWrite(RADIO_EN, LOW);

    // Configure GPS  
    GPSSERIAL.begin(4800, SERIAL_8N1);
    pinMode(GPS_ANT, OUTPUT);
    digitalWrite(GPS_ANT, 1); // Enable external antenna

    delay(200);
    // Enable 5Hz output
    GPSSERIAL.println(GPS_5HZ_NMEA);

    // TODO is there a GPS power sequence that needs to be performed?
    pinMode(FLASH_HOLD, OUTPUT);
    pinMode(FLASH_WP, OUTPUT);

#ifdef DEBUG
    delay(5000);
#endif

    // HOLD high
    // WP high
    digitalWrite(FLASH_HOLD, HIGH);
    digitalWrite(FLASH_WP, HIGH);

    if (flash.initialize())
        Serial.println("Flash Init OK!");
    else
        Serial.println("Flash Init FAIL!");

    Serial.print("Device ID:");
    Serial.println(flash.readDeviceId());

    // Resume logging from last point
    readEEBytes(LOG_HDR_ADDR, (uint8_t*)&log_header, sizeof(LogHeader));

    Serial.print("Value 1:");
    Serial.println(log_header.log_addr1, 10);
    Serial.print("Value 2:");
    Serial.println(log_header.log_addr2, 10);

    if (log_header.log_addr1 == log_header.log_addr2)
    {
        log_address = log_header.log_addr1;
    }
    else if ((log_header.log_addr1 - sizeof(TelemetryValues)) == log_header.log_addr2) // Reset before storing value2
    {
        log_address = log_header.log_addr1;
    }
    else if (log_header.log_addr1 > log_header.log_addr2) // Reset during storing value1
    {
        log_address = log_header.log_addr2 + 1;
    }
    else if (log_header.log_addr1 < log_header.log_addr2) // Reset during storing value1
    {
        log_address = log_header.log_addr1;
    }

    //Wire.begin();
}

// the loop() methor runs over and over again,
// as long as the board has power
void loop() {
    char incomingByte = 0;
    bool new_GPS_data = false;
    unsigned long last_log_ms = millis();

    unsigned long chars;
    unsigned short sentences, failed;

    //RADIOSERIAL.println(radio_counter, DEC);
    //RADIOSERIAL.println();

    //setup_altimeter();

    for (unsigned long start = millis(); millis() - start < GPS_REPORT_PERIOD;)
    {
        if (millis() - last_log_ms > SENSOR_LOG_PERIOD) {
            // Read from sensors and update struct
#ifdef DEBUG
            updateTestTelemetry();
#else
            updateTelemetry();
#endif

            // Write struct to flash
            logTelemetry();
            last_log_ms = millis();
        }
        while (GPSSERIAL.available() > 0) {
            incomingByte = GPSSERIAL.read();
            gps.encode(incomingByte);
            //Serial.write(incomingByte);

            new_GPS_data = true;
            //RADIOSERIAL.write(incomingByte);
        }

        if (Serial.available() > 0)
        {
            incomingByte = Serial.read();

            switch (incomingByte)
            {
            case 'x':
                Serial.write("Reset data logging? y to reset");
                confirm_reset = true;
                break;
            case 'y':
                if (confirm_reset == true)
                {
                    log_address = 0;
                    confirm_reset = false;

                    Serial.print("Erasing Flash chip ... ");
                    flash.chipErase();
                    while (flash.busy());
                    Serial.println("DONE");
                }
                break;
            case 'r':
                // Read out flash data
                printLog();
                break;
            default:
                break;
            }
        }

    }

    Serial.println("3 sec");

#ifdef DEBUG
    updateTestTelemetry();
#else
    updateTelemetry();
#endif
    //sendTelemetry(&RADIOSERIAL);
    displayInfo(&RADIOSERIAL);
}
