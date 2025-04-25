#include <bluefruit.h>
#include <Adafruit_GFX.h>  // OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"  // MAX3010x library
#include "heartRate.h"

// Set up BLE service and characteristics
BLEService        hrms = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrm  = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);

//time
BLEService timeService = BLEService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic timeChar = BLECharacteristic("22222222-3333-4444-5555-666666666666", BLEWrite);

int beatAvg;
#define MAX_ALARMS 10
byte receivedAlarms[MAX_ALARMS][7];  // Stores up to 10 alarms, each 7 bytes long
int alarmCount = 0;
int rtc_day = 1, rtc_hour = 0, rtc_minute = 0, rtc_second = 0;
MAX30105 particleSensor;
#define RATE_SIZE 4  // Increase for more averaging, 4 is good
byte rates[RATE_SIZE];  // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  // Time at which the last beat occurred
float beatsPerMinute;

#define NUM_COMPARTMENTS 5
int ledPins[NUM_COMPARTMENTS] = {2, 3, 4, 5, 6};  // Replace with your actual LED GPIO pins
#define VIBRATION_MOTOR_PIN 7


unsigned long lastMillis = 0;  // Declare a variable to store the last millis time
unsigned long currentMillis;

void sendHRM(uint8_t bpm);
void timeChar_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);



void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Pin setup
  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);

  Serial.println("Starting Random BPM via BLE...");

  // Initialize Bluefruit module
  Bluefruit.begin();
  Bluefruit.setName("nRF52840_BPM");

  // Set up BLE callbacks for connect/disconnect
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Set up Heart Rate Measurement Service
  hrms.begin();
  hrm.setProperties(CHR_PROPS_NOTIFY);
  hrm.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrm.setFixedLen(2);  // 1 byte flags + 1 byte BPM
  hrm.begin();

  // Set up custom Time Service
  timeService.begin();
  timeChar.setProperties(BLEWrite);
  timeChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  timeChar.setFixedLen(7);  // Example: [index, day, hour, min, sec, box_no, vibrate_flag]
  timeChar.begin();

  // Add services to advertising
  Bluefruit.Advertising.addService(hrms);
  Bluefruit.Advertising.addService(timeService);
  
  // Start advertising
  Bluefruit.Advertising.start();
  particleSensor.begin(Wire, I2C_SPEED_FAST);  // Use default I2C port, 400kHz speed
  particleSensor.setup();  // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);

}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 1000) {  // 1 sec elapsed
    lastMillis = currentMillis;

    rtc_second++;
    if (rtc_second >= 60) {
      rtc_second = 0;
      rtc_minute++;
      if (rtc_minute >= 60) {
        rtc_minute = 0;
        rtc_hour++;
        if (rtc_hour >= 24) {
          rtc_hour = 0;
          rtc_day++;
          if (rtc_day > 7) rtc_day = 1;  // Assuming 1-7 for Mon-Sun
        }
      }
    }

    // Optional: Debug print current time
    Serial.printf("Time: Day %d - %02d:%02d:%02d\n", rtc_day, rtc_hour, rtc_minute, rtc_second);

    
  }
  // Check alarms
    for (int i = 0; i < alarmCount; i++) {
      byte* alarm = receivedAlarms[i];
      int a_hour = alarm[2];
      int a_min = alarm[3];
      int a_sec = alarm[4];
      int box_no = alarm[5];        // 1-5
      int vibrate_flag = alarm[6];  // 0/1

      // Skip used alarms (marked with invalid hour)
      if (a_hour == 255) continue;

      if (rtc_hour == a_hour &&
          rtc_minute == a_min &&
          rtc_second == a_sec) {

        Serial.print(" Alarm triggered for box ");
        Serial.println(box_no);

        // Turn on LED
        if (box_no >= 1 && box_no <= NUM_COMPARTMENTS) {
          digitalWrite(ledPins[box_no - 1], HIGH);
        }

        // Vibrate motor
        if (vibrate_flag == 1) {
          digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
          delay(2000);
          digitalWrite(VIBRATION_MOTOR_PIN, LOW);
        }

               // Keep LED on for 5s
        delay(5000);
        if (box_no >= 1 && box_no <= NUM_COMPARTMENTS) {
          digitalWrite(ledPins[box_no - 1], LOW);
        }

        // Mark alarm as triggered
        alarm[2] = 255;
      }
    }

  //beatAvg = random(60, 101);  // Random BPM between 60 and 100

  //sendHRM(beatAvg);
    long irValue = particleSensor.getIR();  // Reading the IR value to detect if there's a finger on the sensor
  if (irValue > 7000) {  // If a finger is detected
    // display.clearDisplay();  // Clear the display
    // display.setTextSize(2);  // Display the BPM
    // display.setTextColor(WHITE);
    // display.setCursor(50, 0);
    // display.println("BPM");

    // Check for heartbeat and calculate BPM
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;  // Measure duration between two beats
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);  // Calculate the BPM

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        // Calculate the average BPM
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;  // Wrap variable

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
      }

      // Display the average BPM on OLED
      // display.setCursor(50, 18);
      // display.println(beatAvg);
      // display.display();

      // Send the BPM via BLE
      sendHRM(beatAvg);
    }
  }

  // If no finger detected, show message on OLED
  if (irValue < 7000) {
    beatAvg = 0;
    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(WHITE);
    // display.setCursor(30, 5);
    // display.println("Please Place ");
    // display.setCursor(30, 15);
    // display.println("your finger ");
    // display.display();
    // noTone(3);  // Stop buzzer
  }

  timeChar.setWriteCallback(timeChar_write_callback);
}



void sendHRM(uint8_t bpm) {
  uint8_t payload[2] = {0b00000000, bpm};
  hrm.notify(payload, sizeof(payload));
}




void timeChar_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len != 7) {
    Serial.println("Invalid data length. Expected 7 bytes.");
    return;
  }

  if (data[0] == 0) {  // RTC sync
    rtc_hour = data[2];
    rtc_minute = data[3];
    rtc_second = data[4];
    Serial.println(" RTC synced.");
    return;
  }

  byte newIndex = data[0];
  bool updated = false;

  for (int i = 0; i < alarmCount; i++) {
    if (receivedAlarms[i][0] == newIndex) {
      memcpy(receivedAlarms[i], data, 7);
      updated = true;
      Serial.print(" Updated alarm at slot ");
      Serial.println(i);
      break;
    }
  }

  if (!updated && alarmCount < MAX_ALARMS) {
    memcpy(receivedAlarms[alarmCount], data, 7);
    alarmCount++;
    Serial.println(" Added new alarm.");
  }

  // Print all alarms
  Serial.println(" Current Alarms:");
  for (int i = 0; i < alarmCount; i++) {
    Serial.printf("Alarm %d: ", i);
    for (int j = 0; j < 7; j++) {
      Serial.print(receivedAlarms[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}




void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("Disconnected");
}


// #include <Ds1302.h>

// // Define the pins connected to DS1302
// #define CE_PIN    9   // Chip Enable (RST pin)
// #define IO_PIN    10  // Data (I/O pin)
// #define SCLK_PIN  8   // Clock (SCLK pin)

// DS1302 rtc(SCLK_PIN, IO_PIN, CE_PIN);  // Initialize DS1302 with pins

// void setup() {
//   Serial.begin(9600);
//   rtc.begin();
// }

// void loop() {
//   // Get the current time from DS1302
//   int hour = rtc.hour();
//   int minute = rtc.minute();
//   int second = rtc.second();

//   // Display the time on the serial monitor
//   Serial.print("Time: ");
//   Serial.print(hour);
//   Serial.print(":");
//   Serial.print(minute);
//   Serial.print(":");
//   Serial.println(second);

//   delay(1000);  // Wait for 1 second
// }
