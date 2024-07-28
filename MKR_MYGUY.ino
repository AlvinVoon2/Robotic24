/**
 * i2c network: send parameter to client and receive response
 * with data relative to the request. SLAVE SKETCH
 *
 * by Renzo Mischianti <www.mischianti.org>
 *
 * https://mischianti.org
 *
 * Arduino UNO <------>   Logic converter <------>  Arduino MKR
 * GND                      GND         GND             GND
 * 5v                       HV          LV              3.3v
 * A4                       HV1         LV1             11
 * A5                       HV2         LV2             12
 *
 */

#include <Wire.h>
#include "Firebase_Arduino_WiFiNINA.h"

#define DATABASE_URL "robotic2024-ccec0-default-rtdb.asia-southeast1.firebasedatabase.app"
#define DATABASE_SECRET "QkyLYF3enr9eh3MMIYIpaTvcYJEe9IT1bg7CBR3C"
#define WIFI_SSID "iPhone de Tad"
#define WIFI_PASSWORD "bromo1262"

// Enum for I2C request types
enum I2C_REQUEST_TYPE {
    NONE = -1,
    GET_DATA = 0
};

FirebaseData fbdo;
I2C_REQUEST_TYPE request = NONE;
int dataToSend = 0;

void requestEvent();
void receiveEvent(int numBytes);

void setup() {
    Wire.begin(0x10);  // join i2c bus with address 0x10

    Serial.begin(115200);
    while (!Serial) {}

    Serial.println();
    Serial.print("Connecting to Wi-Fi");
    int status = WL_IDLE_STATUS;
    while (status != WL_CONNECTED) {
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Firebase.begin(DATABASE_URL, DATABASE_SECRET, WIFI_SSID, WIFI_PASSWORD);
    Firebase.reconnectWiFi(true);

    Wire.onReceive(receiveEvent);  // register an event handler for received data
    Wire.onRequest(requestEvent);  // register an event handler for data requests
}

void loop() {
    String path = "/gap";
    if (Firebase.getInt(fbdo, path)) {
        dataToSend = fbdo.intData();
        Serial.println("Data received from Firebase: " + String(dataToSend));
    } else {
        Serial.println("Error getting data: " + fbdo.errorReason());
    }
    delay(1000);  // wait for a second before next request
}

void requestEvent() {
    Wire.write(dataToSend);  // send data to master
}

void receiveEvent(int numBytes) {
    if (numBytes == 1) {
        int requestVal = Wire.read();
        request = static_cast<I2C_REQUEST_TYPE>(requestVal);
    }
}
