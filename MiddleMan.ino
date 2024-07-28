#include <SoftwareSerial.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04

int numberToSend = 1;  // Define the number to send
int flagPlant = 0;
int flagMove = 0;
int val, flag = 0;

SoftwareSerial mySerial(4, 3);  //rx, tx

void setup() {
    Serial.begin(9600);  // Start serial communication for debugging
    mySerial.begin(9600);   // Initialize software serial for sending data
    Wire.begin(SLAVE_ADDRESS);  // Initialize I2C as a slave device
    Wire.onReceive(receiveData);  // Register function to receive data
    Wire.onRequest(sendData);  // Register function to send data
    Serial.println("Ready!");
}

void loop() {
  if (mySerial.available() > 0) {  // Only enter the loop if data is available
    char receivedChar = mySerial.read(); // read one byte from serial buffer and save to receivedData
    if (receivedChar == '1') { // receive from Mega
      // send data to EV3 for planting
      Serial.print("Received from Mega: "); //received with love from Mega
      Serial.println(receivedChar);
      flagPlant = 1; 
    }
    else if (receivedChar == '0') {
        // do nothing
    }
  }

  if (flag == 1) {
      Serial.print("Received from EV3: "); // received from EV3
      Serial.println(val);
      flag = 0;
      if (val=2) {
        flagMove = 1;
        val=0;
      }

      if (val == 7) {  // Check if the received value is 7
          Serial.println("ClementShuai");
          delay(5000);  // Wait for 5 seconds
          numberToSend = 1;  // Set the value to send back to 1
          Serial.println("Order to plant again sent");
          val = 0;
      }
  }

  if (flagMove == 1) {
    Serial.print("Sending to Mega: "); // send to Mega
    //char inputChar = '2';
    Serial.println(flagMove);
    mySerial.println(flagMove);  // Send the message through mySerial with a newline character
    flagMove=0;
    //inputChar = "";
  }

    
    
}

// Callback for receiving data
void receiveData(int byteCount) {
    while (Wire.available() > 0) {
        val = Wire.read();
        flag = 1;
    }
}

// Callback for sending data
void sendData() {
    if (flagPlant == 1) {  // Only send if flagPlant is not 0
      Serial.print("Sending to EV3: ");
      Serial.println(flagPlant);
      Wire.write(flagPlant);
      flagPlant = 0;  // Reset flagPlant after sending
    }
}