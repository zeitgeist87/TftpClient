#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#elif ARDUINO_ARCH_SAMD
#include <WiFi101.h>
#else
#include <WiFi.h>
#endif

#include <TftpClient.h>
#include <WiFiUdp.h>

// Wifi credentials
const char *ssid = "...";
const char *password = "...";

/*
 * Declaration of the TftpClient. The template
 * parameter sets the platform specific UDP
 * implementation.
 */
TftpClient<WiFiUDP> tftp;

void setup() {
  // Initilize hardware serial
  Serial.begin(115200);

  setupWiFi();

  Serial.println("\nStarting connection to server...");

  // Begin downloading uses default port and broaddcast IP
  tftp.beginDownload("test.txt");
}

void loop() {
  if (tftp.available()) {
    uint8_t buffer[32];
    int bytes_read = tftp.read(buffer, sizeof(buffer));

    Serial.write(buffer, bytes_read);
  }

  if (tftp.error()) {
    Serial.println("TFTP error occurred!");
    tftp.stop();
  }
}

void setupWiFi() {
#if ARDUINO_ARCH_SAMD
  int status = WL_IDLE_STATUS;
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, password);

    // wait 10 seconds for connection:
    delay(10000);
  }
#else
  //Connect to the WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while(1) {
      delay(1000);
    }
  }
#endif
}