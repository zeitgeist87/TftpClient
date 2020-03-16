#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#elif ARDUINO_ARCH_SAMD
#include <WiFi101.h>
#else
#include <WiFi.h>
#endif

#include <TftpClient.h>
#include <WiFiUdp.h>

/*
 * Local port the Arduino listens for responses from the server.
 * Can be any valid UDP port, but every TftpClient instance should
 * have a different one. It acts like a transaction ID.
 */
const uint16_t local_port = 2390;
/*
 * IP Address of the TFTP server. Use broadcast by default for
 * easier setup. Change to the actual IP of your TFTP server.
 */
const IPAddress tftp_server_ip = IPAddress(255, 255, 255, 255);
const uint16_t tftp_server_port = 69;

// Wifi credentials
const char *ssid = "...";
const char *password = "...";

/*
 * Declaration of the TftpClient. The template
 * parameter sets the platform specific UDP
 * implementation.
 */
TftpClient<WiFiUDP> client;

// Buffer to hold incoming data
uint8_t packet_buffer[256];

void setup() {
  // Initilize hardware serial
  Serial.begin(115200);

  setupWiFi();
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");

  /*
   * Set transaction ID, which is also the UDP port.
   * If you omit this call a suitable transaction ID will be
   * automactically chosen.
   */
  client.begin(local_port);

  // Start the download of a file using a broadcast by default
  client.beginDownload("test.txt", tftp_server_ip, tftp_server_port);
}

void loop() {
  if (client.available()) {

    int bytes_read = client.read(packet_buffer, sizeof(packet_buffer));

    // Print the downloaded text
    Serial.print("Received ");
    Serial.print(bytes_read);
    Serial.println(" bytes:");

    Serial.write(packet_buffer, bytes_read);
  }

  if (client.error()) {
    Serial.print("Error occurred: ");
    Serial.println(client.errorMessage());
    client.stop();
  }

  if (client.finished()) {
    Serial.println("Transfer finished!");
    client.stop();
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

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}