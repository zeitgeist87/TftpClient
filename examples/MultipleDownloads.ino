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
 * IP Address of the TFTP server. Use broadcast by default for
 * easier setup. Change to the actual IP of your TFTP server.
 */
const IPAddress tftp_server_ip = IPAddress(255, 255, 255, 255);
//const IPAddress tftp_server_ip = IPAddress(192, 168, 0, 22);
const uint16_t tftp_server_port = 69;

// Wifi credentials
const char *ssid = "...";
const char *password = "....";

/*
 * Declaration of the TftpClient. The template
 * parameter sets the platform specific UDP
 * implementation.
 */
TftpClient<WiFiUDP> client1;
TftpClient<WiFiUDP> client2;

// Buffer to hold incoming data
uint8_t packet_buffer[256];

void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);

  setupWiFi();
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");

  // Begin downloading multiple files at once
  client1.beginDownload("test.txt", tftp_server_ip, tftp_server_port);
  client2.beginDownload("test.txt", tftp_server_ip, tftp_server_port);
}

void loop() {
  downloadWithClient(client1, 1);
  downloadWithClient(client2, 2);
}

void downloadWithClient(TftpClient<WiFiUDP> &client, int client_id) {
  if (client.available()) {
    int bytes_read = client.read(packet_buffer, sizeof(packet_buffer));

    // Print the downloaded text
    Serial.print("Client ");
    Serial.print(client_id);
    Serial.print(" Received ");
    Serial.print(bytes_read);
    Serial.println(" bytes:");

    Serial.write(packet_buffer, bytes_read);
  }

  if (client.error()) {
    Serial.print("Client ");
    Serial.print(client_id);
    Serial.print(" Error occurred:");
    Serial.println(client.errorMessage());
    client.stop();
  }

  if (client.finished()) {
    Serial.print("Client ");
    Serial.print(client_id);
    Serial.println(" Transfer finished!");
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