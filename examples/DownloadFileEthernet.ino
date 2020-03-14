#include <UIPEthernet.h>
#include <TftpClient.h>

/*
 * Declaration of the TftpClient. The template
 * parameter sets the platform specific UDP
 * implementation.
 */
TftpClient<EthernetUDP> tftp;

void setup() {
  // Initilize hardware serial
  Serial.begin(9600);

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
  Ethernet.begin(mac, IPAddress(192, 168, 0, 6));

  Serial.println("\nStarting connection to server...");

  // Begin downloading uses default port and broaddcast IP
  tftp.beginDownload("test.txt", IPAddress(192, 168, 0, 22));
}

void loop() {
  if (tftp.available()) {
    uint8_t buffer[32];
    int bytes_read = tftp.read(buffer, sizeof(buffer));

    Serial.write(buffer, bytes_read);
  }

  if (tftp.error()) {
    Serial.print("Error occurred: ");
    Serial.println(tftp.errorMessage());
    tftp.stop();
  }
}