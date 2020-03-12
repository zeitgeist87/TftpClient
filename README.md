# Arduino TFTP Client Library

This is a simple TFTP Client Library using the Arduino's `UDP` interface. By using the standard interface it is compatible with any device that offers an implementation of this interface. This includes the ESP32, ESP8266, MKR1000 and many more. It should also work with older Arduino Shields like the WiFi Shield or the Ethernet Shield.

## Usage

The TFTP Client class implements the Arduino Stream interface, so its usage should be familiar to any Arduino programmer. There are comprehensive examples provided in the `examples` directory.

```cpp
TftpClient<WiFiUDP> client;

// Buffer to hold incoming data
uint8_t packet_buffer[256];

void setup() {
  // Setup your network, wifi, etc

  // Set local port to listen to
  client.begin(local_port);

  // Start the download of a file using a broadcast by default
  client.beginDownload("test.txt");
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

    // Start another download
    client.beginDownload("another_test.txt");
  }

  if (client.finished()) {
    Serial.println("Transfer finished!");
    client.stop();
  }
}
```

## Features

### Supported Features

* File Download
* `blksize` Option

### Unsupported Features

* File Upload
* `tsize` and `timeout` Options
* Obscure and useless transfer modes like **Netascii** and **Mail transfer**

## License

This library is licenses under the Apache 2.0 Open Source License.

## Sponsor

This library was sponsored by Jonathan Beri.