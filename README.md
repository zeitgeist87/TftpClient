# Arduino TFTP Client Library

This is a simple TFTP Client Library using the Arduino's `UDP` interface. By using the standard interface it is compatible with any device that offers an implementation of this interface. This includes the ESP32, ESP8266, MKR1000 and many more. It should also work with older Arduino Shields like the WiFi Shield or the Ethernet Shield.

## Usage

The TFTP Client class implements the Arduino Stream interface, so its usage should be familiar to any Arduino programmer. There are comprehensive examples provided in the `examples` directory.

```cpp
TftpClient<WiFiUDP> tftp;

void setup() {
  // Initilize hardware serial
  Serial.begin(115200);

  // Setup your Wifi or Ethernet
  //setupWiFi();

  Serial.println("\nStarting connection to server...");

  // Begin downloading uses default port and broadcast IP
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
```

## Getting Started

### Linux

* Install a TFTP server for your Linux Distro. For example:
  `sudo pacman -S tftp-hpa`
* Start the `tftpd` service: `sudo systemctl start tftpd`
* Copy some files to the TFTP directory `sudo cp test.txt /srv/tftp`

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

This library was sponsored by [Jonathan Beri](https://github.com/beriberikix) via Upwork.