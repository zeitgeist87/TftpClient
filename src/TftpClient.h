/*
 * Copyright 2020 Andreas Rohner
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TFTP_CLIENT_H
#define TFTP_CLIENT_H

#include <Client.h>
#include <Udp.h>
#include <IPAddress.h>
#include <Arduino.h>

template<class UDPImpl>
class TftpClient : public Stream {
 public:
  uint8_t begin(uint16_t local_port = 0) {
    if (local_port == 0) {
      // Use a different port for every instance
      static uint16_t stored_local_port;
      local_port = ++stored_local_port;
      if (local_port < 10000) {
        local_port += 10000;
      }
    }

    local_port_ = local_port;
    return udp_.begin(local_port);
  }

  void stop() {
    connection_state_ = ConnectionState::STOPPED;
    error_message_ = String();
    current_block_id_ = 0;
    udp_.stop();
  }

  bool beginDownload(const char *name,
                     const IPAddress &tftp_server_ip = IPAddress(255, 255, 255, 255),
                     uint16_t tftp_server_port = 69) {
    stop();
    begin(local_port_);

    // Create read request
    if (!udp_.beginPacket(tftp_server_ip, tftp_server_port))
      return false;

    udp_.write('\0');
    udp_.write(OpCode::READ);
    udp_.print(name);
    udp_.write('\0');
    udp_.print("octet");
    udp_.write('\0');
    udp_.print("blksize");
    udp_.write('\0');
    udp_.print(BLOCK_SIZE);
    udp_.write('\0');

    if (!udp_.endPacket())
      return false;

    current_block_id_ = 1;
    connection_state_ = ConnectionState::TRANSFERING;
    last_action_ts_ = millis();
    return true;
  }

  int available() override {
    if (finished() || error() || stopped())
      return 0;

    int available_bytes = udp_.available();
    if (available_bytes)
      return available_bytes;

    int packet_size = udp_.parsePacket();
    if (packet_size == 0) {
      if (millis() - last_action_ts_ > TIMEOUT_MS) {
        abort(0, "Timeout of connection");
      }
      return 0;
    }

    last_action_ts_ = millis();

    // Read first byte of the opcode
    if (udp_.read() != 0) {
      abort(0, "Unexpected OpCode");
      return 0;
    }

    switch (udp_.read()) {
      case OpCode::DATA: {
        handleDataBlock();
        return udp_.available();
      }
      case OpCode::OPT_ACK: {
        handleOptAck();
        break;
      }
      case OpCode::ERROR: {
        handleError();
        break;
      }
      default:
        // Invalid opcode
        abort(0, "Unexpected OpCode");
        break;
    }

    return 0;
  }

  int read(uint8_t *buf, size_t size) {
    if (available() == 0)
      return 0;

    int bytes_read = udp_.read(buf, size);

    checkAcknowledge();
    return bytes_read;
  }

  int read() override {
    uint8_t val;
    if (read(&val, 1) != 1)
      return 0;
    return val;
  }

  int peek() override {
    return udp_.peek();
  }

  void flush() override {
    udp_.flush();
  }

  bool finished() {
    return connection_state_ == ConnectionState::FINISHED;
  }

  bool error() {
    return connection_state_ == ConnectionState::ERROR;
  }

  bool stopped() {
    return connection_state_ == ConnectionState::STOPPED;
  }

  const String &errorMessage() {
    return error_message_;
  }

  size_t write(uint8_t) override {}
  size_t write(const uint8_t *buf, size_t size) {}

  IPAddress remoteIP() {
    return udp_.remoteIP();
  }
  uint16_t remotePort() {
    return udp_.remotePort();
  }

 private:
  void handleDataBlock() {
    uint16_t block_id = udp_.read();
    block_id <<= 8;
    block_id |= udp_.read();

    if (block_id > current_block_id_) {
      // Something went wrong
      abort(0, "Unexpected Block Number");
      return;
    }

    if (block_id < current_block_id_) {
      // Ignore the packet
      uint8_t buf[128];
      while (udp_.available())
        udp_.read(buf, sizeof(buf));

      // Resend the acknowledge for the block
      sendAcknowledge(block_id);
    }

    if (udp_.available() < BLOCK_SIZE) {
      // Last packet is smaller
      connection_state_ = ConnectionState::LAST_PACKET;
      checkAcknowledge();
    }
  }

  void handleError() {
    uint16_t error_code = udp_.read();
    error_code <<= 8;
    error_code |= udp_.read();

    char buf[128];
    udp_.read(buf, sizeof(buf) - 1);
    // Add terminating 0
    buf[sizeof(buf) - 1] = 0;

    abort(error_code, buf, false);
  }

  void handleOptAck() {
    // Read all of the packet
    uint8_t buf[32];
    while (udp_.available())
      udp_.read(buf, sizeof(buf));

    // Acknowledge block 0
    sendAcknowledge(0);
  }

  void sendAcknowledge(uint16_t block_id) {
    // Create read request
    if (!udp_.beginPacket(remoteIP(), remotePort())) {
      abort();
      return;
    }

    udp_.write('\0');
    udp_.write(OpCode::ACK);
    udp_.write(block_id >> 8);
    udp_.write(block_id & 0xFF);

    if (!udp_.endPacket()) {
      abort();
      return;
    }
  }

   void sendError(uint16_t error_code = 0,
                  const char *error_msg = "Unknown error") {
    // Create read request
    if (!udp_.beginPacket(remoteIP(), remotePort())) {
      return;
    }

    udp_.write('\0');
    udp_.write(OpCode::ERROR);
    udp_.write(error_code >> 8);
    udp_.write(error_code & 0xFF);
    udp_.print(error_msg);
    udp_.write('\0');

    if (!udp_.endPacket()) {
      return;
    }
  }

  void checkAcknowledge() {
    // Send acknowledge after all bytes have been read
    if (available() == 0) {
      flush();

      sendAcknowledge(current_block_id_);
      ++current_block_id_;

      if (connection_state_ == ConnectionState::LAST_PACKET) {
        stop();
        connection_state_ = ConnectionState::FINISHED;
      }
    }
  }

  void abort(uint16_t error_code = 0,
             const char *error_msg = "Unknown error",
             bool send_error = true) {
    if (send_error && (connection_state_ == ConnectionState::TRANSFERING ||
        connection_state_ == ConnectionState::LAST_PACKET)) {
      sendError(error_code, error_msg);
    }

    connection_state_ = ConnectionState::ERROR;
    error_message_ = error_msg;
    current_block_id_ = 0;
    udp_.stop();
  }

#ifdef ARDUINO_ARCH_AVR
  static constexpr unsigned int BLOCK_SIZE = 68;
#else
  static constexpr unsigned int BLOCK_SIZE = 512;
#endif
  static constexpr unsigned int TIMEOUT_MS = 2000;

  enum OpCode {
    READ = 1,
    WRITE,
    DATA,
    ACK,
    ERROR,
    OPT_ACK
  };

  enum class ConnectionState {
    STOPPED,
    FINISHED,
    TRANSFERING,
    LAST_PACKET,
    ERROR
  };

  // Connection status
  unsigned long last_action_ts_ = 0;
  ConnectionState connection_state_ = ConnectionState::STOPPED;
  String error_message_;
  uint16_t current_block_id_ = 0;

  // Port to listen to
  uint16_t local_port_ = 0;
  UDPImpl udp_;
};

#endif  // TFTP_CLIENT_H