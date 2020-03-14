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
  /**
   * Initializes this instance with a transaction id (UDP port)
   *
   * This method allows the user to set the transaction ID explicitly. Usually
   * this is not necessary unless there is a conflict with another UDP program.
   * The transaction id is the UDP port the Arduino listens on for packets from
   * the server. However, it is not the UDP port of the server which is usually
   * 69. The server port can be set with the @see beginDownload() method.
   *
   * @param local_port The UDP port used by the Arduino for this instance.
   *                   Acts as a transaction id. Defaults to 0 which means, a
   *                   unique id will be automatically generated.
   * @returns 1 if successful and 0 otherwise.
   */
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

  /**
   * Stops the current transaction and resets the internal state
   */
  void stop() {
    connection_state_ = ConnectionState::STOPPED;
    error_message_ = String();
    current_block_id_ = 0;
    udp_.stop();
  }

  /**
   * Begins the download of a file from the server
   *
   * @param name              File name to download
   * @param tftp_server_ip    IP address of the TFTP server. Defaults to
   *                          the broadcast IP. Which should work with servers
   *                          on the same subnet as the client.
   * @param tftp_server_port  UDP port of the TFTP server. Defaults to 69.
   *
   * @returns true if successful and false otherwise
   */
  bool beginDownload(const char *name,
                     const IPAddress &tftp_server_ip = IPAddress(255, 255, 255, 255),
                     uint16_t tftp_server_port = 69) {
    stop();
    if (!begin(local_port_))
      return false;

    if (is_same<UIP_UDPImpl, UDPImpl>::value) {
      /*
       * The UIPUDP library cannot handle the fact, that the TFTP server
       * responds on a different port than 69. So it is necessary to use a
       * different socket for the initial packet.
       */
      UDPImpl init;

      init.begin(local_port_);

      if (!sendReadRequest(name, tftp_server_ip, tftp_server_port, &init))
        return false;

      // Wait until the first response from the server arrives
      while (!error() && !finished() && !available()) {
        yield();
      }

      // Release memory
      init.stop();
      return error() == false;
    } else {
      if (!sendReadRequest(name, tftp_server_ip, tftp_server_port, &udp_))
        return false;
    }

    return true;
  }

  /**
   * Returns the number of bytes that are available for reading
   *
   * @returns Number of bytes available or 0
   */
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

  /**
   * Reads data from the TFTP file into a buffer
   *
   * @param buf  Pointer to a byte buffer
   * @param size Size of the buffer
   *
   * @returns Number of bytes actually read. Can be smaller than @p size
   */
  int read(uint8_t *buf, size_t size) {
    if (available() == 0)
      return 0;

    int bytes_read = udp_.read(buf, size);

    checkAcknowledge();
    return bytes_read;
  }

  /**
   * Reads a single byte
   */
  int read() override {
    uint8_t val;
    if (read(&val, 1) != 1)
      return 0;
    return val;
  }

  /**
   * Reads a single byte without consuming it.
   */
  int peek() override {
    return udp_.peek();
  }

  void flush() override {
    udp_.flush();
  }

  /**
   * Returns true if the transfer finished successfully
   */
  bool finished() {
    return connection_state_ == ConnectionState::FINISHED;
  }

  /**
   * Returns true if there was an error during the transfer
   *
   * If this method returns true, then the current error message is available
   * with the method @see errorMessage()
   */
  bool error() {
    return connection_state_ == ConnectionState::ERROR;
  }

  /**
   * Returns true if the stop function was called
   */
  bool stopped() {
    return connection_state_ == ConnectionState::STOPPED;
  }

  /**
   * Returns the current error message as a String object
   */
  const String &errorMessage() {
    return error_message_;
  }

  size_t write(uint8_t) override {
    // Not implemented yet
    return 0;
  }
  size_t write(const uint8_t *buf, size_t size) {
    // Not implemented yet
    return 0;
  }

  /**
   * Returns the IP address of the TFTP server this instance is currently
   * connected to
   */
  IPAddress remoteIP() {
    return udp_.remoteIP();
  }

  /**
   * Returns the port number of the current connection
   */
  uint16_t remotePort() {
    return udp_.remotePort();
  }

  ~TftpClient() {
    // Free up resources
    udp_.stop();
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

    udp_.write(ZERO_BYTE);
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

    udp_.write(ZERO_BYTE);
    udp_.write(OpCode::ERROR);
    udp_.write(error_code >> 8);
    udp_.write(error_code & 0xFF);
    udp_.print(error_msg);
    udp_.write(ZERO_BYTE);

    if (!udp_.endPacket()) {
      return;
    }
  }

  bool sendReadRequest(const char *name,
                       const IPAddress &tftp_server_ip,
                       uint16_t tftp_server_port,
                       UDPImpl *udp) {
        // Create read request
    if (!udp->beginPacket(tftp_server_ip, tftp_server_port))
      return false;

    udp->write(ZERO_BYTE);
    udp->write(OpCode::READ);
    udp->print(name);
    udp->write(ZERO_BYTE);
    udp->print("octet");
    udp->write(ZERO_BYTE);
    udp->print("blksize");
    udp->write(ZERO_BYTE);
    udp->print(BLOCK_SIZE);
    udp->write(ZERO_BYTE);

    if (!udp->endPacket())
      return false;

    current_block_id_ = 1;
    connection_state_ = ConnectionState::TRANSFERING;
    last_action_ts_ = millis();

    return true;
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
  static constexpr int BLOCK_SIZE = 96;
#else
  static constexpr int BLOCK_SIZE = 512;
#endif
  static constexpr unsigned int TIMEOUT_MS = 2000;
  static constexpr uint8_t ZERO_BYTE = 0;

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

  /*
   *  Some utility functions and classes to detect the UDPImpl capabilities
   */
#ifdef UIPUDP_H
  using UIP_UDPImpl = UIPUDP;
#else
  using UIP_UDPImpl = void;
#endif

  // Some template magic to detect types
  template<typename _Tp, _Tp __v>
  struct integral_constant {
    static constexpr _Tp                  value = __v;
  };

  template<typename, typename>
  struct is_same : public integral_constant<bool, false> { };

  template<typename _Tp>
  struct is_same<_Tp, _Tp> : public integral_constant<bool, true> { };
};

#endif  // TFTP_CLIENT_H