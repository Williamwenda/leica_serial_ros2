/**
 * \file serial_total_station_interface.cpp
 * \author Wenda Zhao
 * \date 05.11.2025
 * \brief Implementation of the serial total station interface
 */

#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/algorithm/string.hpp>

#include "leica_serial_ros2/serial_total_station_interface.h"

SerialTSInterface::SerialTSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : TSInterface(locationCallback),
    serial_port_(*io_context_) {}

SerialTSInterface::~SerialTSInterface() {
  serial_port_.cancel();
  serial_port_.close();
  if (contextThread_.joinable()) {
    contextThread_.join();
  }
}

void SerialTSInterface::connect(std::string comport) {
  try {
    // Connect to total station and call startReader and startTimer
    // if successfull
    boost::system::error_code ec;

    // what baud rate do we communicate at
    boost::asio::serial_port_base::baud_rate BAUD(115200);
    // how big is each "packet" of data (default is 8 bits)
    boost::asio::serial_port_base::character_size C_SIZE(8);
    // what flow control is used (default is none)
    boost::asio::serial_port_base::flow_control FLOW(boost::asio::serial_port_base::flow_control::none);
    // what parity is used (default is none)
    boost::asio::serial_port_base::parity PARITY(boost::asio::serial_port_base::parity::none);
    // how many stop bits are used (default is one)
    boost::asio::serial_port_base::stop_bits STOP(boost::asio::serial_port_base::stop_bits::two);

    std::cout << "get into the connect function" << std::endl;

    serial_port_.open(comport, ec);
    if (ec) {
      std::cerr << "Failed to open serial port: " << ec.message() << std::endl;
      return;
    }
    
    serial_port_.set_option(BAUD);
    serial_port_.set_option(C_SIZE);
    serial_port_.set_option(FLOW);
    serial_port_.set_option(PARITY);
    serial_port_.set_option(STOP);
    std::cout << "finished setting up" << std::endl;

    if (!ec) {
      std::cout << "Starting reader..." << std::endl;
      startReader();
      std::cout << "Reader started successfully" << std::endl;
    }

    // Start io_context in separate thread
    std::cout << "Starting io_context thread..." << std::endl;
    contextThread_ = std::thread([this](){ 
      try {
        std::cout << "io_context thread started, running..." << std::endl;
        io_context_->run(); 
        std::cout << "io_context thread finished" << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "Exception in io_context thread: " << e.what() << std::endl;
      }
    });
    std::cout << "io_context thread created successfully" << std::endl;
    
    // Schedule automatic start using a timer to avoid race conditions
    if (!ec) {
      auto startTimer = std::make_shared<boost::asio::deadline_timer>(*io_context_, boost::posix_time::milliseconds(1000));
      startTimer->async_wait([this, startTimer](const boost::system::error_code& timer_ec) {
        if (!timer_ec) {
          std::cout << "Starting total station measurements automatically..." << std::endl;
          start();
        }
      });
    }
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void SerialTSInterface::startReader() {
  boost::asio::async_read_until(serial_port_,
                                readData_,
                                "\r\n",
                                std::bind(&SerialTSInterface::readHandler,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2)
                                );
}

void SerialTSInterface::write(std::vector<char> command) {
  boost::asio::async_write(serial_port_,
                           boost::asio::buffer(command),
                           std::bind(&SerialTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void SerialTSInterface::writeHandler(const boost::system::error_code& ec,
                                  std::size_t /*bytes_transferred*/) {
  if (!ec) {
    std::cout << "Command sent." << std::endl;
  }
}

void SerialTSInterface::readHandler(const boost::system::error_code& ec,
                              std::size_t bytes_transferred) {

  if (!ec) {
    // Convert streambuf to std::string
    boost::asio::streambuf::const_buffers_type bufs = readData_.data();
    std::string data(boost::asio::buffers_begin(bufs),
                     boost::asio::buffers_begin(bufs) + readData_.size());

    readData_.consume(bytes_transferred);

    // Print received message
    // std::cout << data << std::endl;

    // Check for responses if the total station searches the prism
    bool searching = false;
    {
      std::lock_guard<std::mutex> guard(searchingPrismMutex_);
      searching = searchingPrismFlag_;
    }

    if (searching) {
      // Catch the response
      if (data.find("%R8P,0,0:") != std::string::npos) {
        std::cout << "Got an answer." << std::endl;

        // Catch the negative response
        if (data.find(":31") != std::string::npos) {
          std::cout << "Prism not found!" << std::endl;
          searchPrism();
        } else if (data.find(":0") != std::string::npos) { // Catch the positive response
          std::cout << "Prism found" << std::endl;
          {
            std::lock_guard<std::mutex> guard(searchingPrismMutex_);
            searchingPrismFlag_ = false;
          }
          {
            std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
            messagesReceivedFlag_ = true;
          }
        }
      } else if (!data.empty() && data[0] == 'T') {
        // Total station resumed streaming while search flag is active
        std::cout << "Prism stream resumed while searching." << std::endl;
        std::lock_guard<std::mutex> guard(searchingPrismMutex_);
        searchingPrismFlag_ = false;
      }
    }

    if (!data.empty() && data[0] == 'T') { // Forward x, y and z coordinate if location was received
      // Split the received message to access the coordinates
      std::vector<std::string> results;
      boost::split(results, data, [](char c){return c == ',';});

      if (results.size() >= 4) {
        double x = std::stod(results[2]);  // east axis
        double y = std::stod(results[1]);  // north axis
        double z = std::stod(results[3]);

        locationCallback_(x, y, z);

        // Indicate that a message was received
        std::lock_guard<std::mutex> guard(messageReceivedMutex_);
        messagesReceivedFlag_ = true;

        // Start timer if it is not yet started
        if (!timerStartedFlag_) {
          startTimer();
          timerStartedFlag_ = true;
        }
      } else {
        std::cerr << "Malformed coordinate message: " << data << std::endl;
      }
    }

    // Restart reading
    boost::asio::async_read_until(serial_port_,
                                  readData_,
                                  "\r\n",
                                  std::bind(&SerialTSInterface::readHandler,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)
                                 );
  }
}

void SerialTSInterface::searchPrism(void) {
  {
    std::lock_guard<std::mutex> guard(searchingPrismMutex_);
    searchingPrismFlag_ = true;
  }
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '6', ':', '1', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);

  std::cout << "Search prism" << std::endl;
}