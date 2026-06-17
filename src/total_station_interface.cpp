/**
 * \file total_station_interface.cpp
 * \author Wenda Zhao
 * \date 05.11.2025
 * \brief Implementation of the total station interface base class
 */

#include <iostream>
#include <vector>

#include <boost/lexical_cast.hpp>

#include "leica_serial_ros2/total_station_interface.h"

TSInterface::TSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : tsState_(TSState::on),
    prismPosition_(3),
    io_context_(new boost::asio::io_service()),
    timerStartedFlag_(false),
    timer_(*io_context_, boost::posix_time::seconds(2)),
    messagesReceivedFlag_(false),
    missedMessageCount_(0),
    searchingPrismFlag_(false),
    externalPositionReceivedFlag_(false),
    locationCallback_(locationCallback)
{}

void TSInterface::startTimer() {
  std::cout << "Start timer" << std::endl;
  timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(2));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

void TSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::on;
}

void TSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::off;
}

void TSInterface::setPrismPosition(double x, double y, double z) {
  prismPosition_[0] = x;
  prismPosition_[1] = y;
  prismPosition_[2] = z;

  // Set external position
  std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
  externalPositionReceivedFlag_ = true;
}

void TSInterface::timerHandler() {
  if (TSState::on == tsState_) {
    bool prism_lost = false;
    bool has_external_position = false;

    // Check flags while holding their mutexes, but send commands after the
    // locks are released. turnTelescope() and searchPrism() also lock state.
    {
      std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
      std::lock_guard<std::mutex> guard2(searchingPrismMutex_);
      if (messagesReceivedFlag_) {
        missedMessageCount_ = 0;
      } else if (!searchingPrismFlag_) {
        ++missedMessageCount_;
      }
      prism_lost = missedMessageCount_ >= 3 && !searchingPrismFlag_;
      if (prism_lost) {
        missedMessageCount_ = 0;
      }
      messagesReceivedFlag_ = false;
    }

    {
      std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
      has_external_position = externalPositionReceivedFlag_;
      externalPositionReceivedFlag_ = false;
    }

    // Start to search the prism if no message was received
    if (prism_lost) {
      std::cout << "Prism lost!" << std::endl;

      // Turn total station to prism if a recent external position was received
      if (has_external_position) {
        turnTelescope();
      }

      searchPrism();
    }
  }

  // Restart timer
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(800));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

void TSInterface::searchPrism(void) {
  {
    std::lock_guard<std::mutex> guard(searchingPrismMutex_);
    searchingPrismFlag_ = true;
  }
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '6', ':', '1', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);

  std::cout << "Search prism" << std::endl;
}

void TSInterface::turnTelescope(void) {
  {
    std::lock_guard<std::mutex> guard(searchingPrismMutex_);
    searchingPrismFlag_ = true;
  }
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '7', ':', '1'};
  std::string y = boost::lexical_cast<std::string>(prismPosition_[1]);
  for (char c : y) {
    command.emplace_back(c);
  }
  std::string x = boost::lexical_cast<std::string>(prismPosition_[0]);
  for (char c : x) {
    command.emplace_back(c);
  }
  std::string z = boost::lexical_cast<std::string>(prismPosition_[2]);
  for (char c : z) {
    command.emplace_back(c);
  }
  command.emplace_back(0x0d/*CR*/);
  command.emplace_back(0x0a/*LF*/);

  write(command);
}
