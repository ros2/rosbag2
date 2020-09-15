#pragma once

#include <any>
#include <iostream>

namespace vtr {
namespace storage {

class VTRMessage {
 public:
  VTRMessage() = default;
  template <class T>
  VTRMessage(T message) : message_{message} {}

  virtual ~VTRMessage() = default;

  template <class T>
  VTRMessage& operator=(const T& message) {
    message_ = std::make_any<T>(message);
    return *this;
  }

  template <class T>
  void set(T message) {
    message_ = std::make_any<T>(message);
  }

  template <class T>
  T get() const {
    try {
      return std::any_cast<T>(message_);
    } catch (const std::bad_any_cast& e) {
      std::stringstream ss;
      ss << "Any cast failed in retrieving data in VTR Storage. Error: "
         << e.what();
      throw std::runtime_error(ss.str());
    }
  }

 private:
  std::any message_;
};

}  // namespace storage
}  // namespace vtr