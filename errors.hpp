#include <iostream>
#include <stdexcept>
#include <string>

class TimeoutError : public std::runtime_error {
   public:
    TimeoutError(const std::string& msg) : std::runtime_error(msg) {}
};

class BrokenPipeError : public std::runtime_error {
   public:
    BrokenPipeError(const std::string& msg) : std::runtime_error(msg) {}
};