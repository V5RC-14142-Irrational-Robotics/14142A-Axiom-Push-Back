#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "liblvgl/llemu.hpp"
#include <functional>
#include <string>
#include <vector>

class Telemetry {
public:
  Telemetry();
  void clear();

  void addLine(const std::string &label, const std::string &value);

  void addLine(const std::string &label, int value);
  void addLine(const std::string &label, double value);

  void addLine(const std::string &label,
               std::function<std::string()> getter);

  void display();

private:
  std::vector<std::pair<std::string,
    std::function<std::string()>>> lines;

  void registerCore();
};

#endif