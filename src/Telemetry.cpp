#include "Telemetry.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "pros/colors.hpp"

Telemetry::Telemetry() {
  registerCore();
}

void Telemetry::clear() {
  lines.clear();
  registerCore();
}

void Telemetry::addLine(const std::string &label,
                        const std::string &value) {
  lines.emplace_back(label, [value]{ return value; });
}

void Telemetry::addLine(const std::string &label, int value) {
  lines.emplace_back(label,
    [value]{ return std::to_string(value); });
}

void Telemetry::addLine(const std::string &label, double value) {
  lines.emplace_back(label,
    [value]{ return std::to_string(value); });
}

void Telemetry::addLine(const std::string &label,
                        std::function<std::string()> getter) {
  lines.emplace_back(label, std::move(getter));
}

void Telemetry::display() {
  pros::lcd::clear();
  for (std::int16_t i = 0; i < lines.size() && i < 8; ++i) {
    auto &entry = lines[i];
    std::string text = entry.first + ": " + entry.second();
    pros::screen::set_pen(pros::Color::black);
    pros::screen::print(pros::E_TEXT_MEDIUM, i, text.c_str());
  }
}

void Telemetry::registerCore() {
  addLine("Uptime", []{
    return std::to_string(pros::c::millis()) + " ms";
  });
}