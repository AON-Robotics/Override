#pragma once
#ifndef AON_TOOLS_GUI_LAYOUT_HPP_
#define AON_TOOLS_GUI_LAYOUT_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include "../../../../api.h"
#include "aon/constants.hpp"

namespace aon {
namespace ui {

class Button {
 public:
  int x1, y1, x2, y2;
  std::string label;
  pros::Color bg;
  pros::Color fg;
  std::function<void()> onPress = nullptr;

  Button(int x1 = 0, int y1 = 0, int x2 = 0, int y2 = 0, const std::string& label = "",
         pros::Color bg = pros::Color::black, pros::Color fg = pros::Color::white,
         std::function<void()> onPress = nullptr)
      : x1(x1), y1(y1), x2(x2), y2(y2), label(label), bg(bg), fg(fg), onPress(onPress) {}

  void draw(int textFmt = pros::E_TEXT_MEDIUM) const {
    pros::screen::set_eraser(static_cast<uint32_t>(bg));
    pros::screen::erase_rect(x1, y1, x2, y2);

    int charWidth = 12;
    int charHeight = 16;
    if (textFmt == pros::E_TEXT_SMALL) {
      charWidth = 8;
      charHeight = 12;
    } else if (textFmt == pros::E_TEXT_LARGE) {
      charWidth = 18;
      charHeight = 20;
    }

    int textWidth = static_cast<int>(label.size()) * charWidth;
    int textX = x1 + (x2 - x1 - textWidth) / 2;
    int textY = y1 + (y2 - y1 - charHeight) / 2;

    pros::screen::set_pen(static_cast<uint32_t>(fg));
    pros::screen::print(static_cast<pros::text_format_e_t>(textFmt), textX, textY, label.c_str());
  }

  bool isHit(int x, int y) const {
    return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
  }
};

}  // namespace ui

// Back buttons
inline const ui::Button backBtnGray  = {10, 10, 70, 40, "BACK", pros::Color::dark_gray,  pros::Color::white};
inline const ui::Button backBtnRed   = {10, 10, 70, 40, "BACK", pros::Color::dark_red,   pros::Color::white};
inline const ui::Button BackBtnBlue  = {10, 10, 70, 40, "BACK", pros::Color::dark_blue,  pros::Color::white};
inline const ui::Button BackBtnGreen = {10, 10, 70, 40, "BACK", pros::Color::dark_green, pros::Color::white};

// Main menu buttons
inline const ui::Button AutonsBtn = {0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT, "AUTONS", pros::Color::green, pros::Color::white};
inline const ui::Button redBtn    = {45, BRAIN_SCREEN_HEIGHT / 2 - 30, 145, BRAIN_SCREEN_HEIGHT / 2 + 30, "RED", pros::Color::red, pros::Color::white};
inline const ui::Button blueBtn   = {BRAIN_SCREEN_WIDTH - 130, BRAIN_SCREEN_HEIGHT / 2 - 30, BRAIN_SCREEN_WIDTH - 30, BRAIN_SCREEN_HEIGHT / 2 + 30, "BLUE", pros::Color::blue, pros::Color::white};
inline const ui::Button skillsBtn = {BRAIN_SCREEN_WIDTH / 2 - 60, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 80, BRAIN_SCREEN_HEIGHT - 50, "SKILLS", pros::Color::green, pros::Color::white};

// Auton selection buttons
inline const ui::Button aut1Btn = {50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50, "AUT1", pros::Color::black, pros::Color::black};
inline       ui::Button Aut2Btn = {BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50, "AUT2", pros::Color::black, pros::Color::black};
inline const ui::Button Aut3Btn = {BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50, "AUT3", pros::Color::black, pros::Color::black};

}  // namespace aon

#endif  // AON_TOOLS_GUI_LAYOUT_HPP_
