#include "../../../../../include/aon/tools/gui/gui.hpp"
#include "../../../../../include/aon/tools/gui/gui-debug.hpp"
#include "../../../../../include/aon/constants.hpp"
#include "../../../../../include/aon/tools/gui/ui/gui-layout.hpp"

namespace aon {

void Gui::displayMainMenu() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();

  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2, (BRAIN_SCREEN_HEIGHT - 225) / 4);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(pros::Color::white); // Default color for "NO AUTON"
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::set_pen(pros::Color::green);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw the "AUTONS" button using UI helper
  AutonsBtn.draw(pros::E_TEXT_LARGE);
}

void Gui::displayAutonMenu() {
  // Main AUTONS hub: shows current selection and navigates to Red/Blue/Skills submenus
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
  aon::drawAutonSelections();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(pros::Color::white);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::set_pen(pros::Color::green);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw navigation buttons using UI helpers
  backBtnGray.draw();
  blueBtn.draw(pros::E_TEXT_LARGE);
  redBtn.draw(pros::E_TEXT_LARGE);
  skillsBtn.draw(pros::E_TEXT_LARGE);
}

void Gui::displayRedAutonMenu() {
  // Red-side autons list with three option buttons
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();

  // Set background to red
  pros::screen::set_eraser(pros::Color::red);
  pros::screen::erase();

  // Draw BACK button
  backBtnRed.draw();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(pros::Color::white);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "RED" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "RED");

  // Draw auton selection buttons with red theme colors
  ui::Button aut1 = aut1Btn; aut1.bg = pros::Color::light_pink; aut1.label = redAutonOptions[0].buttonLabel;
  ui::Button aut2 = Aut2Btn; aut2.bg = pros::Color::crimson; aut2.label = redAutonOptions[1].buttonLabel;
  ui::Button aut3 = Aut3Btn; aut3.bg = pros::Color::red; aut3.label = redAutonOptions[2].buttonLabel;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

void Gui::displayBlueAutonMenu() {
  // Blue-side autons list
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();

  // Set background to blue
  pros::screen::set_eraser(pros::Color::blue);
  pros::screen::erase();

  // Draw BACK button
  BackBtnBlue.draw();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(pros::Color::white);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "BLUE" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "BLUE");

  // Draw auton selection buttons with blue theme colors
  ui::Button aut1 = aut1Btn; aut1.bg = pros::Color::sky_blue; aut1.label = blueAutonOptions[0].buttonLabel;
  ui::Button aut2 = Aut2Btn; aut2.bg = pros::Color::steel_blue; aut2.label = blueAutonOptions[1].buttonLabel;
  ui::Button aut3 = Aut3Btn; aut3.bg = pros::Color::blue; aut3.label = blueAutonOptions[2].buttonLabel;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

void Gui::displaySkillsMenu() {
  // Skills autons list
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();

  // Set background to green
  pros::screen::set_eraser(pros::Color::green);
  pros::screen::erase();

  // Add a delay to allow the screen to load
  pros::delay(300);

  // Draw BACK button
  BackBtnGreen.draw();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(pros::Color::white);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "SKILLS" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "SKILLS");

  // Draw auton selection buttons with green theme colors
  ui::Button aut1 = aut1Btn; aut1.bg = pros::Color::light_green; aut1.label = skillsAutonOptions[0].buttonLabel;
  ui::Button aut2 = Aut2Btn; aut2.bg = pros::Color::yellow_green; aut2.label = skillsAutonOptions[1].buttonLabel;
  ui::Button aut3 = Aut3Btn; aut3.bg = pros::Color::green; aut3.label = skillsAutonOptions[2].buttonLabel;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

}  // namespace aon
