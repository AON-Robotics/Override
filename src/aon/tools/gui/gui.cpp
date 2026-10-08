#include "../../../../include/aon/tools/gui/gui.hpp"
#include "../../../../include/aon/tools/gui/gui-debug.hpp"
#include "../../../../include/aon/constants.hpp"
#include "../../../../include/aon/tools/gui/ui/gui-layout.hpp"
#include "../../../../include/aon/shadow.hpp"

namespace aon {

// Owning GUI instance — type selected at compile time by TESTING_AUTONOMOUS.
// A single std::unique_ptr<Gui> is used so no redundant reference alias is needed.
#if TESTING_AUTONOMOUS
using ShadowGuiBase = GuiDebug;
#else
using ShadowGuiBase = Gui;
#endif
// Extend the existing AUTONS hub in normal and debug builds. Only the existing
// GUI task draws Shadow; recording/storage stay in the competition control loop.
class ShadowGui final : public ShadowGuiBase {
 public:
  void displayMainMenu() override {
    ShadowGuiBase::displayMainMenu();
    if (shadow::selected()) {
      pros::screen::set_eraser(pros::Color::black);
      pros::screen::erase_rect(100, 0, 365, 36);
      pros::screen::set_pen(pros::Color::green);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "SHADOW AUTON");
    }
  }
  void displayAutonMenu() override {
    ShadowGuiBase::displayAutonMenu();
    if (shadow::selected()) {
      pros::screen::set_eraser(pros::Color::black);
      pros::screen::erase_rect(100, 0, 365, 36);
      pros::screen::set_pen(pros::Color::green);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "SHADOW AUTON");
    }
    pros::screen::set_eraser(pros::Color::dark_gray);
    pros::screen::erase_rect(370, 8, 475, 38);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, 385, 18, "SHADOW");
  }
  void handleAutonMenuTouch() override {
    const auto touch = pros::screen::touch_status();
    if ((touch.touch_status == pros::E_TOUCH_PRESSED || touch.touch_status == pros::E_TOUCH_HELD) &&
        touch.x >= 370 && touch.x <= 475 && touch.y >= 8 && touch.y <= 38) {
      shadow::openGui();
      while (shadow::guiTick()) pros::delay(100);
      displayAutonMenu();
      // Consume BACK release so it cannot also select a native submenu.
      while (pros::screen::touch_status().touch_status == pros::E_TOUCH_HELD ||
             pros::screen::touch_status().touch_status == pros::E_TOUCH_PRESSED) pros::delay(20);
      return;
    }
    ShadowGuiBase::handleAutonMenuTouch();
  }
};
std::unique_ptr<Gui> gui = std::make_unique<ShadowGui>();

// Define the autonomousReader unique_ptr
std::unique_ptr<FunctionReader<int>> autonomousReader =
    std::make_unique<FunctionReader<int>>();

// ============================================================================
// Helper Methods
// ============================================================================

int Gui::displayInitializationMessage() {
  // Clear the screen completely
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();

  // Typewriter-style primary message (shows debug text when enabled for
  // testing)
#if TESTING_AUTONOMOUS
  const char* msg = "Initializing Debug...";
#else
  const char* msg = "Initializing AON...";
#endif
  int len = 0;
  for (const char* p = msg; *p; ++p) ++len;

  // Estimate character dimensions (matches Button helpers)
  int charWidth = 18;  // pixels for large text
  int charHeight = 20;

  int textWidth = len * charWidth;
  int startX = (BRAIN_SCREEN_WIDTH - textWidth) / 2;
  int startY = BRAIN_SCREEN_HEIGHT / 3;  // higher on screen

  pros::screen::set_pen(pros::Color::white);
  for (int i = 1; i <= len; ++i) {
    // Clear the text area to avoid artifacts
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase_rect(startX, startY, startX + textWidth, startY + charHeight + 4);

    // Print substring (typewriter effect)
    char buf[128];
    int copy = (i < (int)sizeof(buf)) ? i : ((int)sizeof(buf) - 1);
    for (int j = 0; j < copy; ++j) buf[j] = msg[j];
    buf[copy] = '\0';
    pros::screen::print(pros::E_TEXT_LARGE, startX, startY, "%s", buf);

    pros::delay(60);
  }

  // Secondary message displayed lower and centered with typewriter effect
#if TESTING_AUTONOMOUS
  const char* secMsg = "Debug is ENABLED";
#else
  const char* secMsg = "AON is ON";
#endif
  int secLen = 0;
  for (const char* p = secMsg; *p; ++p) ++secLen;
  int secTextWidth = secLen * charWidth;
  int secX = (BRAIN_SCREEN_WIDTH - secTextWidth) / 2;
  int secY = startY + 40;
  pros::screen::set_pen(pros::Color::white);
  for (int i = 1; i <= secLen; ++i) {
    // Clear the secondary text area
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase_rect(secX, secY, secX + secTextWidth, secY + charHeight + 4);

    // Print substring
    char buf[64];
    int copy = (i < (int)sizeof(buf)) ? i : ((int)sizeof(buf) - 1);
    for (int j = 0; j < copy; ++j) buf[j] = secMsg[j];
    buf[copy] = '\0';
    pros::screen::print(pros::E_TEXT_LARGE, secX, secY, "%s", buf);

    pros::delay(60);
  }
  if (TESTING_AUTONOMOUS) {
    aon::drawDebugCleaner();
  } else {
    return 0;
  }

  return 0;
}

void Gui::applyPreselectedAuton() {
  // If user already directly set selectedAuton, don't override.
  if (selectedAuton.routine != nullptr && selectedAutonName != "None" &&
      (selectedRedAut == 0 && selectedBlueAut == 0 && selectedSkill == 0)) {
    return;  // Nothing to map; routine already chosen
  }

  if (selectedRedAut > 0) {
    selectAutonByList(Alliance::Red, selectedRedAut);
    return;  // Red takes precedence
  }

  if (selectedBlueAut > 0) {
    selectAutonByList(Alliance::Blue, selectedBlueAut);
    return;  // Blue next
  }

  if (selectedSkill > 0) {
    selectAutonByList(Alliance::Skills, selectedSkill);
  }
}

void Gui::selectAutonByList(Alliance alliance, int index1Based) {
  shadow::selectNative();
  if (index1Based < 1) index1Based = 1;
  if (index1Based > 3) index1Based = 3;

  const AutonOption* options = nullptr;
  switch (alliance) {
    case Alliance::Red:
      options = redAutonOptions;
      selectedRedAut = index1Based;
      selectedBlueAut = 0;
      selectedSkill = 0;
      break;
    case Alliance::Blue:
      options = blueAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = index1Based;
      selectedSkill = 0;
      break;
    case Alliance::Skills:
      options = skillsAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = 0;
      selectedSkill = index1Based;
      break;
  }
  ALLIANCE = alliance;
  const AutonOption& choice = options[index1Based - 1];
  selectedAuton = choice;
  selectedAutonName = choice.name;

  // Register the selected autonomous routine to the autonomousReader
  autonomousReader->AddFunction("autonomous", choice.routine);

  if (currentScreen == MainMenu) {
    displayMainMenu();
  }
}

// ============================================================================
// Touch Handlers
// ============================================================================

void Gui::handleMainMenuTouch(
    const pros::screen_touch_status_s_t& touchStatus) {
  // Check if AUTONS button is pressed
  if (AutonsBtn.isHit(touchStatus.x, touchStatus.y)) {
    if (currentScreen != AutonMenu) {
      displayAutonMenu();
      currentScreen = AutonMenu;
    }
  }
}

void Gui::handleAutonMenuTouch() {
  if (currentScreen != AutonMenu) return;

  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (backBtnGray.isHit(x, y)) {
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (blueBtn.isHit(x, y)) {
    displayBlueAutonMenu();
    currentScreen = BlueAutons;
  } else if (redBtn.isHit(x, y)) {
    displayRedAutonMenu();
    currentScreen = RedAutons;
  } else if (skillsBtn.isHit(x, y)) {
    displaySkillsMenu();
    currentScreen = SkillAutons;
  }
}

void Gui::handleRedAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (backBtnRed.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

void Gui::handleBlueAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnBlue.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

void Gui::handleSkillsMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnGreen.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

// ============================================================================
// GUI Loop
// ============================================================================

void Gui::mainLoop() {
  while (true) {
    pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status > 0) {
      switch (currentScreen) {
        case MainMenu:
          handleMainMenuTouch(TouchStatus);
          break;
        case AutonMenu:
          handleAutonMenuTouch();
          break;
        case RedAutons:
          handleRedAutonMenuTouch();
          break;
        case BlueAutons:
          handleBlueAutonMenuTouch();
          break;
        case SkillAutons:
          handleSkillsMenuTouch();
          break;
        default:
          break;
      }
    }

    pros::delay(100);
  }
}

// ============================================================================
// Initialization
// ============================================================================

void Gui::initialize() {
  std::cout << "Start" << std::endl;

  pros::delay(5);

  currentScreen = MainMenu;

  displayInitializationMessage();

  // Keep initialization message visible briefly before showing main menu
  pros::delay(1000);

  displayMainMenu();

  applyPreselectedAuton();

  this->mainLoop();
}

int Gui::invokeSelectedAuton() {
  if (selectedAuton.routine != nullptr) return selectedAuton.routine();
  return 0;
}

}  // namespace aon
