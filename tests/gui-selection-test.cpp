// Compile the production GUI selection and indexing; replace only display IO.
#define AON_TOOLS_GUI_IMAGE_GENERATOR_GUI_IMAGES_HPP_
#define AON_TOOLS_GUI_DEBUG_HPP_
#include "aon/tools/gui/gui.hpp"
volatile Alliance ALLIANCE = Alliance::Red;
namespace aon {
using GuiDebug = Gui;
void drawDebugCleaner() {}
void Gui::displayMainMenu() {}
void Gui::displayAutonMenu() {}
void Gui::displayRedAutonMenu() {}
void Gui::displayBlueAutonMenu() {}
void Gui::displaySkillsMenu() {}
namespace routines {
int RedRoutine1() { return 11; } int RedRoutine2() { return 12; } int RedRoutine3() { return 13; }
int BlueRoutine1() { return 21; } int BlueRoutine2() { return 22; } int BlueRoutine3() { return 23; }
int SkillsRoutine1() { return 31; } int SkillsRoutine2() { return 32; } int SkillsRoutine3() { return 33; }
int BasicUTurnRoutine() { return 3; } int StaticPathRoutine() { return 4; }
}
}
#include "../src/aon/tools/gui/gui.cpp"

void testSelection() {
  aon::Gui selected;
  assert(selected.invokeSelectedAuton() == 0);
  const int expected[][4] = {{11,12,3,4},{21,22,3,4},{31,32,33,33}};
  const Alliance alliances[] = {Alliance::Red, Alliance::Blue, Alliance::Skills};
  for (int a=0; a<3; ++a) {
    for (int index : {INT32_MIN, -1, 0, 1, 2, 3, 4, 5, INT32_MAX}) {
      selected.selectAutonByList(alliances[a],index);
      const int bounded = index < 1 ? 1 : index > 4 ? 4 : index;
      assert(selected.invokeSelectedAuton() == expected[a][bounded-1]);
    }
  }
  selected.selectAutonByList(Alliance::Red, -100);
  assert(selected.selectedRedAut == 1 && selected.selectedBlueAut == 0 && selected.selectedSkill == 0);
  assert(selected.invokeSelectedAuton() == 11);
  selected.selectAutonByList(Alliance::Blue, 100);
  assert(selected.selectedRedAut == 0 && selected.selectedBlueAut == 4 && selected.selectedSkill == 0);
  assert(selected.invokeSelectedAuton() == 4);
  selected.selectAutonByList(Alliance::Skills, 100);
  assert(selected.selectedRedAut == 0 && selected.selectedBlueAut == 0 && selected.selectedSkill == 3);
  assert(selected.invokeSelectedAuton() == 33);
  selected.selectAutonByList(static_cast<Alliance>(99), 1);
  assert(selected.invokeSelectedAuton() == 33 && ALLIANCE == Alliance::Skills);
}
