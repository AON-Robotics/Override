#include "aon/tools/gui/auton-selection.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

int main() {
  CHECK(aon::clampAutonIndex(3, 3) == 3);
  CHECK(aon::clampAutonIndex(4, 3) == 3);
  CHECK(aon::clampAutonIndex(5, 3) == 3);
  CHECK(aon::clampAutonIndex(0, 3) == 1);
  std::cout << "AON auton selection tests passed\n";
}
