#pragma once

namespace aon {

constexpr int clampAutonIndex(int index1Based, int optionCount) {
  return index1Based < 1 ? 1
                         : (index1Based > optionCount ? optionCount
                                                      : index1Based);
}

}  // namespace aon
