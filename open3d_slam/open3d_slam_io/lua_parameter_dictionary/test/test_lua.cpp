/*
 * test_lua.cpp
 *
 *  Created on: Mar 26, 2021
 *      Author: jelavice
 */

// gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
