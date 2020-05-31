
#include "TestClass.h"

using namespace project_namespace;

TestClass::TestClass(
  uint8_t test_var_init
):
  // INITIALISATION LIST
  test_var(test_var_init)
{}

uint8_t TestClass::getTestVar(){
  return test_var;
}

uint8_t TestClass::returnSomeNumber() {
  return 10;
}
