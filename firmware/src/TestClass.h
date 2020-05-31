
#ifndef MAIN_PROGRAMME_H
#define MAIN_PROGRAMME_H

#include <Arduino.h>

namespace project_namespace {

  class TestClass {
    private:
      uint8_t test_var;
    protected:
    public:
      // CONSTRUCTORS
      TestClass(
        uint8_t test_var_init
      );
      // GETTERS
      uint8_t getTestVar();
      // Misc.
      static uint8_t returnSomeNumber();

  };
}
#endif
