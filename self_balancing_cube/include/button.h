#ifndef BUTTON_H
#define BUTTON_H

#include "Standard_Imports.h"

namespace Button {
    extern const unsigned int BUTTON_PIN;
    extern bool on;

    void init();
    void buttonPressed();
    bool onState();
}

#endif