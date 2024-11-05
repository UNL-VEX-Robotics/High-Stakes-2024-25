#pragma once
#include "vex.h"

class pnuematic{
private:
    vex::pneumatics this_pnuematic;
public:
    pnuematic(vex::triport::port threeWirePort);
    void toggle();
    void set(bool value);
    
    bool getValue();
};