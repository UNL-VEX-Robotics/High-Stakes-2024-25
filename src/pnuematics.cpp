#include "pnuematics.h"

/**
 * Constructor method for the pnuematic class
 * 
 * @param threeWirePort
 */
pnuematic::pnuematic(vex::triport::port threeWirePort) :
this_pnuematic(threeWirePort)
{
}

/**
 * Toggles the pnuematics state
 */
void pnuematic::toggle()
{
    this_pnuematic.set(!this_pnuematic.value());
}

/**
 * Sets the pneumatic to a specific value
 * 
 * @param value the new value of the pneumatic
 */
void pnuematic::set(bool value)
{
    this_pnuematic.set(value);
}

/**
 * Gets the value of the pnuematic
 * 
 * @return the value
 */
bool pnuematic::getValue()
{
    return this_pnuematic.value();
}
