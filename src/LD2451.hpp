#ifndef __LD2451_hpp
#define __LD2451_hpp

#include "LD245X.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete sensor implementations                                      */
/* --------------------------------------------------------------------- */
class LD2451 : public LD245X {
public:
    LD2451();
    int readDataRadarOutput() override;
};

}

#endif
