
#ifndef __LD2450_hpp
#define __LD2450_hpp

#include "LD245X.hpp"

namespace esphome::ld245x {

/* --------------------------------------------------------------------- */
/*  Concrete sensor implementations                                      */
/* --------------------------------------------------------------------- */
class LD2450 : public LD245X {
public:
    LD2450();
    int readDataRadarOutput() override;
};

}

#endif
