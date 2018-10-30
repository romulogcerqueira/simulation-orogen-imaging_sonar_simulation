#ifndef imaging_sonar_simulation_TYPES_HPP
#define imaging_sonar_simulation_TYPES_HPP

#include <string>
#include <vector>
#include <base/Temperature.hpp>

namespace imaging_sonar_simulation {

// Properties used to calculate the sound attenuation underwater
struct AcousticAttenuationProperties {
    double frequency;               // in kHz
    base::Temperature temperature;  // in Kelvin
    double salinity;                // in ppt
    double acidity;                 // in pH

    AcousticAttenuationProperties()
        : frequency(300.0)
        , temperature(base::Temperature::fromCelsius(25.0))
        , salinity(0)
        , acidity(8) {};
};

}

#endif
