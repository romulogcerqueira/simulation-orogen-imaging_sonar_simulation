#ifndef imaging_sonar_simulation_TYPES_HPP
#define imaging_sonar_simulation_TYPES_HPP

#include <string>
#include <vector>
#include <base/Temperature.hpp>

namespace imaging_sonar_simulation {

// Properties used to calculate the sound attenuation underwater
struct AcousticAttenuationProperties {
    base::Temperature water_temperature;    // in Kelvin
    double sonar_frequency;                 // in kHz

    AcousticAttenuationProperties() {
        water_temperature = base::Temperature::fromCelsius(20.0);
        sonar_frequency = 300.0;
    }
};

}

#endif
