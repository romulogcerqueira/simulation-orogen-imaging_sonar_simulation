#ifndef IMAGING_SONAR_SIMULATION_TYPES_HPP
#define IMAGING_SONAR_SIMULATION_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace imaging_sonar_simulation {

    namespace orientation {

        // Multibeam Sonar Orientation
        enum Type {
            Horizontal = 0,
            Vertical = 1
        };

    }
}
#endif
