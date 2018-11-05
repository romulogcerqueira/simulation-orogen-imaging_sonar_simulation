/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

// Rock includes
#include <gpu_sonar_simulation/Utils.hpp>
#include <frame_helper/FrameHelper.h>
#include <base/Float.hpp>

using namespace imaging_sonar_simulation;
using namespace base::samples::frame;

Task::Task(std::string const& name) :
		TaskBase(name) {
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) :
		TaskBase(name, engine) {
}

Task::~Task() {
}

bool Task::configureHook() {
    if (!TaskBase::configureHook())
        return false;

    // check if the properties have valid values
    if (_range.value() <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

    if (_gain.value() < 0 || _gain.value() > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    if (_bin_count.value() <= 0 || _bin_count.value() > 1500) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less than 1500." << RTT::endlog();
        return false;
    }

    if (_beam_width.value().getRad() <= 0 || _beam_height.value().getRad() <= 0) {
        RTT::log(RTT::Error) << "The sonar opening angles must be positives." << RTT::endlog();
        return false;
    }

    if (_attenuation_properties.value().frequency < 0.1
        || _attenuation_properties.value().frequency > 1000) {
        RTT::log(RTT::Error) << "The sonar frequency must be between 100 Hz and 1 MHz." << RTT::endlog();
        return false;
    }

    if(_attenuation_properties.value().temperature.getCelsius() < -6
        || _attenuation_properties.value().temperature.getCelsius() > 35
        || base::isNaN(_attenuation_properties.value().temperature.getCelsius())) {
        RTT::log(RTT::Error) << "The water temperature value must be between -6 and 35 Celsius degrees." << RTT::endlog();
        return false;
    }

    if (_attenuation_properties.value().salinity < 0
        || _attenuation_properties.value().salinity > 50) {
        RTT::log(RTT::Error) << "The water salinity must be between 0 (only consider freshwater contribution) and 50 ppt." << RTT::endlog();
        return false;
    }

    if (_attenuation_properties.value().acidity < 7.7
        || _attenuation_properties.value().acidity > 8.3) {
        RTT::log(RTT::Error) << "The water acidity must have pH between 7.7 and 8.3." << RTT::endlog();
        return false;
    }
    // set the attributes
    attenuation_properties = _attenuation_properties.value();

	return true;
}

bool Task::configureSonarSimulation(bool isScanning)
{
    osg::ref_ptr<osg::Group> root = vizkit3dWorld->getWidget()->getRootNode();
    // generate shader world
    int value = _bin_count.value() * 5.12;    // 5.12 pixels are needed for each bin
    sonar_sim.init(_range.value(), _gain.value(), _bin_count.value(),
            _beam_width.value(), _beam_height.value(), value, isScanning, root);
    // set the attributes

	return true;
}

bool Task::startHook() {
	if (!TaskBase::startHook())
		return false;
	return true;
}
void Task::updateHook() {
	TaskBase::updateHook();
}
void Task::errorHook() {
	TaskBase::errorHook();
}
void Task::stopHook() {
	TaskBase::stopHook();
}
void Task::cleanupHook() {
	TaskBase::cleanupHook();
}

bool Task::setRange(double value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }
    sonar_sim.setRange(value);
    return (imaging_sonar_simulation::TaskBase::setRange(value));
}

bool Task::setGain(double value) {
    if (value < 0 || value > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }
    sonar_sim.setGain(value);
    return (imaging_sonar_simulation::TaskBase::setGain(value));
}

bool Task::setAttenuation_properties(::imaging_sonar_simulation::AcousticAttenuationProperties const & value) {
    if (value.frequency < 0.1 || value.frequency > 1000) {
        RTT::log(RTT::Error) << "The sonar frequency must be between 100 Hz and 1 MHz." << RTT::endlog();
        return false;
    }

    if (value.temperature.getCelsius() < -6 || value.temperature.getCelsius() > 35 || base::isNaN(value.temperature.getCelsius())) {
        RTT::log(RTT::Error) << "The water temperature value must be between -6 and 35 Celsius degrees." << RTT::endlog();
        return false;
    }

    if (value.salinity < 0 || value.salinity > 50) {
        RTT::log(RTT::Error) << "The water salinity must be between 0 (only consider freshwater contribution) and 50 ppt." << RTT::endlog();
        return false;
    }

    if (value.acidity < 7.7 || value.acidity > 8.3) {
        RTT::log(RTT::Error) << "The water acidity must have pH between 7.7 and 8.3." << RTT::endlog();
        return false;
    }

    attenuation_properties = value;
    return (imaging_sonar_simulation::TaskBase::setAttenuation_properties(value));
}

bool Task::setBin_count(int value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive." << RTT::endlog();
        return false;
    }
    sonar_sim.setSonarBinCount(value);
    float width = sonar_sim.getSonarBinCount()* 5.12;  // 5.12 pixels are needed for each bin
    sonar_sim.setupShader(width, false);
    return TaskBase::setBin_count(value);
}