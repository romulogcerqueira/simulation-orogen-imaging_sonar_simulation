/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MultibeamSonarTask.hpp"

using namespace imaging_sonar_simulation;

MultibeamSonarTask::MultibeamSonarTask(std::string const& name) :
		MultibeamSonarTaskBase(name) {
	_beam_width.set(base::Angle::fromDeg(120.0));
	_beam_height.set(base::Angle::fromDeg(20.0));
}

MultibeamSonarTask::MultibeamSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		MultibeamSonarTaskBase(name, engine) {
	_beam_width.set(base::Angle::fromDeg(120.0));
	_beam_height.set(base::Angle::fromDeg(20.0));
}

MultibeamSonarTask::~MultibeamSonarTask() {
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MultibeamSonarTask.hpp for more detailed
// documentation about them.

bool MultibeamSonarTask::configureHook() {
    if (!MultibeamSonarTaskBase::configureHook())
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
    // check if the properties have valid values
    if (_beam_count.value() < 64 || _beam_count.value() > 512) {
        RTT::log(RTT::Error) << "The number of beams must be between 64 and 512." << RTT::endlog();
        return false;
    }
     
    osg::ref_ptr<osg::Group> root = vizkit3dWorld->getWidget()->getRootNode();
    // set the attributes
    // generate shader world
    uint width =  _bin_count.value() * 5.12;  // 5.12 pixels are needed for each bin
    sonar_sim =  new gpu_sonar_simulation::SonarSimulation(_range.value(), _gain.value(), _bin_count.value(),
            _beam_count.value(),_beam_width.value(), _beam_height.value(), 
            width, false, root);

    return true;
}

bool MultibeamSonarTask::startHook() {
	if (!MultibeamSonarTaskBase::startHook())
		return false;

        return true;
}

void MultibeamSonarTask::updateHook() {
	MultibeamSonarTaskBase::updateHook();

    base::samples::RigidBodyState link_pose;

    if (_sonar_pose_cmd.read(link_pose) == RTT::NewData) {

        base::samples::Sonar sonar = sonar_sim->simulateSonarData(link_pose.getTransform());

        // set the sonar bearings
        base::Angle interval = base::Angle::fromRad(sonar_sim->getSonarBeamWidth().getRad() / sonar_sim->getSonarBeamCount());
        base::Angle start = base::Angle::fromRad(-sonar_sim->getSonarBeamWidth().getRad() / 2);
        sonar.setRegularBeamBearings(start, interval);

        // write sonar sample in the output port
        sonar.validate();
        _sonar_samples.write(sonar);
        
        //display the shader image
        std::auto_ptr<base::samples::frame::Frame> frame(new base::samples::frame::Frame());
        *frame = sonar_sim->getLastFrame();
        frame->time = base::Time::now();
        _shader_viewer.write(RTT::extras::ReadOnlyPointer<base::samples::frame::Frame>(frame.release()));
    }
}

void MultibeamSonarTask::errorHook() {
	MultibeamSonarTaskBase::errorHook();
}

void MultibeamSonarTask::stopHook() {
	MultibeamSonarTaskBase::stopHook();
}

void MultibeamSonarTask::cleanupHook() {
	MultibeamSonarTaskBase::cleanupHook();
}

bool MultibeamSonarTask::setBin_count(int value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive." << RTT::endlog();
        return false;
    }
    sonar_sim->setSonarBinCount(value);
    float width = sonar_sim->getSonarBinCount()* 5.12;  // 5.12 pixels are needed for each bin
    sonar_sim->setupShader(width, false);
    return MultibeamSonarTaskBase::setBin_count(value);
}

bool MultibeamSonarTask::setBeam_count(int value) {
    if (value < 64 || value > 512) {
        RTT::log(RTT::Error) << "The number of beams must be between 64 and 512." << RTT::endlog();
        return false;
    }
    sonar_sim->setSonarBeamCount(value);
    return (MultibeamSonarTaskBase::setBeam_count(value));
}

bool MultibeamSonarTask::setRange(double value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }
    sonar_sim->setRange(value);
    return true;
}

bool MultibeamSonarTask::setGain(double value) {
    if (value < 0 || value > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    sonar_sim->setGain(value);
    return true;
}
