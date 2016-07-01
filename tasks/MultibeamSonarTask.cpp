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
    if (_beam_count.value() < 64 || _beam_count.value() > 512) {
        RTT::log(RTT::Error) << "The number of beams must be between 64 and 512." << RTT::endlog();
        return false;
    }

    // set the attributes
    sonar_sim.beam_count = _beam_count.value();

    return true;
}

bool MultibeamSonarTask::startHook() {
	if (!MultibeamSonarTaskBase::startHook())
		return false;

    // generate shader world
    uint width = 1536;
    Task::initShader(width, false);

    return true;
}

void MultibeamSonarTask::updateHook() {
	MultibeamSonarTaskBase::updateHook();

    base::samples::RigidBodyState link_pose;

    if (_sonar_pose_cmd.read(link_pose) == RTT::NewData) {
        // update sonar position
        Task::updateSonarPose(link_pose);

        // receives the shader image
        osg::ref_ptr<osg::Image> osg_image = capture.grabImage(normal_depth_map.getNormalDepthMapNode());

        // process the shader image
        std::vector<float> bins;
        Task::processShader(osg_image, bins);

        // simulate sonar reading
        base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, range);

        // set the sonar bearings
        base::Angle interval = base::Angle::fromRad(sonar_sim.beam_width.getRad() / sonar_sim.beam_count);
        base::Angle start = base::Angle::fromRad(-sonar_sim.beam_width.getRad() / 2);
        sonar.setRegularBeamBearings(start, interval);

        // write sonar sample in the output port
        sonar.validate();
        _sonar_samples.write(sonar);
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
