/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MultibeamSonarTask.hpp"
#include <frame_helper/FrameHelper.h>

using namespace imaging_sonar_simulation;
using namespace base::samples::frame;

MultibeamSonarTask::MultibeamSonarTask(std::string const& name) :
		MultibeamSonarTaskBase(name) {
}

MultibeamSonarTask::MultibeamSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		MultibeamSonarTaskBase(name, engine) {
}

MultibeamSonarTask::~MultibeamSonarTask() {
}

bool MultibeamSonarTask::setRange(double value) {
	_normal_depth_map.setMaxRange(value);
	_msonar.setRange(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setRange(value));
}

bool MultibeamSonarTask::setGain(double value) {
    _msonar.setGain(value);
    return (imaging_sonar_simulation::MultibeamSonarTaskBase::setGain(value));
}

bool MultibeamSonarTask::setNumber_of_bins(int value) {
	_msonar.setNumberOfBins(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setNumber_of_bins(value));
}

bool MultibeamSonarTask::setNumber_of_beams(int value) {
	_msonar.setNumberOfBeams(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setNumber_of_beams(value));
}

bool MultibeamSonarTask::setStart_bearing(double value) {
	_msonar.setStartBearing(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setStart_bearing(value));
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MultibeamSonarTask.hpp for more detailed
// documentation about them.

bool MultibeamSonarTask::configureHook() {
	if (!MultibeamSonarTaskBase::configureHook())
		return false;
	return true;
}

bool MultibeamSonarTask::startHook() {
	if (!MultibeamSonarTaskBase::startHook())
		return false;

	// set shader image parameters
	float fovX = _msonar.getBeamwidthHorizontal();
	float fovY = _msonar.getBeamwidthVertical();
	uint width = _msonar.getNumberOfBeams() * _msonar.getPixelsPerBeam();
	float range = _msonar.getRange();

	Task::init(fovX, fovY, width, range, false);

	return true;
}

void MultibeamSonarTask::updateHook() {
	MultibeamSonarTaskBase::updateHook();

	base::samples::RigidBodyState linkPose;

	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData)
		updateMultibeamSonarPose(linkPose);
}

void MultibeamSonarTask::updateMultibeamSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receives shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// simulate sonar data
	std::vector<uint8_t> sonar_data = _msonar.codeSonarData(cv_image);

	// apply the "gain" (in this case, it is a light intensity change)
	double gain_factor = _msonar.getGain() / 0.5;
	std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), std::bind1st(std::multiplies<double>(), gain_factor));

	// simulate sonar data
	base::samples::SonarScan sonar_scan = _msonar.simulateSonarScan(sonar_data);
	base::samples::Sonar sonar(sonar_scan);

	// display sonar viewer
	_sonar_samples.write(sonar);

	// display shader image
	std::auto_ptr<Frame> frame(new Frame());
	cv::Mat cv_shader;
	cv_image.convertTo(cv_shader, CV_8UC3, 255);
	frame_helper::FrameHelper::copyMatToFrame(cv_shader, *frame.get());
	_shader_viewer.write(RTT::extras::ReadOnlyPointer<Frame>(frame.release()));
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
