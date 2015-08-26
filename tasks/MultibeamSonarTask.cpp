/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MultibeamSonarTask.hpp"

using namespace imaging_sonar_simulation;

MultibeamSonarTask::MultibeamSonarTask(std::string const& name) :
		MultibeamSonarTaskBase(name) {
}

MultibeamSonarTask::MultibeamSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		MultibeamSonarTaskBase(name, engine) {
}

MultibeamSonarTask::~MultibeamSonarTask() {
}

bool MultibeamSonarTask::setRange(double value) {
	if (value < _multibeam_sonar.min_range)
		value = _multibeam_sonar.min_range;

	else if (value > _multibeam_sonar.max_range)
		value = _multibeam_sonar.max_range;

	_normal_depth_map.setMaxRange(value);
	_multibeam_sonar.setRange(value);

	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setRange(value));
}

bool MultibeamSonarTask::setNumber_of_bins(int value) {
	_multibeam_sonar.setNumberOfBins(value);

	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setNumber_of_bins(value));
}

bool MultibeamSonarTask::setNumber_of_beams(int value) {
	_multibeam_sonar.setNumberOfBeams(value);

	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setNumber_of_beams(value));
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
	float fovX = _multibeam_sonar.getBeamwidthHorizontal();
	float fovY = _multibeam_sonar.getBeamwidthVertical();
	uint width = _multibeam_sonar.getNumberOfBeams() * _multibeam_sonar.getPixelsPerBeam();
	float range = _multibeam_sonar.getRange();

	Task::init(fovX, fovY, width, range, false);

	return true;
}

void MultibeamSonarTask::updateMultibeamSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receive shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// split shader image to beams
	std::vector<cv::Mat> cv_beams = _multibeam_sonar.splitShaderImage(cv_image);

	// decode shader image and get ping data
	std::vector<cv::Mat> raw_intensity;
	std::vector<uint8_t> sonar_data;

	for (int i = 0; i < _multibeam_sonar.getNumberOfBeams(); i++) {
		raw_intensity.push_back(_multibeam_sonar.decodeShaderImage(cv_beams[i]));
		std::vector<uint8_t> current_data = _multibeam_sonar.getPingData(raw_intensity[i]);
		sonar_data.insert(sonar_data.end(), current_data.begin(), current_data.end());
	}

	// write in beam_samples port
	base::samples::SonarScan packet = _multibeam_sonar.simulateSonarScan(sonar_data);
	_sonar_samples.write(packet);
}

void MultibeamSonarTask::updateHook() {
	MultibeamSonarTaskBase::updateHook();

	base::samples::RigidBodyState linkPose;
	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData) {
		updateMultibeamSonarPose(linkPose);
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



