/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MultibeamSonarTask.hpp"
#include <base/samples/Frame.hpp>
#include <frame_helper/FrameHelper.h>
#include <opencv2/contrib/contrib.hpp>


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
	_normal_depth_map.setMaxRange(value);
	_msonar.setRange(value);

	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setRange(value));
}

bool MultibeamSonarTask::setGain(int value) {
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

bool MultibeamSonarTask::setAngular_resolution(double value) {
	_msonar.setAngularResolution(value);

	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setAngular_resolution(value));
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

	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData) {

		base::samples::RigidBodyState multibeamSonarPose = linkPose;
		updateMultibeamSonarPose(multibeamSonarPose);
	}
}

void MultibeamSonarTask::updateMultibeamSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receives shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// simulate sonar data
	std::vector<uint8_t> sonar_data = _msonar.codeSonarData(cv_image);
	base::samples::SonarScan packet = _msonar.simulateSonarScan(sonar_data);

	// display sonar viewer
	base::samples::frame::Frame frame;
	_msonar.plotSonarData(packet, _msonar.getRange(), _msonar.getGain());
	frame_helper::FrameHelper::copyMatToFrame(_msonar.getViewer(), frame);
	_sonar_viewer.write(frame);

	// display shader image
	cv::Mat cv_shader;
	cv_image.convertTo(cv_shader, CV_8UC3, 255);
	frame_helper::FrameHelper::copyMatToFrame(cv_shader, frame);
	_shader_viewer.write(frame);
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
