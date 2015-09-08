///* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"
#include <base/samples/Frame.hpp>
#include <frame_helper/FrameHelper.h>

using namespace imaging_sonar_simulation;

ScanningSonarTask::ScanningSonarTask(std::string const& name) :
		ScanningSonarTaskBase(name) {
}

ScanningSonarTask::ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		ScanningSonarTaskBase(name, engine) {
}

ScanningSonarTask::~ScanningSonarTask() {
}

bool ScanningSonarTask::setPing_pong_mode(bool value) {
	_ssonar.setPingPongMode(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setPing_pong_mode(value));
}

bool ScanningSonarTask::setRange(double value) {
	_normal_depth_map.setMaxRange(value);
	_ssonar.setRange(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setRange(value));
}

bool ScanningSonarTask::setGain(int value) {
	_ssonar.setGain(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setGain(value));
}

bool ScanningSonarTask::setStart_angle(double value) {

	_ssonar.setStartAngle(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setStart_angle(value));
}

bool ScanningSonarTask::setEnd_angle(double value) {

	_ssonar.setEndAngle(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setEnd_angle(value));
}

bool ScanningSonarTask::setStep_angle(double value) {

	_ssonar.setStepAngle(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setStep_angle(value));
}

bool ScanningSonarTask::setNumber_of_bins(int value) {
	_ssonar.setNumberOfBins(value);

	return (imaging_sonar_simulation::ScanningSonarTaskBase::setNumber_of_bins(value));
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ScanningSonarTask.hpp for more detailed
// documentation about them.

bool ScanningSonarTask::configureHook() {
	if (!ScanningSonarTaskBase::configureHook())
		return false;
	return true;
}

bool ScanningSonarTask::startHook() {
	if (!ScanningSonarTaskBase::startHook())
		return false;

	// set shader parameters
	float fovX = _ssonar.getBeamwidthHorizontal();
	float fovY = _ssonar.getBeamwidthVertical();
	int height = 500;
	float range = _ssonar.getRange();

	// generate shader world
	Task::init(fovX, fovY, height, range, true);
	_rotZ = 0.0f;

	return true;
}

void ScanningSonarTask::updateScanningSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receive shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// decode shader image
	cv::Mat raw_intensity = _ssonar.decodeShaderImage(cv_image);

	// get ping data
	std::vector<uint8_t> sonar_data = _ssonar.getPingData(raw_intensity);

	// simulate sonar data
	base::samples::SonarBeam sonar_beam = _ssonar.simulateSonarBeam(sonar_data);

	// display sonar viewer
	base::samples::frame::Frame frame;
	_ssonar.plotSonarData(sonar_beam, _ssonar.getRange(), _ssonar.getGain());
	frame_helper::FrameHelper::copyMatToFrame(_ssonar.getViewer(), frame);
	_sonar_viewer.write(frame);

	// display shader image
	cv::Mat cv_shader;
	cv_image.convertTo(cv_shader, CV_8UC3, 255);
	frame_helper::FrameHelper::copyMatToFrame(cv_shader, frame);
	_shader_viewer.write(frame);
}

base::samples::RigidBodyState ScanningSonarTask::rotatePose(base::samples::RigidBodyState pose) {

	base::samples::RigidBodyState new_pose;
	new_pose.position = pose.position;
	new_pose.orientation = pose.orientation * Eigen::AngleAxisd(_rotZ * M_PI / 180.0, Eigen::Vector3d::UnitZ());
	return new_pose;

}

void ScanningSonarTask::updateHook() {

	ScanningSonarTaskBase::updateHook();

	base::samples::RigidBodyState linkPose;

	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData) {

		base::samples::RigidBodyState scanningSonarPose = rotatePose(linkPose);
		updateScanningSonarPose(scanningSonarPose);
	}

	if (_ssonar.isReverseScan()) {
		_rotZ += _ssonar.getStepAngle();
		if (_rotZ >= _ssonar.getEndAngle())
			_rotZ = _ssonar.getStartAngle();
	} else {
		_rotZ -= _ssonar.getStepAngle();
		if (_rotZ <= _ssonar.getStartAngle())
			_rotZ = _ssonar.getEndAngle();
	}

}

void ScanningSonarTask::errorHook() {
	ScanningSonarTaskBase::errorHook();
}

void ScanningSonarTask::stopHook() {
	ScanningSonarTaskBase::stopHook();
}

void ScanningSonarTask::cleanupHook() {
	ScanningSonarTaskBase::cleanupHook();
}
