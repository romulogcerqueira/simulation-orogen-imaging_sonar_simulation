///* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"
#include <frame_helper/FrameHelper.h>

using namespace imaging_sonar_simulation;
using namespace base::samples::frame;

ScanningSonarTask::ScanningSonarTask(std::string const& name) :
		ScanningSonarTaskBase(name) {
	_start_angle.set(base::Angle::fromRad(-M_PI));
	_end_angle.set(base::Angle::fromRad(M_PI));
	_step_angle.set(base::Angle::fromRad(1.8 * M_PI / 180.0));
	_beam_width.set(base::Angle::fromRad(3.0 * M_PI / 180.0));
	_beam_height.set(base::Angle::fromRad(35.0 * M_PI / 180.0));
}

ScanningSonarTask::ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		ScanningSonarTaskBase(name, engine) {
	_start_angle.set(base::Angle::fromRad(-M_PI));
	_end_angle.set(base::Angle::fromRad(M_PI));
	_step_angle.set(base::Angle::fromRad(1.8 * M_PI / 180.0));
	_beam_width.set(base::Angle::fromRad(3.0 * M_PI / 180.0));
	_beam_height.set(base::Angle::fromRad(35.0 * M_PI / 180.0));
}

ScanningSonarTask::~ScanningSonarTask() {
}

bool ScanningSonarTask::setPing_pong_mode(bool value) {
	_ssonar.setPingPongMode(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setPing_pong_mode(value));
}

bool ScanningSonarTask::setRange(double value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

	_normal_depth_map.setMaxRange(value);
	_ssonar.setRange(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setRange(value));
}

bool ScanningSonarTask::setGain(double value) {
    if (value < 0 || value > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    _ssonar.setGain(value);
    return (imaging_sonar_simulation::ScanningSonarTaskBase::setGain(value));
}

bool ScanningSonarTask::setStart_angle(base::Angle value) {
	_ssonar.setStartAngle(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setStart_angle(value));
}

bool ScanningSonarTask::setEnd_angle(base::Angle value) {
	_ssonar.setEndAngle(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setEnd_angle(value));
}

bool ScanningSonarTask::setStep_angle(base::Angle value) {
    if (value.getRad() <= 0 || value.getRad() > (3.6 * M_PI / 180.0)) {
        RTT::log(RTT::Error) << "The step angle value must be positive and less or equal than 3.6 degrees." << RTT::endlog();
        return false;
    }

	_ssonar.setStepAngle(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setStep_angle(value));
}

bool ScanningSonarTask::setBin_count(int value) {
    if (value <= 0 || value > 1500) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less or equal than 1500." << RTT::endlog();
        return false;
    }

	_ssonar.setBinCount(value);
	return (imaging_sonar_simulation::ScanningSonarTaskBase::setBin_count(value));
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ScanningSonarTask.hpp for more detailed
// documentation about them.

bool ScanningSonarTask::configureHook() {
	if (!ScanningSonarTaskBase::configureHook())
		return false;

    _ssonar.setRange(_range.value());
    _normal_depth_map.setMaxRange(_range.value());
    _ssonar.setGain(_gain.value());
    _ssonar.setBinCount(_bin_count.value());
    _ssonar.setPingPongMode(_ping_pong_mode.value());
    _ssonar.setStartAngle(_start_angle.value());
    _ssonar.setEndAngle(_end_angle.value());
    _ssonar.setStepAngle(_step_angle.value());
    _ssonar.setBeamWidth(_beam_width.value());
    _ssonar.setBeamHeight(_beam_height.value());

    if (_ssonar.getRange() <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

    if (_ssonar.getGain() < 0 || _ssonar.getGain() > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    if (_ssonar.getBinCount() <= 0 || _ssonar.getBinCount() > 1500) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less or equal than 1500." << RTT::endlog();
        return false;
    }

    if (_ssonar.getStepAngle().getRad() <= 0 || _ssonar.getStepAngle().getRad() > (3.6 * M_PI / 180.0)) {
        RTT::log(RTT::Error) << "The step angle value must be positive and less or equal than 3.6 degrees." << RTT::endlog();
        return false;
    }

    if (_ssonar.getBeamHeight().getRad() <= 0 || _ssonar.getBeamWidth().getRad() <= 0) {
        RTT::log(RTT::Error) << "The sonar opening angles must be positives." << RTT::endlog();
        return false;
    }
	return true;
}

bool ScanningSonarTask::startHook() {
	if (!ScanningSonarTaskBase::startHook())
		return false;

	// generate shader world
	int height = 500;
	Task::init(_ssonar.getBeamWidth(), _ssonar.getBeamHeight(), height, _ssonar.getRange(), true);
	_rotZ = 0.0;

	return true;
}

void ScanningSonarTask::updateScanningSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receive shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// decode shader image
	std::vector<float> sonar_data = _ssonar.decodeShaderImage(cv_image);

	// apply the "gain" (in this case, it is a light intensity change)
	float gain_factor = _ssonar.getGain() / 0.5;
	std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), std::bind1st(std::multiplies<float>(), gain_factor));
	std::replace_if(sonar_data.begin(), sonar_data.end(), bind2nd(greater<float>(), 1.0), 1.0);

	// simulate sonar data
	base::samples::Sonar sonar = _ssonar.simulateSingleBeam(sonar_data);
	_sonar_samples.write(sonar);

	// display shader image
	std::auto_ptr<Frame> frame(new Frame());
	cv::Mat cv_shader;
	cv_image.convertTo(cv_shader, CV_8UC3, 255);
	frame_helper::FrameHelper::copyMatToFrame(cv_shader, *frame.get());
	_shader_viewer.write(RTT::extras::ReadOnlyPointer<Frame>(frame.release()));

    // rotate sonar
    _rotZ = _ssonar.getBearing().getRad();
    _ssonar.moveHeadPosition();
}

base::samples::RigidBodyState ScanningSonarTask::rotatePose(base::samples::RigidBodyState pose) {

	base::samples::RigidBodyState new_pose;
	new_pose.position = pose.position;
	new_pose.orientation = pose.orientation * Eigen::AngleAxisd(_rotZ, Eigen::Vector3d::UnitZ());
	return new_pose;
}

void ScanningSonarTask::updateHook() {

	ScanningSonarTaskBase::updateHook();

	base::samples::RigidBodyState linkPose;

	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData) {
		base::samples::RigidBodyState scanningSonarPose = rotatePose(linkPose);
		updateScanningSonarPose(scanningSonarPose);
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
