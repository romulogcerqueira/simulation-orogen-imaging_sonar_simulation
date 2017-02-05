///* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"
#include <frame_helper/FrameHelper.h>

using namespace imaging_sonar_simulation;
using namespace base::samples::frame;

ScanningSonarTask::ScanningSonarTask(std::string const& name) :
		ScanningSonarTaskBase(name) {
	_left_limit.set(base::Angle::Min());
	_right_limit.set(base::Angle::Max());
	_motor_step.set(base::Angle::fromDeg(1.8));
	_beam_width.set(base::Angle::fromDeg(3.0));
	_beam_height.set(base::Angle::fromDeg(35.0));
}

ScanningSonarTask::ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
		ScanningSonarTaskBase(name, engine) {
	_left_limit.set(base::Angle::Min());
	_right_limit.set(base::Angle::Max());
	_motor_step.set(base::Angle::fromDeg(1.8));
	_beam_width.set(base::Angle::fromDeg(3.0));
	_beam_height.set(base::Angle::fromDeg(35.0));
}

ScanningSonarTask::~ScanningSonarTask() {
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ScanningSonarTask.hpp for more detailed
// documentation about them.

bool ScanningSonarTask::configureHook() {
	if (!ScanningSonarTaskBase::configureHook())
		return false;

    // check if the properties have valid values
    if (_motor_step.value().getRad() <= 0 || _motor_step.value() > base::Angle::fromDeg(3.6)) {
        RTT::log(RTT::Error) << "The step angle value must be positive and less or equal than 3.6 degrees." << RTT::endlog();
        return false;
    }

    // set attributes
    left_limit = _left_limit.value();
    right_limit = _right_limit.value();
    motor_step = _motor_step.value();
    continuous = _continuous.value();
    sonar_sim.beam_count = 1;

    return true;
}

bool ScanningSonarTask::startHook() {
	if (!ScanningSonarTaskBase::startHook())
		return false;

    // generate shader world
    int height = sonar_sim.bin_count * 5.12;    // 5.12 pixels are needed for each bin
    Task::setupShader(height, true);

    current_bearing = base::Angle::fromRad(0.0);
    invert = false;

    return true;
}

void ScanningSonarTask::updateHook() {

	ScanningSonarTaskBase::updateHook();

    base::samples::RigidBodyState link_pose;

    if (_sonar_pose_cmd.read(link_pose) == RTT::NewData) {
        // update sonar position and orientation
        base::samples::RigidBodyState sonar_pose = rotatePose(link_pose);
        Task::updateSonarPose(sonar_pose);

        // receives the shader image
        osg::ref_ptr<osg::Image> osg_image = capture.grabImage(normal_depth_map.getNormalDepthMapNode());

        // process the shader image
        std::vector<float> bins;
        Task::processShader(osg_image, bins);

        // simulate sonar reading
        base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, range);

        // set the sonar bearing
        sonar.bearings.push_back(current_bearing);

        // write sonar sample in the output port
        sonar.validate();
        _sonar_samples.write(sonar);

        // move the head position
        moveHeadPosition();
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

base::samples::RigidBodyState ScanningSonarTask::rotatePose(base::samples::RigidBodyState pose) {
    base::samples::RigidBodyState new_pose;
    new_pose.position = pose.position;
    new_pose.orientation = pose.orientation * Eigen::AngleAxisd(current_bearing.getRad(), Eigen::Vector3d::UnitZ());
    return new_pose;
}

void ScanningSonarTask::moveHeadPosition() {
    if (continuous)
        current_bearing += motor_step;

    else {
        // clockwise reading
        if (!invert) {
            if (left_limit <= right_limit && current_bearing >= right_limit && current_bearing > left_limit) {
                current_bearing = right_limit;
                invert = true;
            } else if (left_limit > right_limit && current_bearing >= right_limit && current_bearing < left_limit) {
                current_bearing = right_limit;
                invert = true;
            } else {
                current_bearing += motor_step;
            }
        }

        // counterclockwise reading
        else {
            if (left_limit <= right_limit && current_bearing <= left_limit && current_bearing < right_limit) {
                current_bearing = left_limit;
                invert = false;
            } else if (left_limit > right_limit && current_bearing <= left_limit && current_bearing > right_limit) {
                current_bearing = left_limit;
                invert = false;
            } else {
                current_bearing -= motor_step;
            }
        }
    }
}

bool ScanningSonarTask::setLeft_limit(::base::Angle const & value) {
    left_limit = value;
    return (imaging_sonar_simulation::ScanningSonarTaskBase::setLeft_limit(value));
}

bool ScanningSonarTask::setRight_limit(::base::Angle const & value) {
    right_limit = value;
    return (imaging_sonar_simulation::ScanningSonarTaskBase::setRight_limit(value));
}

bool ScanningSonarTask::setMotor_step(::base::Angle const & value) {
    if (value.getRad() <= 0 || value > base::Angle::fromRad(3.6)) {
        RTT::log(RTT::Error) << "The step angle value must be positive and less or equal than 3.6 degrees." << RTT::endlog();
        return false;
    }

    motor_step = value;
    return (imaging_sonar_simulation::ScanningSonarTaskBase::setMotor_step(value));
}

bool ScanningSonarTask::setContinuous(bool value) {
    continuous = value;
    return (imaging_sonar_simulation::ScanningSonarTaskBase::setContinuous(value));
}

bool ScanningSonarTask::setBin_count(int value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive." << RTT::endlog();
        return false;
    }

    sonar_sim.bin_count = value;
    float height = sonar_sim.bin_count * 5.12;  // 5.12 pixels are needed for each bin
    Task::setupShader(height, true);
    return (imaging_sonar_simulation::TaskBase::setBin_count(value));
}
