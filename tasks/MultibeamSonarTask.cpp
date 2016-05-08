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
    if (value < 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

	_normal_depth_map.setMaxRange(value);
	_msonar.setRange(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setRange(value));
}

bool MultibeamSonarTask::setGain(double value) {
    if (value < 0 || value > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    _msonar.setGain(value);
    return (imaging_sonar_simulation::MultibeamSonarTaskBase::setGain(value));
}

bool MultibeamSonarTask::setNumber_of_bins(int value) {
    if (value < 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less than 1500." << RTT::endlog();
        return false;
    }

	_msonar.setNumberOfBins(value);
	return (imaging_sonar_simulation::MultibeamSonarTaskBase::setNumber_of_bins(value));
}

bool MultibeamSonarTask::setOrientation(::imaging_sonar_simulation::orientation::Type const & value) {
    switch (value) {
        case imaging_sonar_simulation::orientation::Horizontal:
            _current_orientation = imaging_sonar_simulation::orientation::Horizontal;
            break;
        case imaging_sonar_simulation::orientation::Vertical:
            _current_orientation = imaging_sonar_simulation::orientation::Vertical;
            break;
        default:
            throw std::invalid_argument("Orientation parameter does not match a known enum value");
    }

    return (imaging_sonar_simulation::MultibeamSonarTaskBase::setOrientation(value));
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MultibeamSonarTask.hpp for more detailed
// documentation about them.

bool MultibeamSonarTask::configureHook() {
	if (!MultibeamSonarTaskBase::configureHook())
		return false;

    _msonar.setRange(_range.value());
    _normal_depth_map.setMaxRange(_range.value());
    _msonar.setGain(_gain.value());
    _msonar.setNumberOfBins(_number_of_bins.value());
    _msonar.setNumberOfBeams(_number_of_beams.value());
    _msonar.setBeamWidth(_beam_width.value());
    _msonar.setBeamHeight(_beam_height.value());
    _current_orientation = _orientation.value();

    if (_msonar.getRange() < 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

    if (_msonar.getGain() < 0 || _msonar.getGain() > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    if (_msonar.getNumberOfBins() < 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less than 1500." << RTT::endlog();
        return false;
    }

    if (_msonar.getNumberOfBeams() < 64 || _msonar.getNumberOfBeams() > 512) {
        RTT::log(RTT::Error) << "The number of beams must be between 64 and 512." << RTT::endlog();
        return false;
    }

    if (_msonar.getBeamHeight().rad <= 0 || _msonar.getBeamWidth().rad <= 0) {
        RTT::log(RTT::Error) << "The sonar opening angles must be positives." << RTT::endlog();
        return false;
    }

	return true;
}

bool MultibeamSonarTask::startHook() {
	if (!MultibeamSonarTaskBase::startHook())
		return false;

	// set shader image parameters
	uint width = _msonar.getNumberOfBeams() * _msonar.getPixelsPerBeam();
	Task::init(_msonar.getBeamWidth(), _msonar.getBeamHeight(), width, _msonar.getRange(), false);

	return true;
}

void MultibeamSonarTask::updateHook() {
	MultibeamSonarTaskBase::updateHook();

	base::samples::RigidBodyState linkPose;

	if (_sonar_pose_cmd.read(linkPose) == RTT::NewData) {
	    base::samples::RigidBodyState multibeamSonarPose = rotatePose(linkPose);
		updateMultibeamSonarPose(multibeamSonarPose);
	}
}

void MultibeamSonarTask::updateMultibeamSonarPose(base::samples::RigidBodyState pose) {

	Task::updateSonarPose(pose);

	// receives shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// simulate sonar data
	std::vector<float> sonar_data = _msonar.codeSonarData(cv_image);

	// apply the "gain" (in this case, it is a light intensity change)
	float gain_factor = _msonar.getGain() / 0.5;
	std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), std::bind1st(std::multiplies<float>(), gain_factor));
	std::replace_if(sonar_data.begin(), sonar_data.end(), bind2nd(greater<float>(), 1.0), 1.0);

	// simulate sonar data
	base::samples::Sonar sonar = _msonar.simulateMultiBeam(sonar_data);
	_sonar_samples.write(sonar);

	// display shader image
	std::auto_ptr<Frame> frame(new Frame());
	cv::Mat cv_shader;
	cv_image.convertTo(cv_shader, CV_8UC3, 255);
	frame_helper::FrameHelper::copyMatToFrame(cv_shader, *frame.get());
	_shader_viewer.write(RTT::extras::ReadOnlyPointer<Frame>(frame.release()));
}

base::samples::RigidBodyState MultibeamSonarTask::rotatePose(base::samples::RigidBodyState pose) {
    base::samples::RigidBodyState new_pose;
    new_pose.position = pose.position;

    if (_current_orientation == imaging_sonar_simulation::orientation::Horizontal)
        new_pose.orientation = pose.orientation * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    else
        new_pose.orientation = pose.orientation * Eigen::AngleAxisd(-90, Eigen::Vector3d::UnitZ());

    return new_pose;
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
