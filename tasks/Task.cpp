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
    range = _range.value();
    gain = _gain.value();
    normal_depth_map.setMaxRange(_range.value());
    sonar_sim.bin_count = _bin_count.value();
    sonar_sim.beam_width = _beam_width.value();
    sonar_sim.beam_height = _beam_height.value();
    attenuation_properties = _attenuation_properties.value();

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

void Task::setupShader(uint value, bool isHeight) {
    // initialize shader (NormalDepthMap and ImageViewerCaptureTool)
    normal_depth_map = normal_depth_map::NormalDepthMap(range);
    capture = normal_depth_map::ImageViewerCaptureTool(sonar_sim.beam_height.getRad(), sonar_sim.beam_width.getRad(), value, isHeight);
    capture.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
    osg::ref_ptr<osg::Group> root = vizkit3dWorld->getWidget()->getRootNode();
    normal_depth_map.addNodeChild(root);
}

void Task::updateSonarPose(base::samples::RigidBodyState pose) {
    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
    osg::Matrixd rock_coordinate_matrix = osg::Matrixd::rotate( M_PI_2, osg::Vec3(0, 0, 1)) * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

    // transformation matrixes multiplication
    osg::Matrixd matrix;
    matrix.setTrans(osg::Vec3(pose.position.x(), pose.position.y(), pose.position.z()));
    matrix.setRotate(osg::Quat(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w()));
    matrix.invert(matrix);

    // correct coordinate system and apply geometric transformations
    osg::Matrixd m = matrix * rock_coordinate_matrix;
    osg::Vec3 eye, center, up;
    m.getLookAt(eye, center, up);
    capture.setCameraPosition(eye, center, up);
}

void Task::processShader(osg::ref_ptr<osg::Image>& osg_image, std::vector<float>& bins) {
    // receives shader image in opencv format
    cv::Mat cv_image, cv_depth;
    gpu_sonar_simulation::convertOSG2CV(osg_image, cv_image);
    osg::ref_ptr<osg::Image> osg_depth = capture.getDepthBuffer();
    gpu_sonar_simulation::convertOSG2CV(osg_depth, cv_depth);

    // replace depth matrix
    std::vector<cv::Mat> channels;
    cv::split(cv_image, channels);
    channels[1] = cv_depth;
    cv::merge(channels, cv_image);

    // decode shader informations to sonar data
    sonar_sim.decodeShader(cv_image, bins, _enable_speckle_noise.value());

    // apply the additional gain
    sonar_sim.applyAdditionalGain(bins, gain);

    // display shader image
    if (_write_shader_image.value()) {
        std::unique_ptr<Frame> frame(new Frame());
        cv_image.convertTo(cv_image, CV_8UC3, 255);
        cv::flip(cv_image, cv_image, 0);
        frame_helper::FrameHelper::copyMatToFrame(cv_image, *frame.get());
        frame->time = base::Time::now();
        _shader_image.write(RTT::extras::ReadOnlyPointer<Frame>(frame.release()));
    }
}

bool Task::setRange(double value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The range must be positive." << RTT::endlog();
        return false;
    }

    range = value;
    normal_depth_map.setMaxRange(value);
    return (imaging_sonar_simulation::TaskBase::setRange(value));
}

bool Task::setGain(double value) {
    if (value < 0 || value > 1) {
        RTT::log(RTT::Error) << "The gain must be between 0.0 and 1.0." << RTT::endlog();
        return false;
    }

    gain = value;
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
