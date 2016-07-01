/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

// Rock includes
#include <gpu_sonar_simulation/Utils.hpp>
#include <frame_helper/FrameHelper.h>

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

    // set the attributes
    range = _range.value();
    gain = _gain.value();
    normal_depth_map.setMaxRange(_range.value());
    sonar_sim.bin_count = _bin_count.value();
    sonar_sim.beam_width = _beam_width.value();
    sonar_sim.beam_height = _beam_height.value();

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

void Task::initShader(uint value, bool isHeight) {
    double const half_fovx = sonar_sim.beam_width.getRad() / 2;
    double const half_fovy = sonar_sim.beam_height.getRad() / 2;

    // initialize shader (NormalDepthMap and ImageViewerCaptureTool)
    normal_depth_map = vizkit3d_normal_depth_map::NormalDepthMap(range, half_fovx, half_fovy);
    capture = vizkit3d_normal_depth_map::ImageViewerCaptureTool(sonar_sim.beam_height.getRad(), sonar_sim.beam_width.getRad(), value, isHeight);
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
    cv::Mat cv_image;
    gpu_sonar_simulation::convertOSG2CV(osg_image, cv_image);

    // decode shader informations to sonar data
    sonar_sim.decodeShader(cv_image, bins);

    // apply the additional gain
    sonar_sim.applyAdditionalGain(bins, gain);

    // display shader image
    std::auto_ptr<Frame> frame(new Frame());
    cv_image.convertTo(cv_image, CV_8UC3, 255);
    frame_helper::FrameHelper::copyMatToFrame(cv_image, *frame.get());
    _shader_viewer.write(RTT::extras::ReadOnlyPointer<Frame>(frame.release()));
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

bool Task::setBin_count(int value) {
    if (value <= 0) {
        RTT::log(RTT::Error) << "The number of bins must be positive and less than 1500." << RTT::endlog();
        return false;
    }

    sonar_sim.bin_count = value;
    return (imaging_sonar_simulation::TaskBase::setBin_count(value));
}
