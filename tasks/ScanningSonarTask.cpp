/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"


using namespace imaging_sonar_simulation;

ScanningSonarTask::ScanningSonarTask(std::string const& name) :
        ScanningSonarTaskBase(name) {
}

ScanningSonarTask::ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine) :
        ScanningSonarTaskBase(name, engine) {
}

ScanningSonarTask::~ScanningSonarTask() {
}

bool ScanningSonarTask::setEnd_angle(double value) {

    _scan_sonar.setEndAngle(value);

    return (imaging_sonar_simulation::ScanningSonarTaskBase::setEnd_angle(value));
}

bool ScanningSonarTask::setPing_pong_mode(bool value) {
    _scan_sonar.setPingPongMode(value);

    return (imaging_sonar_simulation::ScanningSonarTaskBase::setPing_pong_mode(value));
}

bool ScanningSonarTask::setRange(double value) {
    if (value < _scan_sonar.min_range)
        value = _scan_sonar.min_range;

    else if (value > _scan_sonar.max_range)
        value = _scan_sonar.max_range;

    _normal_depth_map.setMaxRange(value);
    _scan_sonar.setRange(value);

    return (imaging_sonar_simulation::ScanningSonarTaskBase::setRange(value));
}

bool ScanningSonarTask::setStart_angle(double value) {

    _scan_sonar.setStartAngle(value);

    return (imaging_sonar_simulation::ScanningSonarTaskBase::setStart_angle(value));
}

bool ScanningSonarTask::setStep_angle(double value) {

    _scan_sonar.setStepAngle(value);

    return (imaging_sonar_simulation::ScanningSonarTaskBase::setStep_angle(value));
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

    float fovY = 35.0f, fovX = 3.0f;
    int resolution = 1000;

    vizkit3dWorld->setCameraParams(320, 240, 45, 0.1, 100.0);
    vizkit3dWorld->getWidget()->setTransformer(false);
    vizkit3dWorld->getWidget()->setAxes(false);
    vizkit3dWorld->getWidget()->setAxesLabels(false);

    vizkit3d_normal_depth_map::NormalDepthMap normal_depth_map(_scan_sonar.getRange());
    vizkit3d_normal_depth_map::ImageViewerCaptureTool capture(fovY, fovX, resolution);

    // create sample scene
    _root = vizkit3dWorld->getWidget()->getRootNode();
    normal_depth_map.addNodeChild(_root);
    _normal_depth_map = normal_depth_map;

    _capture = capture;
    _rotZ = 0.0f;

    return true;
}

void ScanningSonarTask::updateScanningSonarPose(base::samples::RigidBodyState pose) {

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
    _capture.setCameraPosition(eye, center, up);

    // receive shader image
    osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
    cv::Mat cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

    // decode shader image
    cv::Mat raw_intensity = _scan_sonar.decodeShaderImage(cv_image);

    // get ping data
    std::vector<uint8_t> sonar_data = _scan_sonar.getPingData(raw_intensity);

    // write in beam_samples port
    base::samples::SonarBeam sonar_beam = _scan_sonar.simulateSonarBeam(sonar_data);
    _beam_samples.write(sonar_beam);
}

void ScanningSonarTask::updateCameraPose(base::samples::RigidBodyState cameraPose) {
    vizkit3dWorld->setTransformation(cameraPose);
    vizkit3dWorld->setCameraPose(cameraPose);
}

base::samples::RigidBodyState ScanningSonarTask::rotatePose(base::samples::RigidBodyState pose){

    base::samples::RigidBodyState new_pose;
    new_pose.position = pose.position;
    new_pose.orientation = pose.orientation * Eigen::AngleAxisd(_rotZ, Eigen::Vector3d::UnitZ());
    return new_pose;

}

void ScanningSonarTask::updateHook() {

    ScanningSonarTaskBase::updateHook();

    base::samples::RigidBodyState linkPose;

    if (_scanning_sonar_pose_cmd.read(linkPose) == RTT::NewData) {

        base::samples::RigidBodyState scanningSonarPose = rotatePose(linkPose);

        if (_show_gui.get()){
            updateCameraPose(scanningSonarPose);
        }

        vizkit3dWorld->notifyEvents();
        updateScanningSonarPose(scanningSonarPose);
    }

    if (_scan_sonar.isReverseScan()){
        _rotZ += base::Angle::deg2Rad(_scan_sonar.getStepAngle());
    	if(_rotZ >= base::Angle::deg2Rad(_scan_sonar.getEndAngle()))
    		_rotZ = base::Angle::deg2Rad(_scan_sonar.getStartAngle());}
    else{
        _rotZ -= base::Angle::deg2Rad(_scan_sonar.getStepAngle());
        if(_rotZ <= base::Angle::deg2Rad(_scan_sonar.getStartAngle()))
			_rotZ = base::Angle::deg2Rad(_scan_sonar.getEndAngle());
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
