/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace imaging_sonar_simulation;

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

void Task::init(const base::Angle& fovX, const base::Angle& fovY, uint value, float range, bool isHeight) {

	// init shader
	_normal_depth_map = vizkit3d_normal_depth_map::NormalDepthMap(range, fovX.getRad() * 0.5, fovY.getRad() * 0.5);
	_capture = vizkit3d_normal_depth_map::ImageViewerCaptureTool(fovY.getRad(), fovX.getRad(), value, isHeight);
	_capture.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
	_root = vizkit3dWorld->getWidget()->getRootNode();
	_normal_depth_map.addNodeChild(_root);
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
	_capture.setCameraPosition(eye, center, up);
}

