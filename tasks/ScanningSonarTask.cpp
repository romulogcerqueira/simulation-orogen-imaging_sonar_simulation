/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#include <osgDB/ReadFile>

#include <math.h>



using namespace gpu_sonar_simulation;

ScanningSonarTask::ScanningSonarTask(std::string const& name)
    : ScanningSonarTaskBase(name)
{
}

ScanningSonarTask::ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ScanningSonarTaskBase(name, engine)
{
}

ScanningSonarTask::~ScanningSonarTask()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ScanningSonarTask.hpp for more detailed
// documentation about them.

bool ScanningSonarTask::configureHook()
{
    if (! ScanningSonarTaskBase::configureHook())
        return false;
    return true;
}
bool ScanningSonarTask::startHook()
{
    if (! ScanningSonarTaskBase::startHook())
        return false;

    initSampleScene2();

    // these variables need to be updated after by RigidBodyState
    _transX = -14.0f;
    _transY = -4.0f;
    _transZ = 2.0f;
    _rotZ = 0.0f;

    return true;
}
void ScanningSonarTask::updateHook()
{

    ScanningSonarTaskBase::updateHook();

    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
	osg::Matrixd rock_coordinate_matrix =
			osg::Matrixd::rotate(-M_PI_2, osg::Vec3(0, 0, -1)) *
			osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

	// transformation matrixes multiplication
	osg::Matrixd matrix;
	matrix.setTrans(osg::Vec3(_transX, _transY, _transZ));
	matrix.setRotate(osg::Quat(_rotZ, osg::Vec3(0, 0, 1)));
	matrix.invert(matrix);

	// correct coordinate system and apply geometric transformations
	osg::Matrixd m = matrix * rock_coordinate_matrix;

	osg::Vec3 eye, center, up;
	m.getLookAt(eye, center, up);
	_capture.setCameraPosition(eye, center, up);

    // receive shader image
    osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
    cv::Mat cv_image = convertShaderOSG2CV(osg_image);

    // decode shader image
    cv::Mat raw_intensity = _scan_sonar.decodeShaderImage(cv_image);

    // get ping data
    std::vector<uint8_t> sonar_data = _scan_sonar.getPingData(raw_intensity);

    // write in beam_samples port
    base::samples::SonarBeam sonar_beam = _scan_sonar.simulateSonarBeam(sonar_data);
    _beam_samples.write(sonar_beam);

    // rotate the sonar
    if(_scan_sonar.isReverseScan())
    	_rotZ += osg::DegreesToRadians(_scan_sonar.getStepAngle());
    else
    	_rotZ -= osg::DegreesToRadians(_scan_sonar.getStepAngle());

}
void ScanningSonarTask::errorHook()
{
    ScanningSonarTaskBase::errorHook();
}
void ScanningSonarTask::stopHook()
{
    ScanningSonarTaskBase::stopHook();
}
void ScanningSonarTask::cleanupHook()
{
    ScanningSonarTaskBase::cleanupHook();
}

void ScanningSonarTask::initSampleScene()
{
	float fovY = 35.0f, fovX = 3.0f;
	int resolution = 800;

	NormalDepthMap normal_depth_map(_scan_sonar.getRange());
	ImageViewerCaptureTool capture(fovY, fovX, resolution);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	// create sample scene
	_root = new osg::Group();

	// create sample scene
	_root = new osg::Group();

	osg::Geode *object = new osg::Geode();

	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3( 20,0,0), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,30,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(-40,0,0), 1, 1)));

	_root->addChild(object);
	normal_depth_map.addNodeChild(_root);
	_normal_depth_map = normal_depth_map;


	_capture = capture;
}

void ScanningSonarTask::initSampleScene2()
{

	float fovY = 35.0f, fovX = 3.0f;
	int resolution = 800;

	NormalDepthMap normal_depth_map(_scan_sonar.getRange());
	ImageViewerCaptureTool capture(fovY, fovX, resolution);

	// create sample scene
	_root = new osg::Group();

	// load and scale objects
	osg::Node* manifold = osgDB::readNodeFile("resources/manifold.dae");

	osg::Matrix mat;
	mat.preMult(osg::Matrix::scale(0.01f, 0.01f, 0.01f));
	mat.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));

	osg::MatrixTransform *pTransform = new osg::MatrixTransform();
	pTransform->setMatrix(mat);
	pTransform->addChild(manifold);

	// add object to main node
	_root->addChild(pTransform);
	normal_depth_map.addNodeChild(_root);
	_normal_depth_map = normal_depth_map;

	_capture = capture;
}

bool ScanningSonarTask::setRange(double value)
{
	if(value < _scan_sonar.min_range)
		value = _scan_sonar.min_range;

	else if(value > _scan_sonar.max_range)
		value = _scan_sonar.max_range;

	_normal_depth_map.setMaxRange(value);
	_scan_sonar.setRange(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setRange(value);
}

bool ScanningSonarTask::setPing_pong_mode(bool value)
{
	_scan_sonar.setPingPongMode(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setPing_pong_mode(value);
}

bool ScanningSonarTask::setLimit_angle_left(double value)
{
	_scan_sonar.setLeftLimit(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setLimit_angle_left(value);
}

bool ScanningSonarTask::setLimit_angle_right(double value)
{
	_scan_sonar.setRightLimit(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setLimit_angle_right(value);
}

bool ScanningSonarTask::setStep_angle(double value)
{
	_scan_sonar.setStepAngle(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setStep_angle(value);
}

