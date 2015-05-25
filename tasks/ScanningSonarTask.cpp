/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanningSonarTask.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>

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

    initSampleScene();

    return true;
}
void ScanningSonarTask::updateHook()
{
    ScanningSonarTaskBase::updateHook();

    float step_angle = 1.8;

    // receive shader image
    osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
    cv::Mat cv_image = convertShaderOSG2CV(osg_image);

    // decode shader image
    cv::Mat raw_intensity = _scan_sonar.decodeShaderImage(cv_image);

    // get ping data
    std::vector<uint8_t> sonar_data = _scan_sonar.getPingData(raw_intensity);

    // write in beam_samples port
    base::samples::SonarBeam sonar_beam = _scan_sonar.simulateSonarBeam(sonar_data, step_angle);
    _beam_samples.write(sonar_beam);

    // rotate the sonar
    osg::Matrix m = _capture.getViewMatrix();
    m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-step_angle), osg::Z_AXIS)));
    _capture.setViewMatrix(m);

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
	uint width = 640, height = 480;
	float range = 50.0;

	NormalDepthMap normal_depth_map(range);
	ImageViewerCaptureTool capture(width, height);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	// create sample scene
	_root = new osg::Group();

	osg::Geode *object = new osg::Geode();

	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3( 20,0,0), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,30,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(-40,0,0), 1, 1)));

	_root->addChild(object);
	normal_depth_map.addNodeChild(_root);

	// correct the view
	osg::Matrix m = capture.getViewMatrix();
	m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(90.0), osg::X_AXIS)));
	capture.setViewMatrix(m);

	_normal_depth_map = normal_depth_map;
	_capture = capture;
}


