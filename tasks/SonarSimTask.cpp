/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SonarSimTask.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <osgDB/WriteFile>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

SonarSimTask::SonarSimTask(std::string const& name) :
		SonarSimTaskBase(name) {
}

SonarSimTask::SonarSimTask(std::string const& name,
		RTT::ExecutionEngine* engine) :
		SonarSimTaskBase(name, engine) {
}

SonarSimTask::~SonarSimTask() {
}

//draw the scene with a small ball in the center with a big cube, cylinder and cone in back
void SonarSimTask::makeSampleScene(osg::ref_ptr<osg::Group> root) {

	osg::Geode *object = new osg::Geode();

	osg::ShapeDrawable *refSph1 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,0,0), 1));
	refSph1->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));

	osg::ShapeDrawable *refSph2 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,0,0), 1));
	refSph2->setColor(osg::Vec4(1.0f,0.5f,0.0f,1.0f));

	osg::ShapeDrawable *refBox1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 10), 1));
	refBox1->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));

	osg::ShapeDrawable *refBox2 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, -10), 1));
	refBox2->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

	// Sphere
	object->addDrawable(refSph1);
	object->addDrawable(refSph2);
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,-10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,-10,0), 1)));

	// Box
	object->addDrawable(refBox1);
	object->addDrawable(refBox2);
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(10, 0, 10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(10, 0, -10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(-10, 0, 10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(-10, 0, -10), 1)));

	// Cone
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 10, 10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 10, -10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, -10, 10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, -10, -10), 1, 1)));

	root->addChild(object);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SonarSimTask.hpp for more detailed
// documentation about them.

bool SonarSimTask::configureHook() {
	if (!SonarSimTaskBase::configureHook())
		return false;

	return true;
}

void SonarSimTask::initScene(double range)
{
	uint width = 640, height = 480;

	NormalDepthMap normal_depth_map(range);
	ImageViewerCaptureTool capture(width, height);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	// create the scene
	_root = new osg::Group();
	makeSampleScene(_root);

	// correct the view
	_m = capture.getViewMatrix();
	_m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(90.0), osg::X_AXIS)));
	capture.setViewMatrix(_m);

	_root = normal_depth_map.applyShaderNormalDepthMap(_root);
	_capture = capture;
}


cv::Mat SonarSimTask::getFrame(double range, double degree){

	osg::Matrix m = getScanMatrix(_capture.getViewMatrix(), degree);

	_capture.setViewMatrix(m);

    osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_root);

    return (convertShaderOSG2CV(osg_image));
}


bool SonarSimTask::startHook() {
	if (!SonarSimTaskBase::startHook())
		return false;

	SonarConfig config;

	sim.setup(config);
	initScene(sim.getMaxDistance());

	return true;
}

void SonarSimTask::updateHook() {
	SonarSimTaskBase::updateHook();

	// receive shader image
	cv::Mat shader_image = getFrame(sim.getMaxDistance(), 1.8);

	// decode raw image
	cv::Mat raw_intensity = sim.decodeRawImage(shader_image);
	std::vector<uint8_t> data = sim.getPingIntensity(raw_intensity);

//	shader_image *= 255;
//	cv::imshow("teste", shader_image);
//	cv::waitKey(0);


	// simulate beam
	base::samples::SonarBeam beam = sim.simulateSonarBeam(data);

	_beam_samples.write(beam);
}

void SonarSimTask::errorHook() {
	SonarSimTaskBase::errorHook();
}
void SonarSimTask::stopHook() {
	SonarSimTaskBase::stopHook();
}
void SonarSimTask::cleanupHook() {
	SonarSimTaskBase::cleanupHook();
}
