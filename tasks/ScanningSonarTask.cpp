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

    initSampleScene();

    // these variables need to be updated after by RigidBodyState
    _transX = 0.0f;
    _transY = 0.0f;
    _transZ = 0.0f;
    _rotZ = 0.0f;

    return true;
}
void ScanningSonarTask::updateHook()
{

    ScanningSonarTaskBase::updateHook();

    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
	osg::Matrixd rock_coordinate_matrix =
			osg::Matrixd::rotate( M_PI_2, osg::Vec3(0, 0, 1)) *
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

    cv::imshow("teste", cv_image);
    cv::waitKey(30);


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
	int resolution = 1000;

	NormalDepthMap normal_depth_map(_scan_sonar.getRange());
	ImageViewerCaptureTool capture(fovY, fovX, resolution);

	// create sample scene
	_root = new osg::Group();


	// load manifold 3d model
	osg::Node* manifold_model = osgDB::readNodeFile("resources/manifold.dae");
	osg::Matrix mat1;
	mat1.preMult(osg::Matrix::translate(10, 0, -2.0));
	mat1.preMult(osg::Matrix::scale(0.01f,0.01f,0.01f));
	mat1.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *manifold_transf = new osg::MatrixTransform();
	manifold_transf->setMatrix(mat1);
	manifold_transf->addChild(manifold_model);


	// load flatfish 3D model
	osg::Node* flatfish_model = osgDB::readNodeFile("resources/flatfish_02.dae");
	osg::Matrix mat2;
	mat2.preMult(osg::Matrix::translate(-15, -5, 2.5));
	mat2.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *flatfish_transf = new osg::MatrixTransform();
	flatfish_transf->setMatrix(mat2);
	flatfish_transf->addChild(flatfish_model);


	// load flatfish 3D model
	osg::Node* oilrig_model = osgDB::readNodeFile("resources/oil_rig_manifold.dae");
	osg::Matrix mat3;
	mat3.preMult(osg::Matrix::translate(0, 13, 0));
	mat3.preMult(osg::Matrix::scale(0.1f,0.1f,0.1f));
	mat3.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *oilrig_transf = new osg::MatrixTransform();
	oilrig_transf->setMatrix(mat3);
	oilrig_transf->addChild(oilrig_model);




	// add objects to main node
	_root->addChild(manifold_transf);
	_root->addChild(flatfish_transf);
	_root->addChild(oilrig_transf);

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

bool ScanningSonarTask::setStart_angle(double value)
{
	_scan_sonar.setStartAngle(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setStart_angle(value);
}

bool ScanningSonarTask::setEnd_angle(double value)
{
	_scan_sonar.setEndAngle(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setEnd_angle(value);
}

bool ScanningSonarTask::setStep_angle(double value)
{
	_scan_sonar.setStepAngle(value);

	 return gpu_sonar_simulation::ScanningSonarTaskBase::setStep_angle(value);
}

