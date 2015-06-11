/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef GPU_SONAR_SIMULATION_SCANNINGSONARTASK_TASK_HPP
#define GPU_SONAR_SIMULATION_SCANNINGSONARTASK_TASK_HPP

#include "gpu_sonar_simulation/ScanningSonarTaskBase.hpp"

#include <gpu_sonar_simulation/ScanSonar.hpp>
#include <gpu_sonar_simulation/SonarUtils.hpp>

#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>

using namespace cv;
using namespace osg;
using namespace gpu_sonar_simulation;
using namespace vizkit3d_normal_depth_map;

namespace gpu_sonar_simulation {

    /*! \class ScanningSonarTask 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','gpu_sonar_simulation::ScanningSonarTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class ScanningSonarTask : public ScanningSonarTaskBase
    {
	friend class ScanningSonarTaskBase;
    protected:



    public:
        /** TaskContext constructor for ScanningSonarTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        ScanningSonarTask(std::string const& name = "gpu_sonar_simulation::ScanningSonarTask");

        /** TaskContext constructor for ScanningSonarTask 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        ScanningSonarTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of ScanningSonarTask
         */
        ~ScanningSonarTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        void initSampleScene();
        void initSampleScene2();


        // Dynamic Properties
        bool setRange(double value);
        bool setPing_pong_mode(bool value);
        bool setLimit_angle_left(double value);
        bool setLimit_angle_right(double value);
        bool setStep_angle(double value);


private:
        osg::ref_ptr<osg::Group> _root;
        NormalDepthMap _normal_depth_map;
        ImageViewerCaptureTool _capture;
        ScanSonar _scan_sonar;
    };
}

#endif

