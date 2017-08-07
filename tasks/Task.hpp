/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef IMAGING_SONAR_SIMULATION_TASK_TASK_HPP
#define IMAGING_SONAR_SIMULATION_TASK_TASK_HPP

#include "imaging_sonar_simulation/TaskBase.hpp"

// Rock includes
#include <base/samples/RigidBodyState.hpp>
#include <gpu_sonar_simulation/Sonar.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>

namespace imaging_sonar_simulation{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','imaging_sonar_simulation::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        /** Sonar simulator */
        gpu_sonar_simulation::Sonar sonar_sim;

        /** Range value used by sonar simulator (in meters) */
        double range;

        /** Additional gain value used by sonar simulator (0.0 - 1.0) */
        double gain;

        /** Normal and depth map from an osg scene. */
        normal_depth_map::NormalDepthMap normal_depth_map;

        /** Capture the osg::Image from a node scene */
        normal_depth_map::ImageViewerCaptureTool capture;

        /**
        *  Initialize shader.
        *  @param value: size of width/height of shader image in pixels
        *  @param isHeight: if true, the value is related with image height
        *                   otherwise, the vale is related with image width
        */
        void setupShader(uint value, bool isHeight = true);

        /**
        *  Update sonar pose according to auv pose.
        *  @param pose: pose of the auv
        */
        void updateSonarPose(base::samples::RigidBodyState pose);

        /**
        *  Process shader image in bins intensity.
        *  @param osg_image: the shader image (normal, depth and angle informations) in osg::Image format
        *  @param bins: the output simulated sonar data (all beams) in float
        */
        void processShader(osg::ref_ptr<osg::Image>& osg_image, std::vector<float>& bins);

        /** Dynamically update sonar range
        *
        * @param value: desired range
        * @return if the process is finished successfully
        */
        virtual bool setRange(double value);

        /** Dynamically update sonar gain
        *
        * @param value: desired gain
        * @return if the process is finished successfully
        */
        virtual bool setGain(double value);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "imaging_sonar_simulation::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

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
    };
}

#endif
