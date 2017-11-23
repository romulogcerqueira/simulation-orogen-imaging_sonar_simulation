require 'vizkit'
require 'rock/bundles'
require "#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/imaging_sonar_simulation/scripts/pose_control.rb"

include Orocos
Bundles.load

def setup_task(task)
    # Load configuration settings
    Orocos.conf.load_dir("#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/imaging_sonar_simulation/scripts/config/")
    Orocos.conf.apply(task, ['default'], :override => true)

    # Load underwater scene
    scene = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/gazebo_scenes/scenes/ssiv_bahia/ssiv_bahia.world"
    task.world_file_path = scene
    task.configure

    # Write the sonar pose
    timer = Qt::Timer.new
    timer.connect(SIGNAL('timeout()')) do
        task.sonar_pose_cmd.write @sonar_pose
    end

    # Start the task
    timer.start(10)
    task.start
end

def setup_widgets(task)
    # Set the sonar widget
    sonar_gui = Vizkit.default_loader.SonarWidget
    sonar_gui.setGain(task.gain * 100.0)
    sonar_gui.setRange(task.range)
    sonar_gui.setMaxRange(150)

    sonar_gui.connect(SIGNAL('gainChanged(int)')) do |value|
        task.property("gain").write value / 100.0
    end
    sonar_gui.connect(SIGNAL('rangeChanged(int)')) do |value|
        task.property("range").write value
    end

    # Connect the components
    task.sonar_samples.connect_to sonar_gui, type: :buffer, size: 30
    # Set the task inspector
    task_inspector = Vizkit.default_loader.TaskInspector
    Vizkit.display task, :widget => task_inspector

    # Pose control
    pose_gui = PoseControl.new(@sonar_pose)
    pose_gui.connect(SIGNAL(:sonar_pose_changed)) do
        @sonar_pose = pose_gui.current_sonar_pose
    end

    # Start the Rock widgets
    begin
        sonar_gui.show
        pose_gui.show
        Vizkit.exec
    rescue Interrupt => e
        task.stop
        task.cleanup
    end
end

Orocos.run 'imaging_sonar_simulation::ScanningSonarTask' => 'sonar_scanning' do
    # Start the orocos task
    task = TaskContext.get 'sonar_scanning'
    setup_task(task)

    @sonar_pose = Types.base.samples.RigidBodyState.new
    @sonar_pose.targetFrame = "world"
    @sonar_pose.sourceFrame = "multibeam_sonar"
    @sonar_pose.position = Eigen::Vector3.new(48.24, 103.48, -3.37)
    @sonar_pose.orientation = Eigen::Quaternion.from_euler(Eigen::Vector3.new(1.57, 0.524, 0), 2, 1, 0)

    # Start the Rock widgets
    setup_widgets(task)
end
