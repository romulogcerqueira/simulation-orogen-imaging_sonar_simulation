require 'orocos'
require 'vizkit'

include Orocos
Orocos.initialize

scene = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], 'simulation', 'uwmodels', 'scenes', 'ssiv_bahia', 'ssiv_bahia.world')

Orocos.run 'imaging_sonar_simulation::MultibeamSonarTask' => 'sonar_multibeam' do
    sonar_pose = Types.base.samples.RigidBodyState.new
    sonar_pose.targetFrame = "world"
    sonar_pose.sourceFrame = "multibeam_sonar"
    sonar_pose.position = Eigen::Vector3.new(-0.133, -0.005, -0.366)
    sonar_pose.orientation = Eigen::Quaternion.from_euler(Eigen::Vector3.new(0, 0.524, 0), 2, 1, 0)

    sonar_multibeam = TaskContext.get 'sonar_multibeam'
    sonar_multibeam.world_file_path = scene
    sonar_multibeam.configure

    timer = Qt::Timer.new
    timer.connect(SIGNAL('timeout()')) do
        sonar_multibeam.sonar_pose_cmd.write sonar_pose
    end

    timer.start(10)
    sonar_multibeam.start

    task_inspector = Vizkit.default_loader.TaskInspector
    Vizkit.display sonar_multibeam, :widget => task_inspector

    begin
        Vizkit.exec
    rescue Interrupt => e
        sonar_multibeam.stop
        sonar_multibeam.cleanup
    end
end
