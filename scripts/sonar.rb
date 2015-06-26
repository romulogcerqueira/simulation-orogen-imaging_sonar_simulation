# In this sample test we send an array with the model state
# updateHook read this information and update the model joints and positions
#
require 'orocos'
require 'sdf'
require 'vizkit'
require 'io/console'

include Orocos
Orocos.initialize

model_path = File.join(Dir.pwd, 'test_data', 'models')

world_path = File.join(Dir.pwd, 'test_data', 'test.world')

SDF::XML.model_path << model_path
sdf = SDF::Root.load(world_path)

## Execute the task ##
Orocos.run 'imaging_sonar_simulation::ScanningSonarTask' => 'sonar_sim' do

    ## Get the specific task context ##
    sonar_sim = Orocos.name_service.get 'sonar_sim'

    #set an array with model paths
    sonar_sim.model_paths = [model_path]
    #path to world file
    sonar_sim.world_file_path = world_path
    #show gui
    sonar_sim.show_gui = true

    ## Configure the tasks ##
    sonar_sim.configure

    ## Start the tasks ##
    sonar_sim.start
   
    Readline::readline("Press Enter to exit\n") do
    end

end
