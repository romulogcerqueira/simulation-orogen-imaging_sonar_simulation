require 'orocos'
require 'readline'
#require 'vizkit'

include Orocos

## Initialize orocos ##
Orocos.initialize


#sonar_widget = Vizkit.default_loader.SonarWidget
#gui_beam = Vizkit.default_loader.SonarView

## Execute the task ##
Orocos.run 'gpu_sonar_simulation::ScanningSonarTask' => 'sonar_sim' do

    ## Get the specific task context ##
    sonar_sim = Orocos.name_service.get 'sonar_sim'

    ## Configure the tasks ##
    sonar_sim.configure

    ## Start the tasks ##
    sonar_sim.start
    
    ## Connect with Sonar Widget
    #sonar_sim.sonar_samples.connect_to sonar_widget
    #sonar_sim.beam_samples.connect_to gui_beam
    
    #sonar_widget.connect SIGNAL("rangeChanged(int)") do |range|
    #  sonar_sim.sonar_range = range
    #end
    
    #sonar_widget.connect SIGNAL("gainChanged(int)") do |gain|
    #  sonar_sim.sonar_gain = gain
    #end
    
    #Vizkit.exec
   
    Readline::readline("Press Enter to exit\n") do
    end
end

