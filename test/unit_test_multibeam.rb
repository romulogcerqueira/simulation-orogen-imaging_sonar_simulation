require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'imaging_sonar_simulation::MultibeamSonarTask' do
    include Orocos::Test::Component

    start 'task', 'imaging_sonar_simulation::MultibeamSonarTask' => 'task'

    before do
        task.apply_conf_file("imaging_sonar_simulation::MultibeamSonarTask.yml",  ['default'] )
        task.world_file_path = "example.world"
    end

    it 'should not break while configuring, starting, stopping and cleaning up' do
        task.configure
        task.start
        task.stop
        task.cleanup
    end

    it 'should fail to configure if range < 0' do
        task.range = -5
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if range = 0' do
        task.range = 0
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if gain < 0.0' do
        task.gain = -5
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if gain > 1.0' do
        task.gain = 2.2
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if number of bins < 0' do
        task.bin_count = -20
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if number of bins = 0' do
        task.bin_count = 0
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if number of bins > 1500' do
        task.bin_count = 2000
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if number of beams < 64' do
        task.beam_count = 32
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if number of beams > 512' do
        task.beam_count = 700
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam width < 0' do
        task.beam_width = Types.base.Angle.new(:rad => -1.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam width = 0' do
        task.beam_width = Types.base.Angle.new(:rad => 0.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam height < 0' do
        task.beam_height = Types.base.Angle.new(:rad => -1.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam height = 0' do
        task.beam_height = Types.base.Angle.new(:rad => 0.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if sonar frequency < 100 Hz' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 0
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 293.15)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if sonar frequency > 1 MHz' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 3000
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 293.15)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water temperature < -6º C' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 250)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water temperature > 35º C' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 350)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water salinity < 0 ppt' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 290)
        new_attenuation.salinity = -5
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water salinity > 50 ppt' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 290)
        new_attenuation.salinity = 100
        new_attenuation.acidity = 8
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water pH < 7.7' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 290)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 7.3
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if water pH > 8.3' do
        new_attenuation = Types.imaging_sonar_simulation.AcousticAttenuationProperties.new
        new_attenuation.frequency = 300
        new_attenuation.temperature = Types.base.Temperature.new(:kelvin => 290)
        new_attenuation.salinity = 30
        new_attenuation.acidity = 9
        task.attenuation_properties = new_attenuation
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end
end
