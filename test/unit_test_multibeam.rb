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
end
