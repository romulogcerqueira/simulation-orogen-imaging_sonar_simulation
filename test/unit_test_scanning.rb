require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'imaging_sonar_simulation::ScanningSonarTask' do
    include Orocos::Test::Component

    start 'task', 'imaging_sonar_simulation::ScanningSonarTask' => 'task'

    before do
        task.apply_conf_file("imaging_sonar_simulation::ScanningSonarTask.yml",  ['default'] )
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

    it 'should fail to configure if beam width < 0' do
        task.beam_width = Types::Base::Angle.new(:rad => -1.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam width = 0' do
        task.beam_width = Types::Base::Angle.new(:rad => 0.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam height < 0' do
        task.beam_height = Types::Base::Angle.new(:rad => -1.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if beam height = 0' do
        task.beam_height = Types::Base::Angle.new(:rad => 0.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if motor step angle < 0' do
        task.step_angle = Types::Base::Angle.new(:rad => -1.0)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end

    it 'should fail to configure if motor step angle > 3.6 degrees' do
        task.step_angle = Types::Base::Angle.new(:rad => Math::PI)
        assert_raises(Orocos::StateTransitionFailed) { task.configure }
    end
end
