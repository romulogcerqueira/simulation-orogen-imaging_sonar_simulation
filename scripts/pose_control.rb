require 'vizkit'

class PoseControl < Qt::Widget

    signals 'sonar_pose_changed()'

    slots 'x_target_changed()',
          'y_target_changed()',
          'z_target_changed()',
          'yaw_target_changed()',
          'pitch_target_changed()',
          'body_frame_control_clicked()'

    # The body control buttons identifiers
    BODY_CONTROL_YDIR_PLUS   = 0
    BODY_CONTROL_YDIR_MINUS  = 1
    BODY_CONTROL_XDIR_PLUS   = 2
    BODY_CONTROL_XDIR_MINUS  = 3
    BODY_CONTROL_YAW_PLUS    = 4
    BODY_CONTROL_YAW_MINUS   = 5
    BODY_CONTROL_PITCH_PLUS  = 6
    BODY_CONTROL_PITCH_MINUS = 7
    BODY_CONTROL_ZDIR_PLUS   = 8
    BODY_CONTROL_ZDIR_MINUS  = 9

    # The values used to increment/decrement the body relative motion
    POS_INCREMENT_VALUES = [10, 5, 1, 0.5, 0.1]
    ROT_INCREMENT_VALUES = [45, 30, 15, 5, 1]

    def initialize(initial_pose, parent = nil, width = 520, height = 300)
        super(parent)
        resize(width, height)
        setWindowTitle("Sonar Pose Control")
        @ui = Vizkit.default_loader.load("#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/imaging_sonar_simulation/scripts/pose_control.ui")

        main_layout = Qt::HBoxLayout.new(self)
        main_layout.add_widget @ui

        @current_pose = {   x: initial_pose.position.x,
                            y: initial_pose.position.y,
                            z: initial_pose.position.z,
                            pitch: initial_pose.orientation.pitch,
                            yaw: initial_pose.orientation.yaw
                        }

        setup_pose_layout
        setup_scale_layout
        set_pose_display

        connect @ui.x_target_editor,     SIGNAL('valueChanged(double)'), self, SLOT('x_target_changed()')
        connect @ui.y_target_editor,     SIGNAL('valueChanged(double)'), self, SLOT('y_target_changed()')
        connect @ui.z_target_editor,     SIGNAL('valueChanged(double)'), self, SLOT('z_target_changed()')
        connect @ui.yaw_target_editor,   SIGNAL('valueChanged(double)'), self, SLOT('yaw_target_changed()')
        connect @ui.pitch_target_editor, SIGNAL('valueChanged(double)'), self, SLOT('pitch_target_changed()')
    end

    #
    # Track changes in x value
    #
    def setup_pose_layout
        @body_control_buttons = [
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowLeft), "", self),
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowRight), "", self),
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowUp), "", self),
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowDown), "", self),
            Qt::PushButton.new("Yaw+", self),
            Qt::PushButton.new("Yaw-", self) ,
            Qt::PushButton.new("Pitch+", self),
            Qt::PushButton.new("Pitch-", self),
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowUp), "", self),
            Qt::PushButton.new(style.standardIcon(Qt::Style::SP_ArrowDown), "", self)
        ]

        @body_control_buttons.each do |button|
            connect button, SIGNAL(:clicked), self, SLOT(:body_frame_control_clicked)
        end

        @ui.pose_layout.add_widget(@body_control_buttons[0], 2, 0)
        @ui.pose_layout.add_widget(@body_control_buttons[1], 2, 2)
        @ui.pose_layout.add_widget(@body_control_buttons[2], 1, 1)
        @ui.pose_layout.add_widget(@body_control_buttons[3], 3, 1)
        @ui.pose_layout.add_widget(@body_control_buttons[4], 1, 0)
        @ui.pose_layout.add_widget(@body_control_buttons[5], 1, 2)
        @ui.pose_layout.add_widget(@body_control_buttons[6], 3, 0)
        @ui.pose_layout.add_widget(@body_control_buttons[7], 3, 2)

        @ui.depth_layout.add_widget(@body_control_buttons[8], 1, 0)
        @ui.depth_layout.add_widget(@body_control_buttons[9], 2, 0)
    end

    def setup_scale_layout
        POS_INCREMENT_VALUES.each do |value|
            @ui.pos_scale_selector.add_item("%.1f m" % [value], Qt::Variant.new(value))
        end
        ROT_INCREMENT_VALUES.each do |value|
            @ui.rot_scale_selector.add_item("%.1f°" % [value], Qt::Variant.new(value))
        end

        # the default scale
        @ui.pos_scale_selector.currentIndex = 2 # 1 meter position
        @ui.rot_scale_selector.currentIndex = 3 # 5 degrees angle
    end

    #
    # Intercept the body control buttons clicked
    # Move the target_pose relative to the body
    #
    def body_frame_control_clicked
        button = @body_control_buttons.find { |m| m == sender}
        if button
            index = @body_control_buttons.index(button)

            case index
            when BODY_CONTROL_XDIR_MINUS, BODY_CONTROL_YDIR_MINUS
                xinc_world, yinc_world = body_to_world(index, pos_increment)
                @current_pose[:x] -= xinc_world
                @current_pose[:y] -= yinc_world
            when BODY_CONTROL_XDIR_PLUS, BODY_CONTROL_YDIR_PLUS
                xinc_world, yinc_world = body_to_world(index, pos_increment)
                @current_pose[:x] += xinc_world
                @current_pose[:y] += yinc_world
            when BODY_CONTROL_ZDIR_MINUS
                @current_pose[:z] -= pos_increment
            when BODY_CONTROL_ZDIR_PLUS
                @current_pose[:z] += pos_increment
            when BODY_CONTROL_YAW_MINUS
                @current_pose[:yaw] -= rot_increment
            when BODY_CONTROL_YAW_PLUS
                @current_pose[:yaw] += rot_increment
            when BODY_CONTROL_PITCH_MINUS
                @current_pose[:pitch] -= rot_increment
            when BODY_CONTROL_PITCH_PLUS
                @current_pose[:pitch] += rot_increment
            end

            set_pose_display
            emit sonar_pose_changed
        end
    end

    #
    # Returns the increment value for position
    #
    def pos_increment
        @ui.pos_scale_selector.item_data(@ui.pos_scale_selector.current_index).to_float
    end

    #
    # Returns the increment value for rotation
    #
    def rot_increment
        @ui.rot_scale_selector.item_data(@ui.rot_scale_selector.current_index).to_int * Math::PI / 180
    end

    #
    # Convert body-relative to world-relative
    #
    def body_to_world(button, body_pos)
        cosine = Math.cos(@current_pose[:yaw])
        sine = Math.sin(@current_pose[:yaw])
        world_x = 0
        world_y = 0
        if button == BODY_CONTROL_YDIR_MINUS ||  button == BODY_CONTROL_YDIR_PLUS
            world_x = -body_pos * sine
            world_y = body_pos * cosine
        elsif button == BODY_CONTROL_XDIR_MINUS ||  button == BODY_CONTROL_XDIR_PLUS
            world_x = body_pos * cosine
            world_y = body_pos * sine
        end
        [world_x, world_y]
    end

    #
    # Update the current pose in the widget labels
    #
    def set_pose_display
        @ui.x_current_label.text      = "%.2f" % [@current_pose[:x]]
        @ui.x_target_editor.value     = @current_pose[:x]

        @ui.y_current_label.text      = "%.2f" % [@current_pose[:y]]
        @ui.y_target_editor.value     = @current_pose[:y]

        @ui.z_current_label.text      = "%.2f" % [@current_pose[:z]]
        @ui.z_target_editor.value     = @current_pose[:z]

        @ui.yaw_current_label.text    = "%.2f" % [@current_pose[:yaw] * 180 / Math::PI]
        @ui.yaw_target_editor.value   = @current_pose[:yaw] * 180 / Math::PI

        @ui.pitch_current_label.text  = "%.2f" % [@current_pose[:pitch] * 180 / Math::PI]
        @ui.pitch_target_editor.value = @current_pose[:pitch] * 180 / Math::PI
    end

    #
    # Track changes in x value
    #
    def x_target_changed
        @current_pose[:x] = @ui.x_target_editor.value
        set_pose_display
        emit sonar_pose_changed
    end

    #
    # Track changes in y value
    #
    def y_target_changed
        @current_pose[:y] = @ui.y_target_editor.value
        set_pose_display
        emit sonar_pose_changed
    end

    #
    # Track changes in z value
    #
    def z_target_changed
        @current_pose[:z] = @ui.z_target_editor.value
        set_pose_display
        emit sonar_pose_changed
    end

    #
    # Track changes in yaw value
    #
    def yaw_target_changed
        @current_pose[:yaw] = @ui.yaw_target_editor.value * Math::PI / 180
        set_pose_display
        emit sonar_pose_changed
    end

    #
    # Track changes in pitch value
    #
    def pitch_target_changed
        @current_pose[:pitch] = @ui.pitch_target_editor.value * Math::PI / 180
        set_pose_display
        emit sonar_pose_changed
    end


    #
    # Set current pose
    #
    def current_sonar_pose
        sample = Types.base.samples.RigidBodyState.new
        sample.targetFrame = "world"
        sample.sourceFrame = "multibeam_sonar"
        sample.position = Eigen::Vector3.new(@current_pose[:x], @current_pose[:y], @current_pose[:z])

        q = Eigen::Quaternion.from_angle_axis(@current_pose[:yaw], Eigen::Vector3.UnitZ) *
            Eigen::Quaternion.from_angle_axis(@current_pose[:pitch], Eigen::Vector3.UnitY) *
            Eigen::Quaternion.from_angle_axis(0, Eigen::Vector3.UnitX)

        sample.orientation = q
        sample
    end
end

# p = PoseControl.new
# p.show
# Vizkit.exec
