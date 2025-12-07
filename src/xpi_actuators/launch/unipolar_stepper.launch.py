from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'in1_pin',
            default_value='17',
            description='GPIO BCM pin for ULN2003 IN1'
        ),
        DeclareLaunchArgument(
            'in2_pin',
            default_value='27',
            description='GPIO BCM pin for ULN2003 IN2'
        ),
        DeclareLaunchArgument(
            'in3_pin',
            default_value='22',
            description='GPIO BCM pin for ULN2003 IN3'
        ),
        DeclareLaunchArgument(
            'in4_pin',
            default_value='23',
            description='GPIO BCM pin for ULN2003 IN4'
        ),
        DeclareLaunchArgument(
            'steps_per_revolution',
            default_value='2048', # Common for 28BYJ-48 with gear reduction
            description='Number of steps per full revolution of the motor'
        ),
        DeclareLaunchArgument(
            'step_delay_ms',
            default_value='2', # Milliseconds, controls maximum speed
            description='Delay in milliseconds between each step (lower = faster)'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real stepper motor'
        ),
        
        Node(
            package='xpi_actuators',
            executable='unipolar_stepper_node',
            name='unipolar_stepper_driver',
            parameters=[{
                'in1_pin': LaunchConfiguration('in1_pin'),
                'in2_pin': LaunchConfiguration('in2_pin'),
                'in3_pin': LaunchConfiguration('in3_pin'),
                'in4_pin': LaunchConfiguration('in4_pin'),
                'steps_per_revolution': LaunchConfiguration('steps_per_revolution'),
                'step_delay_ms': LaunchConfiguration('step_delay_ms'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
