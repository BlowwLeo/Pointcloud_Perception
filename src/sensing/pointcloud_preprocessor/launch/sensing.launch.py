import launch
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    param_file = LaunchConfiguration('param_file').perform(context)
    topics = LaunchConfiguration('pointcloud_topics').perform(context)
    topics_list = yaml.safe_load(topics)  # Convert the string to a list

    input_twist = LaunchConfiguration('input/twist').perform(context)
    input_imu = LaunchConfiguration('input/imu').perform(context)

    nodes = []
    input_concatenate_node=[]
    for topic in topics_list:
        node = Node(
            package='pointcloud_preprocessor',
            executable='distortion_corrector_node',
            name='distortion_corrector_{}'.format(topic.strip('/').replace('/', '_')),
            remappings=[
                ('~/input/pointcloud', topic),
                ('~/input/twist', input_twist),
                ('~/input/imu', input_imu),
                ('~/output/pointcloud', '/rectified_{}'.format(topic.strip('/')))
            ],
            parameters=[param_file]
        )
        nodes.append(node)

        node = Node(
            package='pointcloud_preprocessor',
            executable='voxel_grid_outlier_filter_node',
            name='voxel_outlier_filter_{}'.format(topic.strip('/')),
            remappings=[
                ('input', '/rectified_{}'.format(topic.strip('/'))),
                ('output', '/filtered_{}'.format(topic.strip('/')))
            ]
        )
        nodes.append(node)
        input_concatenate_node.append('/filtered_{}'.format(topic.strip('/')))

    if (len(topics_list) > 1):
        node = Node(
        package='pointcloud_preprocessor',
        executable='concatenate_data_node',
        name='concat_node',
        remappings=[
            ('~/output/points', '/testtpointcloud'),
            ('~/input/twist', input_twist)
        ],
        parameters=[
            {
                "input_topics": input_concatenate_node,
                "output_frame": LaunchConfiguration("output_frame"),
            }
        ]
        )

        nodes.append(node)

        
    return nodes



def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        'pointcloud_topics',
        "['/pointcloud1', '/pointcloud2']"
    )
    add_launch_arg(
        'input/twist', '/sensing/vehicle_velocity_converter/twist_with_covariance'
    )
    add_launch_arg(
        'input/imu', '/sensing/imu/imu_data'
    )
    add_launch_arg(
        'param_file', '$(find-pkg-share pointcloud_preprocessor)/config/distortion_corrector_node.param.yaml'
    )
    add_launch_arg(
        'output_frame', 'base_link'
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
