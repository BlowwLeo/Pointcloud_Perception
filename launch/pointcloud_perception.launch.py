import yaml
import launch
import ast
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    #input pointclouds
    topics = LaunchConfiguration('pointcloud_topics').perform(context)
    topics_list = yaml.safe_load(topics)

    #distortion_corrector
    param_file_distortion_corrector = LaunchConfiguration('param_file_distortion_corrector').perform(context)
    input_twist = LaunchConfiguration('input/twist').perform(context)
    input_imu = LaunchConfiguration('input/imu').perform(context)

    #cropbox_filter
    crop_coord_str = LaunchConfiguration('crop_coord').perform(context)
    crop_coord = ast.literal_eval(crop_coord_str)

    #lidar_centerpoint
    model_param_path = LaunchConfiguration('model_param_path')
    ml_package_param_path = LaunchConfiguration('ml_package_param_path').perform(context)
    class_remapper_param_path = LaunchConfiguration('class_remapper_param_path').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)

    #obstacle_validator
    param_file_obstacle_validator= LaunchConfiguration('param_file_obstacle_validator').perform(context)

    #ground_segmentation
    param_file_ground_segmentation= LaunchConfiguration('param_file_ground_segmentation').perform(context)

    #Apollo
    param_file_apollo=LaunchConfiguration('param_file_apollo').perform(context)
    onnx_file_apollo=LaunchConfiguration('onnx_file_apollo').perform(context)

    #shape_estimation
    param_file_shape_estimation=LaunchConfiguration('param_file_shape_estimation').perform(context)

    #detection_by_tracker
    param_file_detection_by_tracker=LaunchConfiguration('param_file_detection_by_tracker').perform(context)

    #object_merger
    param_file_object_merger=LaunchConfiguration('param_file_object_merger').perform(context)
    distance_threshold_list_path=LaunchConfiguration('distance_threshold_list_path').perform(context)
    data_association_matrix_path=LaunchConfiguration('data_association_matrix_path').perform(context)
    
    #object_merger
    config_path_object_tracker=LaunchConfiguration('config_path_object_tracker').perform(context)

    nodes = []
    input_concatenate_node = []

    #Launch distrotion_corerctor and outlier_filer for each topic in input
    for topic in topics_list:
        nodes.append(ComposableNode(
            package='pointcloud_preprocessor',
            plugin='pointcloud_preprocessor::DistortionCorrectorComponent',
            name='distortion_corrector_{}'.format(topic.strip('/').replace('/', '_')),
            remappings=[
                ('~/input/pointcloud', topic),
                ('~/input/twist', input_twist),
                ('~/input/imu', input_imu),
                ('~/output/pointcloud', '/rectified_{}'.format(topic.strip('/')))
            ],
            parameters=[param_file_distortion_corrector]
        ))

        nodes.append(ComposableNode(
            package='pointcloud_preprocessor',
            plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
            name='voxel_outlier_filter_{}'.format(topic.strip('/')),
            remappings=[
                ('input', '/rectified_{}'.format(topic.strip('/'))),
                ('output', '/filtered_{}'.format(topic.strip('/')))
            ]
        ))
        input_cropbox_filter='/filtered_{}'.format(topic.strip('/'))
        if len(topics_list) > 1:
            input_concatenate_node.append('/filtered_{}'.format(topic.strip('/')))

    #Launch concatenate_pointcloud
    if len(topics_list) > 1:
        nodes.append(ComposableNode(
            package='pointcloud_preprocessor',
            plugin='pointcloud_preprocessor::PointCloudConcatenationComponent',
            name='concat_node',
            remappings=[
                ('~/input/twist', input_twist)
            ],
            parameters=[{
                "input_topics": input_concatenate_node,
                "output_frame": LaunchConfiguration("output_frame").perform(context)
            }]
        ))
        input_cropbox_filter='/output'
    
    #Launch cropbox_filter
    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            ("input",input_cropbox_filter),
            ("output",'/output_cropbox'),
        ],
        parameters=[
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "min_x": crop_coord[0],
                "max_x": crop_coord[1],
                "min_y": crop_coord[2],
                "max_y": crop_coord[3],
                "min_z": crop_coord[4],
                "max_z": crop_coord[5],
                "negative": False,
            }
        ],
    ))

    input_perception='/output_cropbox'

    #Launch downsample_filter
    if LaunchConfiguration('launch_downsample_node').perform(context)=='true':
        nodes.append(ComposableNode(
            package='pointcloud_preprocessor',
            plugin='pointcloud_preprocessor::ApproximateDownsampleFilterComponent',
            name='approximate_downsample_node',
            remappings=[
                ('input', '/output_cropbox'),
                ('output', '/output_downsample')
            ],
            parameters=[{
                "input_frame": "base_link",
                "output_frame": "base_link",
                "voxel_size_x": LaunchConfiguration('voxel_size_x'),
                "voxel_size_y": LaunchConfiguration('voxel_size_y'),
                "voxel_size_z": LaunchConfiguration('voxel_size_z'),
            }]
        ))

        input_perception='/output_downsample'
   
    #Launch lidar_centerpoint
    nodes.append(ComposableNode(
            package='lidar_centerpoint',
            plugin='centerpoint::LidarCenterPointNode',
            name='lidar_centerpoint',
            remappings=[
                ('~/input/pointcloud', input_perception),
                ('~/output/objects', '/output_centerpoint')
            ],
            parameters=[
                model_param_path,
                ml_package_param_path,
                class_remapper_param_path,
                {'encoder_onnx_path': model_path + '/pts_voxel_encoder_centerpoint.onnx'},
                {'encoder_engine_path': model_path + '/pts_voxel_encoder_centerpoint.engine'},
                {'head_onnx_path': model_path + '/pts_backbone_neck_head_centerpoint.onnx'},
                {'head_engine_path': model_path + '/pts_backbone_neck_head_centerpoint.engine'}
            ]
        ))
    
    #Launch ground_segmentation  
    nodes.append(ComposableNode(
            package='ground_segmentation',
            plugin='ground_segmentation::RayGroundFilterComponent',
            name='ground_segmentation',
            remappings=[
                ('input', input_perception),
                ('output','/output_ground_segmentation')
            ],
            parameters=[param_file_ground_segmentation]
        ))
    
    #Launch obstacle_pointcloud_base_validator
    nodes.append(ComposableNode(
            package='detected_object_validation',
            plugin='obstacle_pointcloud_based_validator::ObstaclePointCloudBasedValidator',
            name='obstale_pointcloud_based_validator_node',
            remappings=[
                ('~/input/detected_objects', '/output_centerpoint'),
                ('~/input/obstacle_pointcloud','/output_ground_segmentation'),
                ('~/output/objects', '/output_valid_objects')
            ],
            parameters=[param_file_obstacle_validator]
        ))
    
    #Launch Apollo
    nodes.append(ComposableNode(
            package='lidar_apollo_instance_segmentation',
            plugin='lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode',
            name='lidar_apollo',
            remappings=[
                ('input/pointcloud', input_perception),
                ('output/labeled_clusters', '/output_apollo')
            ],
            parameters=[
                {'onnx_file':onnx_file_apollo},
                param_file_apollo
                

            ]
        ))
    
    #Launch shape_estimation
    nodes.append(ComposableNode(
            package='shape_estimation',
            plugin='ShapeEstimationNode',
            name='shape_estimation',
            remappings=[
                ('input', '/output_apollo'),
                ('objects', '/output_shape_estimation')
            ],
            parameters=[param_file_shape_estimation]
        ))
    
    #Launch detection_by_tracker
    nodes.append(ComposableNode(
            package='detection_by_tracker',
            plugin='DetectionByTracker',
            name='detection_by_tracker',
            remappings=[
                ('~/input/tracked_objects', '/output_object_tracker'),
                ('~/input/initial_objects', '/output_shape_estimation'),
                ('~/output', '/output_detection_by_tracker')
            ],
            parameters=[param_file_detection_by_tracker]
        ))
    
    #Launch object_merger
    nodes.append(ComposableNode(
            package='object_merger',
            plugin='object_association::ObjectAssociationMergerNode',
            name='object_merger',
            remappings=[
                ('input/object0', '/output_valid_objects'),
                ('input/object1','/output_detection_by_tracker'),
                ('output/object', '/output_object_merger')
            ],
            parameters=[
                param_file_object_merger,
                distance_threshold_list_path,
                data_association_matrix_path
            ]
        ))
    
    #Launch multi_object_tracker
    nodes.append(ComposableNode(
            package='multi_object_tracker',
            plugin='multi_object_tracker::MultiObjectTracker',
            name='multi_object_tracker',
            remappings=[
                ('output', '/output_object_tracker')
            ],
            parameters=[
                {'selected_input_channels':['detected_objects']},
                config_path_object_tracker + 'multi_object_tracker_node.param.yaml',
                config_path_object_tracker+'data_association_matrix.param.yaml',
                config_path_object_tracker+'input_channels.param.yaml'


            ]
        ))
    
    #Launch Container
     
    container = ComposableNodeContainer(
        name="pointcloud_perception_container",
        namespace='pointcloud_perception',
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return [container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg('pointcloud_topics', "['/pointcloud1', '/pointcloud2']")
    add_launch_arg('input/twist', '/sensing/vehicle_velocity_converter/twist_with_covariance')
    add_launch_arg('input/imu', '/sensing/imu/imu_data')
    add_launch_arg('param_file_distortion_corrector', '$(find-pkg-share pointcloud_preprocessor)/config/distortion_corrector_node.param.yaml')
    add_launch_arg('output_frame', 'base_link')
    add_launch_arg("crop_coord","[-200.0,300.0,-50.0,50.0,-2.0,3.0]")
    add_launch_arg('launch_downsample_node','true')
    add_launch_arg('voxel_size_x')
    add_launch_arg('voxel_size_y')
    add_launch_arg('voxel_size_z')
    add_launch_arg('model_param_path')
    add_launch_arg('ml_package_param_path')
    add_launch_arg('class_remapper_param_path')
    add_launch_arg('model_path')
    add_launch_arg('param_file_ground_segmentation')
    add_launch_arg('param_file_obstacle_validator')
    add_launch_arg('param_file_apollo')
    add_launch_arg('onnx_file_apollo')
    add_launch_arg('param_file_shape_estimation')
    add_launch_arg('param_file_detection_by_tracker')
    add_launch_arg('data_association_matrix_path')
    add_launch_arg('distance_threshold_list_path')
    add_launch_arg('param_file_object_merger')
    add_launch_arg('config_path_object_tracker')

    return launch.LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_setup)]
    )

