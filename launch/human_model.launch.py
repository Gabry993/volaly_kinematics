import os
from typing import List
import yaml

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def urdf(tf_prefix: str = '') -> str:
    urdf_xacro = os.path.join(get_package_share_directory('volaly_kinematics'),
                              'urdf', 'human_kinematics.urdf.xacro')
    xacro_keys = [k for k, _ in urdf.__annotations__.items() if k not in ('return', 'biometrics')]
    kwargs = dict(locals())
    xacro_args = [f'{arg_name}:={kwargs.get(arg_name)}' for arg_name in xacro_keys]
    opts, input_file_name = xacro.process_args([urdf_xacro] + xacro_args)
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
    except Exception as e:
        raise(e)
    return doc.toprettyxml(indent='  ')


def robot_state_publisher(context: LaunchContext,
                          **substitutions: launch.substitutions.LaunchConfiguration
                          ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}
    namespace = kwargs.pop('namespace')
    params = {'robot_description': urdf(**kwargs)}
    node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[params], output='screen')
    return [node]


def joint_state_publisher(context: LaunchContext,
                          **substitutions: launch.substitutions.LaunchConfiguration
                          ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}
    params = {
        'rate': float(kwargs['rate']),
        'source_list': [kwargs['imu_joint_states_topic']]
    }
    use_gui = kwargs['use_gui'] in ('true', 'True')
    tf_prefix = kwargs['tf_prefix']
    try:
        with open(kwargs['biometrics'], 'r') as f:
            data = yaml.safe_load(f)
            params.update({f'zeros.{tf_prefix}{k}': v for k, v in data.items()})
    except FileNotFoundError:
        pass
    name = 'joint_state_publisher'
    if use_gui:
        name += "_gui"
    print(params)
    print(use_gui)
    print('tf_prefix', tf_prefix)
    node = Node(
        package=name,
        executable=name,
        name='joint_state_publisher',
        namespace=kwargs['namespace'],
        parameters=[params], output='screen')
    return [node]


def generate_launch_description() -> None:
    arguments = [
        launch.actions.DeclareLaunchArgument(
            k, default_value=str(urdf.__defaults__[i]), description='')
        for i, (k, _) in enumerate(urdf.__annotations__.items()) if k != 'return']
    kwargs = {k: launch.substitutions.LaunchConfiguration(k)
              for (k, _) in urdf.__annotations__.items() if k != 'return'}
    arguments.append(launch.actions.DeclareLaunchArgument(
        'namespace', default_value='', description=''))
    arguments.append(launch.actions.DeclareLaunchArgument(
        'biometrics', default_value='', description=''))
    arguments.append(launch.actions.DeclareLaunchArgument(
        'use_gui', default_value=str(False), description=''))
    arguments.append(launch.actions.DeclareLaunchArgument(
        'rate', default_value=str(50.0), description=''))
    arguments.append(launch.actions.DeclareLaunchArgument(
        'imu_joint_states_topic', default_value='', description=''))
    kwargs['namespace'] = launch.substitutions.LaunchConfiguration('namespace')
    joint_state_kwargs = {
        k: launch.substitutions.LaunchConfiguration(k)
        for k in ('tf_prefix', 'use_gui', 'namespace', 'biometrics', 'rate',
                  'imu_joint_states_topic')
    }
    return LaunchDescription(
        arguments + [
            launch.actions.OpaqueFunction(
                function=robot_state_publisher,
                kwargs=kwargs),
            launch.actions.OpaqueFunction(
                function=joint_state_publisher,
                kwargs=joint_state_kwargs)
        ])
