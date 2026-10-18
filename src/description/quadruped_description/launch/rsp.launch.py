import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

MOUNT_ARGS = [p + a for p in ("imu_", "lidar_", "cam_") for a in ("x", "y", "z", "roll", "pitch", "yaw")]
CAM_DEFAULTS = {"cam_roll": "-1.5707963", "cam_yaw": "-1.5707963"}


def rpy_to_matrix(r, p, y):
    """URDF fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_to_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def inverse_mount(x, y, z, r, p, yw):
    """T_body<-base for T_base<-imu = (t, R): (-R^T t, R^T); body is the IMU frame."""
    m = rpy_to_matrix(r, p, yw)
    rt = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-(rt[i][0] * x + rt[i][1] * y + rt[i][2] * z) for i in range(3)]
    return t, matrix_to_rpy(rt)


def _rsp(context):
    pkg_share = get_package_share_directory('quadruped_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'quadruped_sensors.urdf.xacro')
    mounts = {k: LaunchConfiguration(k).perform(context) for k in MOUNT_ARGS}
    imu = [float(mounts['imu_' + a]) for a in ("x", "y", "z", "roll", "pitch", "yaw")]
    t, rpy = inverse_mount(*imu)
    mappings = dict(mounts)
    mappings['body_to_base_xyz'] = ' '.join('%.9f' % v for v in t)
    mappings['body_to_base_rpy'] = ' '.join('%.9f' % v for v in rpy)
    doc = xacro.process_file(xacro_file, mappings=mappings)
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc.toxml(),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=CAM_DEFAULTS.get(k, '0.0'),
                                  description='sensor mount relative to base_link [m / rad]')
            for k in MOUNT_ARGS]
    args.append(DeclareLaunchArgument('use_sim_time', default_value='false'))
    return LaunchDescription(args + [OpaqueFunction(function=_rsp)])
