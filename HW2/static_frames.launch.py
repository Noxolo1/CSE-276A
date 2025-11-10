from launch import LaunchDescription
from launch_ros.actions import Node

def static_tf_q(x, y, z, qx, qy, qz, qw, parent, child, name):
    # creates a static_transform_publisher node in quaternion form
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        # args in quaternion form: x, y, z, qx, qy, qz, qw, parent child
        arguments=[str(x), str(y), str(z), str(qx), str(qy), str(qz), str(qw), parent, child],
        output="screen",
    )

def generate_launch_description():
    nodes = []

    # base_link (ground) to camera_frame 
    nodes.append(static_tf_q(
        0.090, 0.000, 0.124,  # measured x, y, z of camera in relation to robot center (on ground)
        -0.5,  0.5, -0.5,  0.5, 
        "base_link", "camera_frame", "tf_base_to_camera"
    ))

    # tag 0: x = 1m - 10.5in, y = -26.5in, yaw = 90°
    nodes.append(static_tf_q(
        0.73330, -0.67310, 0.1645,
        -0.5000, 0.5000, 0.5000, 0.5000,
        "map", "tag_0", "tf_map_to_tag0"
    ))

    # tag 1: x = 1m + 11.5in, y = 0, yaw = 180°
    nodes.append(static_tf_q(
        1.29210, 0.00000, 0.1645,
        -0.7071, 0.0000, 0.7071, 0.0000,
        "map", "tag_1", "tf_map_to_tag1"
    ))

    # tag 2: x = 1m + 28.5in, y = 2m - 12in, yaw = 180°
    nodes.append(static_tf_q(
        1.72390, 1.69520, 0.1695,
        -0.7071, 0.0000, 0.7071, 0.0000,
        "map", "tag_2", "tf_map_to_tag2"
    ))

    # tag 3: x = 1m, y = 2m + 27in, yaw = 270°
    nodes.append(static_tf_q(
        1.00000, 2.68580, 0.1645,
        -0.5000, -0.5000, 0.5000, -0.5000,
        "map", "tag_3", "tf_map_to_tag3"
    ))

    # tag 4: x = 1m - 47in, y = 2m - 15in, yaw = 0°
    nodes.append(static_tf_q(
        -0.19380, 1.61900, 0.1645,
        0.0000, 0.7071, 0.0000, 0.7071,
        "map", "tag_4", "tf_map_to_tag4"
    ))

    return LaunchDescription(nodes)
