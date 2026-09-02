import glob
import sqlite3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def fetch_lidar_info_from_db():
    """
    Fetches lidar topic names and their corresponding lidar_id from the database.
    Returns a list of dictionaries containing lidar_id and topic name for each lidar.
    """
    db_path = 'src/safety_demo/config/bopt_2000.db'
    
    # # Get the path to the database inside your package (config folder)
    # package_share_directory = get_package_share_directory('ldlidar_stl_ros2')
    # db_path = os.path.join(package_share_directory, 'config', 'amr_policies_01.db')
    
    # if not os.path.exists(db_path):
    #     raise FileNotFoundError(f"The database file was not found at {db_path}")
    

    # Connect to the SQLite database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Query to get lidar_id and topic from the lidar table
    cursor.execute("SELECT lidar_id, topic FROM lidar")

    # Fetch all rows and store them in a list of dictionaries
    lidars = []
    for row in cursor.fetchall():
        lidar = {
            'lidar_id': row[0],
            'topic': row[1]
        }
        lidars.append(lidar)

    # Close the database connection
    conn.close()
    
    return lidars

def generate_launch_description():
    # Fetch LIDAR information (lidar_id and topic name) from the database
    lidars = fetch_lidar_info_from_db()

    # Create a custom mapping dictionary for assigning ports to LIDARs
    # Example: {lidar_id: port_name}
    custom_port_mapping = {
        1: "/dev/Lidar_L",
        2: "/dev/Lidar_R",
        3: "/dev/Lidar_F",
        4: "/dev/Lidar_NF"
    }

    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Iterate over the fetched LIDARs
    for lidar in lidars:
        # Get the port name for this LIDAR based on the custom mapping
        lidar_id = lidar["lidar_id"]
        port_name = custom_port_mapping.get(lidar_id)

        if not port_name:
            # If a port is not mapped for this lidar_id, skip it or handle the error
            print(f"No port assigned for LIDAR with ID {lidar_id}")
            continue

        # Construct a unique node name for each LIDAR based on lidar_id
        node_name = f'LD19_{lidar_id}'
     
        # Generate the frame_id dynamically using the topic name
        frame_id = f'frame_{lidar["topic"]}'

        # Create parameters for the node dynamically
        parameters = {
            'product_name': 'LDLiDAR_LD19',
            'topic_name': lidar['topic'],  # Topic name from the database
            'frame_id': frame_id,          # Dynamically generated frame_id
            'port_name': port_name,        # Custom port_name from the dictionary
            'port_baudrate': 230400
        }

        # Create a Node action for the LIDAR
        node = Node(
            package='ldlidar_stl_ros2',  # Replace with your LIDAR package name
            executable='ldlidar_stl_ros2_node',  # Replace with your node executable
            name=node_name,  # Unique node name based on lidar_id
            output='screen',  # Output to the terminal
            parameters=[parameters],
        )
        
        # Add a static transform publisher for each LiDAR
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_publisher_{lidar_id}',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '0',  # roll, pitch, yaw
                'base_link',          # parent frame (fixed frame in RViz)
                frame_id  # child frame (LiDAR frame)
            ]
        )
        
        # Add both the LiDAR node and the static transform publisher to the launch description
        ld.add_action(node)
        ld.add_action(static_tf)

    return ld
