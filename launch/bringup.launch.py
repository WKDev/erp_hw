from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([

        # # ERP
        # launch_ros.actions.Node(
        #     namespace= "erp", package='hw', executable='erp_driver', output='screen',
        #     parameters=[
        #     # {"erp_port": '/dev/ttyUSB0'},
        #     {"erp_port": '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'},
        #     {"erp_baud": '115200'},
        #     ]),

        ## IMU
        launch_ros.actions.Node(
            namespace= "", package='hw', executable='imu_driver', output='screen',
            parameters=[
            # {"imu_port": '/dev/ttyUSB0'},
            {"imu_port": '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0001-if00-port0'},
            {"imu_baud": '921600'},
            {"is_launch": True},
            ]),

        ## GPS

        # launch_ros.actions.Node(
        #     namespace= "erp", package='nmea_navsat_driver', executable='erp_driver', output='screen',
        #     parameters=[
        #     {"port": '/dev/ttyUSB0'},
        #     {"baud": '115200'},
        #     ])

        ## Odometry Calculation
        # launch_ros.actions.Node(
        #     namespace= "", package='hw', executable='odometry_manager', output='screen',
        #     # parameters=[
        #     # {"port": '/dev/ttyUSB0'},
        #     # {"baud": '115200'},
        #     # ]
        #     )

    ])