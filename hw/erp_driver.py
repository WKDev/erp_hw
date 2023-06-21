# 230516 chson
# ros2 foxy

import binascii
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
import traceback

from erp_msgs.msg import ErpData


class ErpNode(Node):
    def __init__(self):

        Node.__init__(self,'erp_driver')

        self.declare_parameter('erp_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0')
        self.declare_parameter('erp_baud', '115200')
        
        self.publisher_ = self.create_publisher(ErpData, 'ErpData', 10) # declare publisher
        pub_rate = 100  # seconds, 100hz

        self.lg = self.get_logger() # logger

        self.timer = self.create_timer(1/pub_rate, self.timer_callback)
        self.i = 0

        self.erp_msg = ErpData()

        self.t = threading.Thread(None, self.ser_thread, daemon=True)

        self.t.start()


    def timer_callback(self):
        # self.lg.error('publishing....')
        self.publisher_.publish(self.erp_msg)

    def ser_thread(self):
        while True:
            port = self.get_parameter('erp_port').get_parameter_value().string_value
            baud = self.get_parameter('erp_baud').get_parameter_value().string_value

            try:
                self.lg.debug('erp_node')
                ser = serial.Serial(port, baud)
                self.lg.info('trying to connect ERP')

                # try:
                #     data = ser.readline()
                #     self.lg.info('ERP OK')
                # except:
                #     self.lg.error('ERP FAILED')

                while True:
                    # Read data from the serial port

                    data = ser.readline()

                    # self.lg.info(f'init_data : {data}')

                    # total length of datas should be 12.

                    try:
                        ## TODO : there seems a few problems to fix(i thinkg m_or_a is not working) 230620chson
                        # d
                        hex_string = binascii.hexlify(data).decode('utf-8')
                        pair_list = [hex_string[i:i+2] for i in range(0, len(hex_string), 2)][3:-2]

                        if len(pair_list)>8:
                            m_or_a = int(pair_list[0],16)
                            e_stop = int(pair_list[1],16)
                            gear = int(pair_list[2],16)
                            speed = float(int(''.join(pair_list[3:4][::-1]),16)*10)
                            steer = int(''.join(pair_list[5:7][::-1]),16)

                            if steer > 30000:
                                steer = steer - 65536

                            steer = -steer/71
                            brake = int(pair_list[7],16)
                            enc = int(''.join(pair_list[8:-1][::-1]),16)
                            alive = int(pair_list[-1],16)


                            if enc > 2147483647:
                                enc = enc-4294967286

                            gear_state = 'D' if gear ==0 else 'N' if gear ==1 else 'R'

                            print(enc)
                            
                            # print(f'{m_or_a=} {e_stop=} {gear=}({gear_state}) {speed=} {steer=:.2f} {brake=} {enc=} {alive=}')

                            self.erp_msg.alive = alive
                            self.erp_msg.brake = brake
                            self.erp_msg.gear = gear
                            self.erp_msg.m_or_a = m_or_a
                            self.erp_msg.e_stop = e_stop
                            self.erp_msg.speed = speed
                            self.erp_msg.steer = steer
                            self.erp_msg.encoder = enc

                            # self._publisher.publish(self.erp_msg)

                       
                    except:
                        self.lg.warn(f'{traceback.format_exc()}')
                        continue


            except serial.SerialException:
                self.lg.warn(f'Unable to connect to serial port:{port}, baud:{baud}, retrying...')
                time.sleep(1.0)        

            

def main(args=None):
    rclpy.init(args=args)

    erp_node = ErpNode()

    rclpy.spin(erp_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    erp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
