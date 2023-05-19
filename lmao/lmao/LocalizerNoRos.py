import rclpy

from Localizer import Localizer
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication
 

def main(args=None):
    rclpy.init(args=args)
    #QApp = QApplication([])
    #QWidget = QWidget()
    localizer = Localizer(ros=False, fname="/home/junge/master_ws/src/Danitech-master/lmao/lmao/localizer_positions3.pkl")
    bag_file = "/home/junge/Documents/bags/island_boy_to_rule_them_all" #QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")

    time = 0

    with Reader(bag_file) as reader:
        print("We in da bag")
        #for connection in reader.connections:
            #print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/wagon/base_scan/lidar_data":
                msg = deserialize_cdr(rawdata, connection.msgtype)
                localizer.lidar_callback(msg)
                localizer.viz_loop()
            elif connection.topic == "/wagon/base_link_odom_gt":
                msg = deserialize_cdr(rawdata, connection.msgtype)

                time1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if time1 - time > 1:
                    print(time1)
                    time = time1
                localizer.odom_callback(msg)
            elif connection.topic == "/cmd_vel":
                msg = deserialize_cdr(rawdata, connection.msgtype)
                localizer.cmd_vel_callback(msg)


if __name__ == "__main__":
    main()