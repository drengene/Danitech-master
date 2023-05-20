from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pickle

def load_from_bag(bag_file, topic):
    gt_positions = []
    with Reader(bag_file) as reader:
        #for connection in reader.connections:
            #print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                #print(msg.header.frame_id)
                #print(msg.pose.pose.position.x)
                gt_positions.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    return gt_positions

def main():
    bagfile = "/home/danitech/Documents/bags/island_boy_2"
    topic = "/wagon/base_link_pose_gt"
    gt_positions = load_from_bag(bagfile, topic)
    # Save to pickle
    with open(bagfile + '_gt.pkl', 'wb') as f:
        pickle.dump(gt_positions, f)


if __name__ == '__main__':
    main()
