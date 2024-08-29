import os, sys, argparse
import numpy as np
import xml.etree.ElementTree as ET
import pykitti, cv2
import math
from datetime import datetime
from tqdm import tqdm

import rospy, rosbag, tf
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    calib_data = {}

    for laser in root.findall('laser'):
        laser_id = laser.get('id')
        yaw_deg = laser.find('yaw_rotation_deg').text
        pitch_deg = laser.find('pitch_rotation_deg').text
        line_coeff_a = laser.find('beam_line_coefficient_a').text
        line_coeff_b = laser.find('beam_line_coefficient_b').text

        calib_data[int(laser_id)] = {
            'yaw_rotation_deg': float(yaw_deg),
            'pitch_rotation_deg': float(pitch_deg),
            'beam_line_coefficient_a': float(line_coeff_a),
            'beam_line_coefficient_b': float(line_coeff_b)
        }

    return calib_data

def get_raw_frame_info_by_sequence(sequence):
    date = ''
    drive = ''
    start_frame = 0
    end_frame = 0
    if sequence == '00':
        date = '2011_10_03'
        drive = '0027'
        start_frame = 0
        end_frame = 4540
    elif sequence == '01':
        date = '2011_10_03'
        drive = '0042'
        start_frame = 0
        end_frame = 1100
    elif sequence == '02':
        date = '2011_10_03'
        drive = '0034'
        start_frame = 0
        end_frame = 4660
    elif sequence == '03':
        date = '2011_09_26'
        drive = '0067'
        start_frame = 0
        end_frame = 800
    elif sequence == '04':
        date = '2011_09_30'
        drive = '0016'
        start_frame = 0
        end_frame = 270
    elif sequence == '05':
        date = '2011_09_30'
        drive = '0018'
        start_frame = 0
        end_frame = 2760
    elif sequence == '06':
        date = '2011_09_30'
        drive = '0020'
        start_frame = 0
        end_frame = 1100
    elif sequence == '07':
        date = '2011_09_30'
        drive = '0027'
        start_frame = 0
        end_frame = 1100
    elif sequence == '08':
        date = '2011_09_30'
        drive = '0028'
        start_frame = 1100
        end_frame = 5170
    elif sequence == '09':
        date = '2011_09_30'
        drive = '0033'
        start_frame = 0
        end_frame = 1590
    elif sequence == '10':
        date = '2011_09_30'
        drive = '0034'
        start_frame = 0
        end_frame = 1200
        
    return date, drive, start_frame, end_frame

def save_imu_data_raw(bag, start_frame_time, end_frame_time, kitti, imu_frame_id, topic):
    print("Exporting IMU Raw")
    synced_path = kitti.data_path
    unsynced_path = synced_path.replace('sync', 'extract')
    unsynced_imu_path = os.path.join(unsynced_path, 'oxts')
    # read time stamp (convert to ros seconds format)
    with open(os.path.join(unsynced_imu_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        imu_datetimes = []
        count = 0
        start_frame = 0
        end_frame = 0
        start_init_flag = False
        end_init_flag = False
        for line in lines:
            if len(line) == 1:
                continue
            
            timestamp_tmp = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            timestamp = float(timestamp_tmp.strftime("%s.%f"))
               
            if timestamp >= start_frame_time and start_init_flag == False:
                start_frame = count
                start_init_flag = True
            if timestamp > end_frame_time and end_init_flag == False:
                end_frame = count - 1
                end_init_flag = True
                break

            count += 1
            if timestamp >= start_frame_time and timestamp <= end_frame_time:
                imu_datetimes.append(timestamp)
                
        if end_init_flag == False:
            end_frame = count - 1
            end_init_flag = True

    # get all imu data
    imu_data_dir = os.path.join(unsynced_imu_path, 'data')
    imu_filenames = sorted(os.listdir(imu_data_dir))
    imu_data = [None] * len(imu_datetimes)

    imu_filenames = imu_filenames[start_frame:end_frame+1]
    for i, imu_file in enumerate(imu_filenames):
        imu_data_file = open(os.path.join(imu_data_dir, imu_file), "r")
        for line in imu_data_file:
            if len(line) == 1:
                continue
            stripped_line = line.strip()
            line_list = stripped_line.split()
            imu_data[i] = line_list

    assert len(imu_datetimes) == len(imu_data)
    
    iterable = zip(imu_datetimes, imu_data)
    for timestamp, data in tqdm(iterable, total=len(imu_datetimes)):
        roll, pitch, yaw = float(data[3]), float(data[4]), float(data[5])
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(timestamp)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = float(data[11])
        imu.linear_acceleration.y = float(data[12])
        imu.linear_acceleration.z = float(data[13])
        imu.angular_velocity.x = float(data[17])
        imu.angular_velocity.y = float(data[18])
        imu.angular_velocity.z = float(data[19])
        bag.write(topic, imu, t=imu.header.stamp)

def save_gps_fix_data(bag, start_frame, end_frame, kitti, gps_frame_id, topic):
    timestamps = kitti.timestamps[start_frame:end_frame+1]
    oxts = kitti.oxts[start_frame:end_frame+1]
    for timestamp, oxts in zip(timestamps, oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)

def save_gps_vel_data(bag, start_frame, end_frame, kitti, gps_frame_id, topic):
    timestamps = kitti.timestamps[start_frame:end_frame+1]
    oxts = kitti.oxts[start_frame:end_frame+1]
    for timestamp, oxts in zip(timestamps, oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


def read_calib_file(filepath):

    calib_file = open(filepath)
    calib = {}
    for line in calib_file:
        key, content = line.strip().split(":")
        values = [float(v) for v in content.strip().split()]
        pose = np.zeros((4,4))
        
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        calib[key] = pose

    calib_file.close()
    
    return calib


def read_poses_file(filepath, calibration):
    pose_file = open(filepath)

    poses = []


    Tr = calibration["Tr"]
    Tr_inv = np.linalg.inv(Tr)

    for line in pose_file:
        values = [float(v) for v in line.strip().split()]

        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        poses.append(np.matmul(Tr_inv, np.matmul(pose, Tr)))

    pose_file.close()
    return poses

def save_pose_msg(bag, start_frame_time, semantic_data_path, poses, frame_id, child_frame_id, topic):
    print("Exporting pose msg")

    times_file = os.path.join(semantic_data_path, 'times.txt')
    times = []
    with open(times_file, 'r') as f:
        for line in f.readlines():
            #number = datetime.fromtimestamp(float(line))
            number = float(line) + start_frame_time
            if number == 0.0:
                number = 0.0001
            times.append(number)

    prev_pose = PoseStamped()
    prev_time = 0.00
    count = 0
    for time in tqdm(times, total=len(times)):
        count += 1
        pose_mat = poses[count - 1]
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.from_sec(time)

        t = pose_mat[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(pose_mat)

        pose.pose.position.x = t[0]
        pose.pose.position.y = t[1]
        pose.pose.position.z = t[2]

        q_n = q / np.linalg.norm(q)

        pose.pose.orientation.x = q_n[0]
        pose.pose.orientation.y = q_n[1]
        pose.pose.orientation.z = q_n[2]
        pose.pose.orientation.w = q_n[3]

        bag.write(topic, pose, t=pose.header.stamp)

        delta_time = (time - prev_time)
        if(count == 1):
            delta_time = 0.1
            prev_pose = pose
        
        vx = (pose.pose.position.x - prev_pose.pose.position.x )/delta_time
        vy = (pose.pose.position.y - prev_pose.pose.position.y )/delta_time
        vz = (pose.pose.position.z - prev_pose.pose.position.z )/delta_time

        vqx = (pose.pose.orientation.x - prev_pose.pose.orientation.x)
        vqy = (pose.pose.orientation.y - prev_pose.pose.orientation.y)
        vqz = (pose.pose.orientation.z - prev_pose.pose.orientation.z)
        vqw = (pose.pose.orientation.w - prev_pose.pose.orientation.w)
  
        v_roll = math.atan2( 2*(vqw*vqx + vqy*vqz), 1-2*(vqx**2 + vqy**2)  )/delta_time
        v_pitch = math.asin( 2*(vqw*vqy - vqz*vqx) )/delta_time
        v_yaw = math.atan2( 2*(vqw*vqz + vqx*vqy) , 1-2*(vqy**2 + vqz**2)  )/delta_time

        odom = Odometry()
        odom.header.stamp = pose.header.stamp
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id

        odom.pose.pose.position = pose.pose.position
        odom.pose.pose.orientation = pose.pose.orientation
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        
        
        odom.twist.twist.angular.x = v_roll
        odom.twist.twist.angular.y = v_pitch
        odom.twist.twist.angular.z = v_yaw

        bag.write(topic + '/odom', odom, t=odom.header.stamp)
    
        prev_pose = pose
        prev_time = time

def save_camera_data_match(bag, start_frame, end_frame, kitti, util, bridge, camera, camera_frame_id, topic):
    print("Exporting camera {}".format(camera))
    camera_pad = '{0:02d}'.format(camera)
    image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
    image_path = os.path.join(image_dir, 'data')
    image_filenames = sorted(os.listdir(image_path))
    with open(os.path.join(image_dir, 'timestamps.txt')) as f:
        image_datetimes = list(map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines()))
    
    calib = CameraInfo()
    calib.header.frame_id = camera_frame_id
    calib.width, calib.height = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
    calib.distortion_model = 'plumb_bob'
    calib.K = util['K_{}'.format(camera_pad)]
    calib.R = util['R_rect_{}'.format(camera_pad)]
    calib.D = util['D_{}'.format(camera_pad)]
    calib.P = util['P_rect_{}'.format(camera_pad)]
            
    image_datetimes = image_datetimes[start_frame:end_frame + 1]
    image_filenames = image_filenames[start_frame:end_frame + 1]
    iterable = zip(image_datetimes, image_filenames)
    for time, filename in tqdm(iterable, total=len(image_filenames)):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = cv_image.shape[:2]
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8" if camera in (0, 1) else "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id

        image_message.header.stamp = rospy.Time.from_sec(float(datetime.strftime(time, "%s.%f")))
        topic_ext = "/image_raw"

        calib.header.stamp = image_message.header.stamp
        bag.write(topic + topic_ext, image_message, t = image_message.header.stamp)
        bag.write(topic + '/camera_info', calib, t = calib.header.stamp) 


def get_rgb(sem_id):
    RGB_id = 0
    if sem_id==0:
        RGB_id = 0x000000
    elif sem_id==1:
        RGB_id = 0xff0000
    elif sem_id==10:
        RGB_id = 0x6496f5
    elif sem_id==11:
        RGB_id = 0x64e6f5
    elif sem_id==13:
        RGB_id = 0x6450fa
    elif sem_id==15:
        RGB_id = 0x1e3c96
    elif sem_id==16:
        RGB_id = 0x0000ff
    elif sem_id==18:
        RGB_id = 0x501eb4
    elif sem_id==20:
        RGB_id = 0x0000ff
    elif sem_id==30:
        RGB_id = 0xff1e1e
    elif sem_id==31:
        RGB_id = 0xff28c8
    elif sem_id==32:
        RGB_id = 0x961e5a
    elif sem_id==40:
        RGB_id = 0xff00ff
    elif sem_id==44:
        RGB_id = 0xff96ff
    elif sem_id==48:
        RGB_id = 0x4b004b
    elif sem_id==49:
        RGB_id = 0xaf004b
    elif sem_id==50:
        RGB_id = 0xffc800
    elif sem_id==51:
        RGB_id = 0xff7832
    elif sem_id==52:
        RGB_id = 0xff9600
    elif sem_id==60:
        RGB_id = 0x96ffaa
    elif sem_id==70:
        RGB_id = 0x00af00
    elif sem_id==71:
        RGB_id = 0x873c00
    elif sem_id==72:
        RGB_id = 0x96f050
    elif sem_id==80:
        RGB_id = 0xfff096
    elif sem_id==81:
        RGB_id = 0xff0000
    elif sem_id==99:
        RGB_id = 0x32ffff
    elif sem_id==252:
        RGB_id = 0x6496f5
    elif sem_id==253:
        RGB_id = 0xff28c8
    elif sem_id==254:
        RGB_id = 0xff1e1e
    elif sem_id==255:
        RGB_id = 0x961e5a
    elif sem_id==256:
        RGB_id = 0x0000ff
    elif sem_id==257:
        RGB_id = 0x6450fa
    elif sem_id==258:
        RGB_id = 0x501eb4
    elif sem_id==259:
        RGB_id = 0x0000ff
    else:
        RGB_id = 0x000000

    return RGB_id

def convertdata(labelscan):
    
    semantic_id = []
    rgb_id = []

    for counting in range(len(labelscan)):
        sem_id = int(labelscan[counting]) & 0xFFFF
        rgb = get_rgb(sem_id)

        semantic_id.append(sem_id)
        rgb_id.append(rgb)
    
    return semantic_id, rgb_id

def save_velo_data_with_time_ring_label(bag, lidar_calib, raw_scan_data_path, label_data_path, start_frame, end_frame, velo_frame_id, velo_topic):
    print("Exporting Velodyne and Label data")

    ## Get lidar, label, timestamp file names
    velo_data_path = os.path.join(raw_scan_data_path, 'velodyne_points/data')
    velo_filenames = sorted(os.listdir(velo_data_path))

    label_data_path = os.path.join(label_data_path, 'labels')
    label_filenames = sorted(os.listdir(label_data_path))

    raw_data_timestamp_start_file = os.path.join(raw_scan_data_path, 'velodyne_points/timestamps_start.txt')
    raw_data_timestamp_end_file = os.path.join(raw_scan_data_path, 'velodyne_points/timestamps_end.txt')

    ## Parse time stamp
    with open(raw_data_timestamp_end_file) as f:
        lines = f.readlines()
        timestamps_end = []
        for line in lines:
            if len(line) == 1:
                continue
            time = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            timestamps_end.append(time)

    with open(raw_data_timestamp_start_file) as f:
        lines = f.readlines()
        timestamps_start = []
        for line in lines:
            if len(line) == 1:
                continue
            time = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            timestamps_start.append(time)

    ## Set data scope
    timestamps_end = timestamps_end[start_frame:end_frame+1]
    timestamps_start = timestamps_start[start_frame:end_frame+1]
    velo_filenames = velo_filenames[start_frame:end_frame+1]

    iterable = zip(timestamps_start, timestamps_end, velo_filenames, label_filenames)

    frame_idx = -1
    for start_time, end_time, veloname, labelname in tqdm(iterable, total=len(velo_filenames)):
        frame_idx += 1

        ### Read binary data
        velo_filename = os.path.join(velo_data_path, veloname)
        points_xyzi = np.fromfile(velo_filename, dtype=np.float32).reshape(-1, 4)
        label_filename = os.path.join(label_data_path, labelname)
        label_data = np.fromfile(label_filename, dtype=np.int32)
        points_label, points_rgb = convertdata(label_data)
        points_label = np.array(points_label)
        points_rgb = np.array(points_rgb)
        
        ### Compute ring
        ## Laser line fitting method
        ## - compute ring by compute projected point to each channel layser line distasnce  
        ## - to get each_ring_line_coeff (layser line), points ring segmentation (using point pitch correction method) is needed  
        horizontal_dist = np.linalg.norm(points_xyzi[:,0:2], ord=2, axis=1)
        height = points_xyzi[:,2]
        point_to_line_dist = np.zeros([len(points_xyzi), 64])
        for i in range(64):
            point_to_line_dist[:,i] = abs(lidar_calib[i]['beam_line_coefficient_a']*horizontal_dist - height + lidar_calib[i]['beam_line_coefficient_b']) / math.sqrt(lidar_calib[i]['beam_line_coefficient_a']**2 + 1)
        points_ring = np.argmin(point_to_line_dist, axis=1)

        ### Compute time
        points_time = np.zeros((len(points_xyzi)))
        points_index = np.arange(0, len(points_xyzi), 1)
        points_frame = np.full((len(points_xyzi)), frame_idx)
        start_time_float = float(datetime.strftime(start_time, "%s.%f"))
        end_time_float = float(datetime.strftime(end_time, "%s.%f"))
        delta_time = end_time_float - start_time_float
        for point_idx in range(len(points_xyzi)):
            point_horizontal_angle_deg = np.arctan2(points_xyzi[point_idx][1], points_xyzi[point_idx][0]) * 180. / math.pi
            yaw_correction = lidar_calib[points_ring[point_idx]]['yaw_rotation_deg']
            point_lidar_rotation_angle_deg = -point_horizontal_angle_deg + 180.0 + yaw_correction
            
            if point_lidar_rotation_angle_deg < 0:
                point_lidar_rotation_angle_deg += 360.0
            elif point_lidar_rotation_angle_deg > 360.0:
                point_lidar_rotation_angle_deg -= 360.0

            points_time[point_idx] = (point_lidar_rotation_angle_deg / 360.0) * delta_time


        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32),
            ('rgb', np.uint32),
            ('label', np.uint16),
            ('point_index', np.uint32),
            ('frame_idx', np.uint16)
        ])

        # 데이터를 Structured array로 변환
        scanframes = np.zeros(points_xyzi.shape[0], dtype=dtype)
        scanframes['x'] = points_xyzi[:, 0]
        scanframes['y'] = points_xyzi[:, 1]
        scanframes['z'] = points_xyzi[:, 2]
        scanframes['intensity'] = points_xyzi[:, 3]
        scanframes['ring'] = points_ring
        scanframes['time'] = points_time
        scanframes['rgb'] = points_rgb
        scanframes['label'] = points_label
        scanframes['point_index'] = points_index
        scanframes['frame_idx'] = points_frame

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(start_time, "%s.%f")))

        # fill pcl msg
        fields =[PointField('x',  0, PointField.FLOAT32, 1),
                 PointField('y',  4, PointField.FLOAT32, 1),
                 PointField('z',  8, PointField.FLOAT32, 1),
                 PointField('intensity', 12, PointField.FLOAT32, 1),
                 PointField('ring', 16, PointField.UINT16, 1),
                 PointField('time', 18, PointField.FLOAT32, 1),
                 PointField('rgb', 22, PointField.UINT32, 1),
                 PointField('label', 26, PointField.UINT16, 1),
                 PointField('point_index', 28, PointField.UINT32, 1),
                 PointField('frame_idx', 32, PointField.UINT16, 1)]

        pcl_msg = sensor_msgs.point_cloud2.create_cloud(header, fields, scanframes)
        pcl_msg.is_dense = True

        bag.write(velo_topic, pcl_msg, t=pcl_msg.header.stamp)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert KITTI Raw to ROS bag with point time, ring data.')
    parser.add_argument('--calib', type=str, help='lidar calibration .xml file')
    parser.add_argument('--raw_data', type=str, help='KITTI Raw data path')
    parser.add_argument('--semantic_data', type=str, help='SemanticKITTI data path')
    parser.add_argument('--sequence', type=str, help='sequence number (00~10)')
    args = parser.parse_args()
    sequence = "{:02}".format(int(args.sequence))
    lidar_calib = parse_xml_file(args.calib)
    
    date, drive, start_frame, end_frame = get_raw_frame_info_by_sequence(sequence)
    
    kitti_raw = pykitti.raw(args.raw_data, date, drive)
    if not os.path.exists(kitti_raw.data_path):
        print('Path {} does not exists. Exiting.'.format(kitti_raw.data_path))
        sys.exit(1)

    print(date, drive, start_frame, end_frame)

    raw_sequence_data_path = kitti_raw.data_path
    semantic_sequence_data_path = os.path.join(args.semantic_data, sequence)

    
    bag_name = "kitti_raw_sequence_{}_date_{}_drive_{}.bag".format(sequence, date, drive)
    bag_write_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), bag_name)
    compression = rosbag.Compression.NONE
    bag = rosbag.Bag(bag_write_path, 'w', compression=compression)
    

        
    start_frame_time = float(kitti_raw.timestamps[start_frame].strftime("%s.%f"))
    end_frame_time = float(kitti_raw.timestamps[end_frame].strftime("%s.%f"))

    ## Set frame id & topic
    imu_frame_id = 'imu_link'
    imu_topic = '/kitti/oxts/imu'
    imu_raw_topic = '/imu_raw'
    gps_fix_topic = '/gps/fix'
    gps_vel_topic = '/gps/vel'
    velo_frame_id = 'velodyne'
    velo_topic = '/velodyne_points'
    world_frame_id = 'map'
    suma_pose_frame_id = 'suma_pose'
    suma_pose_topic = '/suma_pose'
    # gt_pose_frame_id = 'gt_pose'
    # gt_pose_topic = '/gt_pose'
    
    cameras = [(0, 'camera_gray_left', '/kitti/camera_gray_left'),
            (1, 'camera_gray_right', '/kitti/camera_gray_right'),
            (2, 'camera_color_left', '/kitti/camera_color_left'),
            (3, 'camera_color_right', '/kitti/camera_color_right')]
    
    try:
        # IMU data processing
        save_imu_data_raw(bag, start_frame_time - 1.0, end_frame_time + 1.0, kitti_raw, imu_frame_id, imu_raw_topic)
        
        # # GPS data processing
        save_gps_fix_data(bag, start_frame, end_frame, kitti_raw, imu_frame_id, gps_fix_topic)
        save_gps_vel_data(bag, start_frame, end_frame, kitti_raw, imu_frame_id, gps_vel_topic)

        ## Pose data processing (Data reference time is unknown)
        sensor_calib = read_calib_file(os.path.join(semantic_sequence_data_path, 'calib.txt'))
        suma_poses = read_poses_file(os.path.join(semantic_sequence_data_path, "poses.txt"), sensor_calib) # SuMA Odometry
        save_pose_msg(bag, start_frame_time, semantic_sequence_data_path, suma_poses, world_frame_id, suma_pose_frame_id, suma_pose_topic)

        ## Camera data processing
        bridge = CvBridge()
        cam_calib = pykitti.utils.read_calib_file(os.path.join(kitti_raw.calib_path, 'calib_cam_to_cam.txt'))
        # for camera in cameras:
        #     save_camera_data_match(bag, start_frame, end_frame, kitti_raw, cam_calib, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2])
        camera = cameras[2] # Use just one camera image to reduce file size
        save_camera_data_match(bag, start_frame, end_frame, kitti_raw, cam_calib, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2])

        ## Lidar data processing
        save_velo_data_with_time_ring_label(bag, lidar_calib, raw_sequence_data_path, semantic_sequence_data_path, start_frame, end_frame, velo_frame_id, velo_topic)

    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()
            
