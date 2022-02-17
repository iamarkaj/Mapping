import os
import cv2
import rospy
import numpy as np
from datetime import datetime
from sensor_msgs.msg import Image, PointCloud2

from depth import PredictDepth
from layers import BilinearUpSampling2D
from pointcloud import CreatePointCloudMsg


CAMERA_FRAME_ID = "camera_link"
CAMERA_DEPTH_TOPIC = "/camera/depth"
CAMERA_RGB_TOPIC = "/camera/image_raw"

PUBLISH_POINTCLOUD = True
POINTCLOUD_TOPIC = "/point_cloud"

SKIP_FRAMES = 1 # skip n number of frames
VIDEO_PATH = os.path.dirname(os.path.abspath(__file__)) + "/../../Dataset/video.mp4"


class DenseDepth():
    def __init__(self):
        rospy.init_node("DenseDepth", anonymous=True)
        self.video = cv2.VideoCapture(VIDEO_PATH)
        self.depth = PredictDepth()
        
        self.rgb_img_pub = rospy.Publisher(CAMERA_RGB_TOPIC, Image, queue_size=1)
        self.d_img_pub = rospy.Publisher(CAMERA_DEPTH_TOPIC, Image, queue_size=1)

        if PUBLISH_POINTCLOUD:
            self.pc_pub = rospy.Publisher(POINTCLOUD_TOPIC, PointCloud2, queue_size=1)

    def main(self):
        frame_count = 0
        start = datetime.now()

        img_msg = Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.header.frame_id = CAMERA_FRAME_ID
        
        while not rospy.is_shutdown() and self.video.grab():
            frame_count = frame_count + 1
            
            ret, rgb_img = self.video.retrieve()
            
            if not ret: 
                break

            if frame_count % SKIP_FRAMES != 0: 
                continue
            
            print("Frame Count:", frame_count)

            d_img, rgb_img = self.depth.predict(rgb_img)
            
            img_msg.header.stamp = rospy.Time.now()
            img_msg.encoding = "bgr8"
            img_msg.step = 640 * 3
            img_msg.data = np.array(rgb_img, dtype=np.uint8).flatten().tolist()
            self.rgb_img_pub.publish(img_msg)

            img_msg.encoding = "mono8"
            img_msg.step = 640
            img_msg.data = np.array(d_img, dtype=np.uint8).flatten().tolist()
            self.d_img_pub.publish(img_msg)

            if PUBLISH_POINTCLOUD:
                cloud_msg = CreatePointCloudMsg(d_img, rgb_img, rospy.Time.now(), frame_count)
                self.pc_pub.publish(cloud_msg)

        self.video.release()
        print('Time:', datetime.now() - start) 
        return


if __name__ == '__main__':
    a = DenseDepth()
    try:
        a.main()
    except rospy.ROSInterruptException:
        pass