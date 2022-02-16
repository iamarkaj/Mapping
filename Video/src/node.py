#!/usr/bin/env python

"""
1. Converts RGB to RGBD
2. Save the result  or/and
3. Publish the result
"""

CREATE_DATASET = True
DATASET_PATH = "tmp/Dataset" 
ASSOCIATE_PATH = "tmp/associate.txt"

PUBLISH = False
CAMERA_FRAME_ID = "camera_link"
CAMERA_DEPTH_TOPIC = "/camera/depth"
CAMERA_RGB_TOPIC = "/camera/image_raw"

SKIP_FRAMES = 2 # skip n number of frames
CAMERA_FPS = 30
BATCH_SIZE = 2
MIN_DEPTH = 10
MAX_DEPTH = 1000
MODEL_PATH = "Video/model/nyu.h5"
VIDEO_PATH = "Video/data/video.mp4"

import sys
import os
import shutil
import numpy as np
from datetime import datetime
from sensor_msgs.msg import Image
from skimage.transform import resize
from predict_depth import PredictDepth

# Incase of errors while importing opencv, uncomment the below 2 comments

# sys.path.remove("/home/$USER/catkin_ws/devel/lib/python2.7/dist-packages")
import cv2

# sys.path.append("/usr/lib/python2.7/dist-packages")
import rospy

class Video:
    def __init__(self):
        rospy.init_node("video", anonymous=True)
        self.video = cv2.VideoCapture(VIDEO_PATH)
        self.depth = PredictDepth(min_depth=MIN_DEPTH, max_depth=MAX_DEPTH, batch_size=BATCH_SIZE, model_path=MODEL_PATH)

        print("PUBLISHING:", PUBLISH)
        print("CREATE DATASET:", CREATE_DATASET)

        # For publishing ==============================
        if PUBLISH:
            self.rgb_img_pub = rospy.Publisher(CAMERA_RGB_TOPIC, Image, queue_size=1)
            self.d_img_pub = rospy.Publisher(CAMERA_DEPTH_TOPIC, Image, queue_size=1)
        #==============================================

        # For dataset ================================
        if CREATE_DATASET:
            with open(ASSOCIATE_PATH, 'w') as f: pass
            if os.path.exists(DATASET_PATH + '/depth'): shutil.rmtree(DATASET_PATH + '/depth')
            if os.path.exists(DATASET_PATH + '/rgb'): shutil.rmtree(DATASET_PATH + '/rgb')
            os.makedirs(DATASET_PATH + '/depth')
            os.makedirs(DATASET_PATH + '/rgb')
        #==============================================

    def main(self):
        data = ""
        timestamp = 1/CAMERA_FPS
        frame_count = 0
        start = datetime.now()
        
        while not rospy.is_shutdown() and self.video.grab():
            frame_count = frame_count + 1
            
            ret, rgb_img = self.video.retrieve()
            if not ret: break

            if frame_count % SKIP_FRAMES != 0: continue
            print("Frame Count:", frame_count)

            shape = (480, 640)
            rgb_img = cv2.resize(rgb_img, shape)
            d_img = self.depth.predict(rgb_img) * 255
            d_img = cv2.resize(d_img, shape)
            
            # For publishing ==============================
            if PUBLISH:
                img_msg = Image()
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = CAMERA_FRAME_ID

                img_msg.encoding = "bgr8"
                img_msg.width = rgb_img.shape[1]
                img_msg.height = rgb_img.shape[0]
                img_msg.step = rgb_img.shape[1] * rgb_img.shape[2]
                rgb_img_stream = np.array(rgb_img, dtype=np.uint8).flatten().tolist()
                img_msg.data = rgb_img_stream
                self.rgb_img_pub.publish(img_msg)

                img_msg.encoding = "mono8"
                img_msg.width = d_img.shape[1]
                img_msg.height = d_img.shape[0]
                img_msg.step = d_img.shape[1]
                d_img_stream = np.array(d_img, dtype=np.uint8).flatten().tolist()
                img_msg.data = d_img_stream
                self.d_img_pub.publish(img_msg)
            #==============================================

            # For dataset ================================
            if CREATE_DATASET:
                timestamp = timestamp + 0.04
                data = data + "{:6f} rgb/{:6f}.png {:6f} depth/{:6f}.png\n".format(timestamp,timestamp,timestamp,timestamp)
                filename_depth = DATASET_PATH + "/depth/{:6f}.png".format(timestamp)
                filename_rgb = DATASET_PATH + "/rgb/{:6f}.png".format(timestamp)
                cv2.imwrite(filename_depth, d_img)
                cv2.imwrite(filename_rgb, rgb_img)
            #==============================================

        # For dataset ================================
        if CREATE_DATASET:
            with open(ASSOCIATE_PATH, "a") as f:
                f.write(data)
        #==============================================

        self.video.release()
        print('Time: ', datetime.now() - start) 
        return


if __name__ == '__main__':
    vid = Video()
    try:
        vid.main()
    except rospy.ROSInterruptException:
        pass