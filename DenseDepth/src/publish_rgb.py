import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image


MODEL_PATH = "../data/video.mp4"
CAMERA_RGB_TOPIC = "/camera/image_raw"
CAMERA_FRAME_ID = "camera_link"


def main():
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher(CAMERA_RGB_TOPIC, Image,  queue_size=1)

    video = cv2.VideoCapture(MODEL_PATH)

    while video.grab():
        tmp, rgb_img = video.retrieve()
        if tmp: break

    img_msg = Image()
    img_msg.header.stamp = rospy.Time.now()
    img_msg.encoding = "bgr8"
    img_msg.width = rgb_img.shape[1]
    img_msg.height = rgb_img.shape[0]
    img_msg.step = rgb_img.shape[1] * rgb_img.shape[2]
    img_msg.header.frame_id = CAMERA_FRAME_ID

    while not rospy.is_shutdown() and video.grab():
        tmp, rgb_img = video.retrieve()

        if not tmp:
            print ("End")
            break

        img_msg.data = np.array(rgb_img, dtype=np.uint8).flatten().tolist()
        img_pub.publish(img_msg)
    
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass