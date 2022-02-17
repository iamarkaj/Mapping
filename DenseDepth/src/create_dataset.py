import os
import cv2
import shutil
import numpy as np
from datetime import datetime

from depth import PredictDepth
from layers import BilinearUpSampling2D


CAMERA_FPS = 30
SKIP_FRAMES = 1 # skip n number of frames
DATASET_PATH = os.path.dirname(os.path.abspath(__file__)) + "/../../Dataset" 


class DenseDepth():
    def __init__(self):
        self.video = cv2.VideoCapture(DATASET_PATH + "/video.mp4")
        self.depth = PredictDepth()
        
        with open(DATASET_PATH + "/associate.txt", "w") as f:
            pass
        
        if os.path.exists(DATASET_PATH + '/depth'): 
            shutil.rmtree(DATASET_PATH + '/depth')
        
        if os.path.exists(DATASET_PATH + '/rgb'): 
            shutil.rmtree(DATASET_PATH + '/rgb')
        
        os.makedirs(DATASET_PATH + '/depth')
        os.makedirs(DATASET_PATH + '/rgb')

    def main(self):
        data = ""
        frame_count = 0
        timestamp = 1/CAMERA_FPS
        start = datetime.now()
        
        while self.video.grab():
            frame_count = frame_count + 1
            
            ret, rgb_img = self.video.retrieve()
            
            if not ret: 
                break

            if frame_count % SKIP_FRAMES != 0: 
                continue
            
            print("Frame Count:", frame_count)

            d_img, rgb_img = self.depth.predict(rgb_img)
            
            timestamp = timestamp + 0.04
            data = data + "{:6f} rgb/{:6f}.png {:6f} depth/{:6f}.png\n".format(timestamp,timestamp,timestamp,timestamp)
            filename_depth = DATASET_PATH + "/depth/{:6f}.png".format(timestamp)
            filename_rgb = DATASET_PATH + "/rgb/{:6f}.png".format(timestamp)
            cv2.imwrite(filename_depth, d_img)
            cv2.imwrite(filename_rgb, rgb_img)

        with open(DATASET_PATH + "/associate.txt", "a") as f:
            f.write(data)
        
        self.video.release()
        print('Time:', datetime.now() - start) 
        return


if __name__ == '__main__':
    a = DenseDepth()
    a.main()