import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.layers import Layer, InputSpec

from layers import BilinearUpSampling2D


BATCH_SIZE = 1
MIN_DEPTH = 10
MAX_DEPTH = 1000
MODEL_PATH = os.path.dirname(os.path.abspath(__file__)) + "/../model/nyu.h5"


class PredictDepth:
    def __init__(self):
        custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': self.__depth_loss_function}
        self.model = load_model(MODEL_PATH, custom_objects=custom_objects, compile=False)
    
    def __depthNorm(self, x, max_depth):
        ret = max_depth / x
        return ret

    def __depth_loss_function(self, y_true, y_pred):
        theta = 0.1
        max_depth_val = MAX_DEPTH / MIN_DEPTH
        l_depth = tf.keras.backend.mean(tf.keras.backend.abs(y_pred - y_true), axis = -1)
        dy_true, dx_true = tf.image.image_gradients(y_true)
        dy_pred, dx_pred = tf.image.image_gradients(y_pred)
        l_edges = tf.keras.backend.mean(tf.keras.backend.abs(dy_pred - dy_true) + tf.keras.backend.abs(dx_pred - dx_true), axis = -1)
        l_ssim = tf.keras.backend.clip((1 - tf.image.ssim(y_true, y_pred, max_depth_val)) * 0.5, 0, 1)
        w1 = 1.0
        w2 = 1.0
        w3 = theta
        ret = (w1 * l_ssim) + (w2 * tf.keras.backend.mean(l_edges)) + (w3 * tf.keras.backend.mean(l_depth))
        
        return ret

    def __edges(self, img):
        img = cv2.GaussianBlur(img, (3,3), 0) 
        img = cv2.bilateralFilter(img, 3, 10, 10)
        img = np.uint8(img)
        img = cv2.Canny(img, 100, 200)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        return img

    def __uniform(self, img):
        img = img - 25
        img[(img>200)|(img<0)] = 0
        return img

    def predict(self, rgb_img):
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        rgb_img = cv2.resize(rgb_img, (640, 480))
        rgb_img = np.clip(np.asarray(rgb_img, dtype=np.float32) / 255, 0, 1)
        rgb_img = rgb_img.reshape((1, 480, 640, 3))

        d_img = self.model.predict(rgb_img, batch_size=BATCH_SIZE)
        d_img = np.clip(self.__depthNorm(d_img, max_depth=MAX_DEPTH), MIN_DEPTH, MAX_DEPTH) / MAX_DEPTH
        d_img = d_img[0][:,:,0]
        d_img = cv2.resize(d_img, (640,480))

        rgb_img = rgb_img[0] * 255
        d_img = d_img * 255
        
        # Some filtering
        # rgb_img_gray = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
        # rgb_img_edge = self.__edges(rgb_img_gray)
        # d_img = self.__uniform(d_img)
        # d_img[rgb_img_edge==255] = 0
        
        return d_img, rgb_img