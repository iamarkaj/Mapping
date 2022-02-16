#!/usr/bin/env python

import numpy as np
from skimage.transform import resize

from tensorflow.python.client import device_lib 
print("GPU Devices:", device_lib.list_local_devices())

from tensorflow.keras.models import load_model
from tensorflow.keras.layers import Layer, InputSpec

from layers import BilinearUpSampling2D
from fill_depth_colorization import fill_depth_colorization

class PredictDepth:
    def __init__(self, min_depth, max_depth, batch_size, model_path):
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.batch_size = batch_size
        self.model_path = model_path

        custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}
        self.model = load_model(self.model_path, custom_objects=custom_objects, compile=False)
    
    def __depthNorm(self, x, max_depth):
        ret = max_depth / x
        return ret

    def predict(self, rgb_img):
        rgb_img = np.clip(np.asarray(rgb_img, dtype=np.float32) / 255, 0, 1)
        rgb_img = rgb_img.reshape((1, rgb_img.shape[0], rgb_img.shape[1], rgb_img.shape[2]))
        
        d_img_pred = self.model(rgb_img, training=False).numpy() # image shape must be (480, 640, 3)
        # d_img_pred = self.model.predict(rgb_img, batch_size=self.batch_size)
        
        d_img_pred = np.clip(self.__depthNorm(d_img_pred, max_depth=self.max_depth), self.min_depth, self.max_depth) / self.max_depth
        d_img_pred = d_img_pred[0][:,:,0]
        d_img_pred = d_img_pred - np.min(d_img_pred)
        d_img_pred = d_img_pred / np.max(d_img_pred)

        shape = (rgb_img[0].shape[0]/2, rgb_img[0].shape[1]/2, 3)
        rgb_img = resize(rgb_img[0], shape, preserve_range=True, mode='reflect', anti_aliasing=True )
        
        d_img = fill_depth_colorization(rgb_img, d_img_pred)
        return d_img