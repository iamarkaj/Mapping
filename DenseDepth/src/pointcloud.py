import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def CreatePointCloudMsg(depth, rgb, stamp, seq):
    msg = PointCloud2()

    msg.header.frame_id = "map"
    msg.header.stamp = stamp
    msg.header.seq = seq

    height, width = depth.shape

    # Point cloud data numpy array
    i = 0
    data = np.zeros((height * width * 6), dtype=np.float32)

    # Message data size
    msg.height = 1
    msg.width = width * height

    # Iterate images and build point cloud
    for y in range(height):
        for x in range(width):
            data[i] = (x - (width / 2))  / 100.0
            data[i + 1] = (-y + (height / 2)) / 100.0
            data[i + 2] = depth[y, x] / 25
            data[i + 3] = float(rgb[y, x, 0]) / 255.0
            data[i + 4] = float(rgb[y, x, 1]) / 255.0
            data[i + 5] = float(rgb[y, x, 2]) / 255.0
            i += 6

    # Fields of the point cloud
    msg.fields = [
        PointField("y", 0, PointField.FLOAT32, 1),
        PointField("z", 4, PointField.FLOAT32, 1),
        PointField("x", 8, PointField.FLOAT32, 1),
        PointField("b", 12, PointField.FLOAT32, 1),
        PointField("g", 16, PointField.FLOAT32, 1),
        PointField("r", 20, PointField.FLOAT32, 1)
    ]

    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * height * width
    msg.is_dense = True
    msg.data = data.tostring()

    return msg