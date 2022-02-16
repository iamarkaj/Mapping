# Mapping

![](https://img.shields.io/badge/Ubuntu-18.04-green)


## Instructions

### Install
```
./install.sh
```

### Roscore

```
roscore
```

### Video

A sample TUM dataset is provided in `$WORKSPACE/Mapping/tmp/Datatset`. <br> 
So, this step can be skipped or goto `$WORKSPACE/Mapping/Video/data/` and save the new video as `video.mp4` <br>
Goto `$WORKSPACE/Mapping/Video/src/node.py` and set the variable <br>
[1] `CREATE_DATASET = True` for creating dataset or/and <br>
[2] `PUBLISH = True` for publishing the video

```
cd $WORKSPACE/Mapping && tmp/venv/bin/python3 Video/src/node.py
```

### ORB_SLAM2

Goto `$WORKSPACE/Mapping/ORB_SLAM2/config/params.yaml` and set the camera paramaters 

```
cd $WORKSPACE/Mapping && ./ORB_SLAM2/bin/RgbdNode ORB_SLAM2/Vocabulary/ORBvoc.bin ORB_SLAM2/config/params.yaml tmp/Dataset tmp/associate.txt
```

### References

 [1] https://github.com/ialhashim/DenseDepth <br>
 [2] https://github.com/raulmur/ORB_SLAM2