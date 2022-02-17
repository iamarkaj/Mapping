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

### DenseDepth

A sample TUM dataset is provided in `Datatset`. <br> 
So, this step can be skipped or save the new video as `Datatset/video.mp4`

##### Publish RGB and D images
```
cd $WORKSPACE/Mapping && DenseDepth/venv/bin/python3 DenseDepth/src/main.py
```

##### Create Datatset
```
cd $WORKSPACE/Mapping && DenseDepth/venv/bin/python3 DenseDepth/src/create_dataset.py
```

### ORB_SLAM2

Goto `$WORKSPACE/Mapping/ORB_SLAM2/config/params.yaml` and set the camera paramaters 

```
cd $WORKSPACE/Mapping && ./ORB_SLAM2/bin/RgbdNode ORB_SLAM2/Vocabulary/ORBvoc.bin ORB_SLAM2/config/params.yaml Dataset Dataset/associate.txt
```

### References

 [1] https://github.com/ialhashim/DenseDepth <br>
 [2] https://github.com/raulmur/ORB_SLAM2