To save a calibration:

```
rostopic pub /multicamera_calibration/action std_msgs/String "save" -1
```
or
```
rostopic pub /multicamera_calibration/action std_msgs/String "saveCam2WorldPose" -1
```