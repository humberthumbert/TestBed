To run the program, one should:
1. In current directory, run
    pip install -r requirement.txt
   to install the required package.
2. Then, one should go into directory and build a static library.
    cd <current directory>/scripts/preprocess
    make
3. Then, to make yolo works, one should:
    cd <current directory>/scripte/yolo/yolo
    bash yolov3_weight.sh
   to download the precalculated weight.
4. To run the program with kitti data, first read the kitti_data_requirement.txt
   to get the correct directory structure.
   Once the data has been placed correcly, in current directory run:
    python offline_detection.py kitti 20xx_xx_xx xxxx
   where the last two parameters represent the date and drive number given by kitti
   data set. 
5. To run with launch file:
    `roslaunch pixor offline_pipeline.launch type:='-kitti' path:=<path> pc_front:='0' camera0:='0' camera1:='0' camera2:='0' camera3:='0'`
    
    
*  type is -kitti, -rosbag
*  path is path to data directory
*  pc_front, cameraN is '0'/'1' stand for choosing or not