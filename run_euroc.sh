# sleep 1
# roslaunch vins_estimator euroc.launch &
# sleep 1
# roslaunch vins_estimator vins_rviz.launch &
# sleep 1
# roslaunch benchmark_publisher publish.launch  sequence_name:=MH_04_difficult &
# sleep 1
# # rosbag play /home/cxn/leofile/sad_dataset/EuRoc/MH_04_difficult.bag
# # rosbag play /home/cxn/leofile/vo_data/EuRoc/MH_04_difficult.bag
# rosbag play /home/cxn/leofile/vo_data/EuRoc/MH_01_easy.bag

sleep 1
roslaunch vins_estimator plusai.launch &
sleep 1
roslaunch vins_estimator vins_rviz.launch &
sleep 1
rosbag play /home/cxn/vo_data/20231224T081132_pdb-l4e-b0002_22_101to194/ros.bag