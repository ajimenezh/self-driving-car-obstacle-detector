set -e

cd ./src/lidar

rosrun lidar lidar_node &

cd ../..
cd ./src/camera

./ros_node &

wait
