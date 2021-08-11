cd gym-gazebo/gym_gazebo/envs/installation/active_ws/

rosdep install --from-paths src --ignore-src -r -y && cd ..

pip3 install -r requirements.txt

cd ../../../../../Active-Suspension/

pip3 install -e gym-gazebo/

cd gym-gazebo/gym_gazebo/envs/installation

./setup_noetic.bash

