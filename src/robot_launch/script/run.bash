source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate pytorch
echo 'nvidia' | sudo -S pkill -f -INT param_start
echo 'nvidia' | sudo -S pkill -f -INT device_management
echo 'nvidia' | sudo -S pkill -f -INT leapting_robot
echo "wait restart ...."
sleep  2
roslaunch  leapting_robot param_start.launch &
while true; do
    if pgrep -f 'param_start' &> /dev/null; then
        echo "param_start start ok"
        break
    else
        sleep 2 
    fi  
done
sleep  4
roslaunch  leapting_robot leapting_robot.launch
conda deactivate
echo 'nvidia' | sudo -S pkill -f -INT param_start
