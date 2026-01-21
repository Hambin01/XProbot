source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate pytorch
echo 'nvidia' | sudo -S pkill -f -INT mapping
echo 'nvidia' | sudo -S pkill -f -INT device_management
echo "wait mapping start ...."
sleep  3
roslaunch  robot_launch mapping.launch &
while true; do
    if pgrep -f 'param_start' &> /dev/null; then
        echo "param_start start ok"
        break
    else
        sleep 2 
    fi  
done

