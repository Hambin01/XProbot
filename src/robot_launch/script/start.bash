exec 2> /home/nvidia/leapting_robot/script/robot_view/robot.log
exec 1>&2
set -x

__conda_setup="$('/home/nvidia/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/nvidia/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/nvidia/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/nvidia/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
export PATH="/home/nvidia/miniconda3/bin:$PATH"

source /home/nvidia/.bashrc
bash /home/nvidia/leapting_robot/script/start_run.bash &
