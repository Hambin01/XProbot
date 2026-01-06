/////////system
const systemTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/system_cmd',
    messageType: 'std_msgs/String',
});

document.getElementById('reboot').addEventListener('click', () => {
    systemFun('echo \'nvidia\' | sudo -S sudo reboot');
});

document.getElementById('restart').addEventListener('click', () => {
    systemFun('sh /home/nvidia/leapting_robot/script/restart_to_back.sh &');
});

document.getElementById('time_sync').addEventListener('click', () => {
    const now = new Date();
    const formattedDate = `${now.getFullYear()}-${(now.getMonth() + 1).toString().padStart(2, '0')}-${now.getDate().toString().padStart(2, '0')} ${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    console.log(formattedDate);

    systemFun(`echo \'nvidia\' | sudo -S sudo date -s '${formattedDate}'`);
    setTimeout(() => {
        systemFun(`echo \'nvidia\' | sudo -S sudo hwclock --systohc`);
    }, 1000);
});

document.getElementById('send_system').addEventListener('click', () => {
    const cmd = systemInput.value.trim();
    if (cmd !== '') {
        systemFun(cmd + " &");
    }
});

document.getElementById('record_start').addEventListener('click', () => {
    systemFun('bash /home/nvidia/leapting_robot/script/record_bag.sh record &');
});

document.getElementById('record_end').addEventListener('click', () => {
    systemFun('bash /home/nvidia/leapting_robot/script/record_bag.sh end &');
});

function systemFun(cmd) {
    const msg = new ROSLIB.Message({
        data: cmd,
    });
    systemTopic.publish(msg);
}

function light(cmd) {
    systemFun(cmd);
}