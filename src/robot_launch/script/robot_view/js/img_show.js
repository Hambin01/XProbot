////////
var listeners = {
    vision: null,
    infrared: null,
    front: null
};
const img64 = new ROSLIB.Topic({
    ros: ros,
    name: '/video_vision_node/img64',
    messageType: 'std_msgs/String'
});

document.getElementById('vision').style.display = 'none';
document.getElementById('infrared').style.display = 'none';
document.getElementById('front').style.display = 'none';
document.getElementById("visionCheckbox").checked = false;
document.getElementById("infraredCheckbox").checked = false;
document.getElementById("frontCheckbox").checked = false;
pic_width = window.screen.width * 0.4;
pic_height = window.screen.height * 0.4;
function toggleSubscription(topic) {
    if (document.getElementById(topic + 'Checkbox').checked) {
        listeners[topic] = new ROSLIB.Topic({
            ros: ros,
            name: '/video_' + topic + '_node/img64',
            messageType: 'std_msgs/String'
        });
        document.getElementById(topic).style.display = 'inline';

        listeners[topic].subscribe(function (message) {
            const base64Data = message.data;
            imgshow = document.getElementById(topic);
            imgshow.src = "data:image/jpg;base64," + base64Data;
            imgshow.style.width = "90%";
        });
    } else {
        if (listeners[topic]) {
            listeners[topic].unsubscribe();
            listeners[topic] = null;
            console.log(topic + "  unsubscribe!");
            imgshow = document.getElementById(topic);
            imgshow.className = img64;
            imgshow.src = "data:image/jpg;base64," + "base64Data";
            imgshow.style.display = 'none';
        }
    }
}
