// ROS连接处理类
const ROS_WS_URL = 'ws://192.168.9.79:9090';
class ROSHandler {
    constructor(wsUrl) {
        this.wsUrl = wsUrl;
        this.ros = null;
        this.subTopics = new Map();
        this.pubTopics = new Map();
        this.connectStatus = false;
        this.reconnectDelay = 2000;
        this.reconnectTimer = null;
    }

    connect() {
        if (this.ros) this.ros.close();
        this.ros = new ROSLIB.Ros({ url: this.wsUrl });

        this.ros.on('connection', () => {
            this._rebuildSubTopics();
            this._rebuildPubTopics();
            this.connectStatus = true;
            if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
            this.updateConnectionStatus(true);
        });

        this.ros.on('close', () => {
            this.connectStatus = false;
            this.updateConnectionStatus(false);
            this._scheduleReconnect();
        });

        this.ros.on('error', () => {
            this.connectStatus = false;
            this.updateConnectionStatus(false);
            this._scheduleReconnect();
        });
    }

    updateConnectionStatus(connected) {
        document.getElementById('robotStatus').style.backgroundColor = connected ? 'green' : 'red';
        document.getElementById('connectionStatus').className = connected ? 'connected' : 'disconnected';
        document.getElementById('connectionStatus').innerText = connected ? 'Connected Robot OK' : 'Connected Robot Error';
    }

    registerSubTopic(topicName, messageType, callback) {
        this.subTopics.set(topicName, { messageType, callback });
    }

    _rebuildSubTopics() {
        this.subTopics.forEach(({ messageType, callback }, topicName) => {
            const topic = new ROSLIB.Topic({ ros: this.ros, name: topicName, messageType });
            topic.subscribe((msg) => {
                this.lastMsgTime = Date.now();
                callback(msg);
            });
            this.subTopics.set(topicName, { ...this.subTopics.get(topicName), instance: topic });
        });
    }

    registerPubTopic(topicName, messageType) {
        this.pubTopics.set(topicName, { messageType });
    }

    _rebuildPubTopics() {
        this.pubTopics.forEach(({ messageType }, topicName) => {
            const topic = new ROSLIB.Topic({ ros: this.ros, name: topicName, messageType });
            this.pubTopics.set(topicName, { ...this.pubTopics.get(topicName), instance: topic });
        });
    }

    publish(topicName, message) {
        if (!this.ros || !this.connectStatus) {
            console.warn('连接未建立，无法发布消息');
            return;
        }
        const pubTopic = this.pubTopics.get(topicName);
        if (!pubTopic || !pubTopic.instance) {
            console.error(`未注册发布话题：${topicName}`);
            return;
        }
        pubTopic.instance.publish(message);
    }

    _scheduleReconnect() {
        if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
        this.reconnectTimer = setTimeout(() => this.connect(), this.reconnectDelay);
    }
}
const rosHandler = new ROSHandler(ROS_WS_URL);

// ROS话题注册
function registerAllSubTopics() {
    rosHandler.registerSubTopic('/map', 'nav_msgs/OccupancyGrid', (message) => {
        mapData = message;
        initOffscreenCanvas();
        drawOffscreenMap();
        resetView();
        isViewDirty = true;
    });

    rosHandler.registerSubTopic('/robot_pose', 'geometry_msgs/PoseStamped', (message) => {
        robotPose = message;
        isViewDirty = true;
    });

    rosHandler.registerSubTopic('/node_list', 'visualization_msgs/MarkerArray', (message) => {
        markerArrayData = message.markers;
        isViewDirty = true;
        updateNodeSelectList();
    });

    rosHandler.registerSubTopic('/scan_points', 'sensor_msgs/PointCloud', (message) => {
        pointCloudData = message;
        isViewDirty = true;
        if (rosHandler.connectStatus) {
            statusToast.textContent = `已连接 ROS | 点云点数：${message.points.length}`;
        }
    });

    rosHandler.registerSubTopic('/battery_info', 'sensor_msgs/BatteryState', (message) => {
        try {
            let batteryPercentage = 0;
            let chargeStatus = '未充电';

            if (message.percentage !== undefined && !isNaN(message.percentage)) {
                batteryPercentage = Math.round(message.percentage * 100);
            } else if (message.voltage !== undefined && !isNaN(message.voltage)) {
                const minVoltage = 3.0;
                const maxVoltage = 4.2;
                const clampedVoltage = Math.max(minVoltage, Math.min(maxVoltage, message.voltage));
                batteryPercentage = Math.round(((clampedVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100);
            }

            if (message.power_supply_status === 1) chargeStatus = '充电中';
            else if (message.power_supply_status === 4) chargeStatus = '已充满';

            updateRobotState({ battery: batteryPercentage, charge: chargeStatus });
        } catch (e) {
            console.error('解析电池状态失败:', e);
        }
    });

    rosHandler.registerSubTopic('/rosout', 'rosgraph_msgs/Log', (message) => {
        const levelMap = { 0: 'debug', 1: 'info', 2: 'warn', 3: 'error', 4: 'fatal' };
        const logLevel = levelMap[message.level] || 'debug';
        const logContent = `[${message.name}] ${message.msg}`;
        addRosLog(logLevel, logContent);
    });

    rosHandler.registerSubTopic('/navigation_plan_node/task_point', 'visualization_msgs/Marker', (message) => {
        taskPointData = message;
        isViewDirty = true;
        if (rosHandler.connectStatus) {
            statusToast.textContent = `已接收任务路径`;
        }
    });
}

function registerAllPubTopics() {
    rosHandler.registerPubTopic('/remote_cmd', 'std_msgs/Header');
    rosHandler.registerPubTopic('/clicked_point', 'geometry_msgs/PointStamped');
    rosHandler.registerPubTopic('/move_base_simple/goal', 'geometry_msgs/PoseStamped');
    rosHandler.registerPubTopic('/ui_cmd_vel', 'geometry_msgs/Twist');
    rosHandler.registerPubTopic('/initialpose', 'geometry_msgs/PoseWithCovarianceStamped');
}

function publishRemoteCmd(seq, frameId = '') {
    if (!rosHandler.connectStatus) {
        statusToast.textContent = '未连接ROS，无法发布/remote_cmd话题';
        return false;
    }
    const now = new Date();
    const headerMsg = new ROSLIB.Message({
        seq: seq,
        stamp: {
            secs: Math.floor(now.getTime() / 1000),
            nsecs: (now.getTime() % 1000) * 1000000
        },
        frame_id: frameId || ''
    });
    rosHandler.publish('/remote_cmd', headerMsg);
    console.log(`已发布/remote_cmd（seq=${seq}）：`, headerMsg);
    return true;
}
