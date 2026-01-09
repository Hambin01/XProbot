let isSensorControlActive = false;  // 体感控制开关状态
let sensorInterval = null;          // 传感器数据监听定时器
const sensorUpdateRate = 50;        // 传感器数据更新频率(ms)

const btnSensorControl = document.getElementById('btn-sensor-control');
function handleSensorData(event) {
    if (!isSensorControlActive) return;

    // 1. 获取手机加速度/陀螺仪数据（优先用DeviceOrientation，兼容移动端）
    let beta = event.beta || 0;  // 前后倾斜角度 (-180 ~ 180)
    let gamma = event.gamma || 0;// 左右倾斜角度 (-90 ~ 90)

    // 2. 角度映射为速度（线性映射，可根据实际需求调整灵敏度）
    // beta 前后倾斜 → 机器人前进/后退速度 (范围 -maxSpeed ~ maxSpeed)
    const linearX = -(beta / 45) * maxSpeed;
    // gamma 左右倾斜 → 机器人旋转速度 (范围 -1 ~ 1)
    const angularZ = -(gamma / 45) * 1.0;

    // 3. 限制速度范围，防止超量程
    const clampedLinearX = Math.max(-maxSpeed, Math.min(maxSpeed, linearX));
    const clampedAngularZ = Math.max(-1, Math.min(1, angularZ));

    // 4. 发布速度指令（复用原有话题 /ui_cmd_vel）
    rosHandler.publish('/ui_cmd_vel', new ROSLIB.Message({
        linear: { x: clampedLinearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: clampedAngularZ }
    }));
}

// 体感控制开关切换
function toggleSensorControl() {
    isSensorControlActive = !isSensorControlActive;

    if (isSensorControlActive) {
        // 开启体感控制
        btnSensorControl.textContent = '关闭体感控制';
        btnSensorControl.classList.add('active');
        statusToast.textContent = '已开启体感控制：倾斜手机控制机器人';
        // 禁用摇杆控制（防止冲突）
        isJoystickActive = false;
        resetJoystick();
        // 监听手机朝向传感器
        if (window.DeviceOrientationEvent) {
            window.addEventListener('deviceorientation', handleSensorData);
        } else {
            // 备用方案：用加速度传感器（兼容性较差）
            window.addEventListener('devicemotion', (e) => {
                handleSensorData({
                    beta: e.accelerationIncludingGravity.y * 10,
                    gamma: e.accelerationIncludingGravity.x * 10
                });
            });
            statusToast.textContent = '设备不支持陀螺仪，已启用加速度传感';
        }
    } else {
        // 关闭体感控制
        btnSensorControl.textContent = '开启体感控制';
        btnSensorControl.classList.remove('active');
        statusToast.textContent = '已关闭体感控制';
        // 移除传感器监听
        window.removeEventListener('deviceorientation', handleSensorData);
        window.removeEventListener('devicemotion', handleSensorData);
        // 停止机器人运动
        resetJoystick();
    }
}

btnSensorControl.addEventListener('click', toggleSensorControl);