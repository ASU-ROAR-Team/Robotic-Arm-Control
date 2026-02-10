class ArmControlView {
    constructor(container, openmct, wsUrl = "ws://localhost:8080") {
        this.container = container;
        this.openmct = openmct;

        this.mode = "FK";

        this.jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

        this.jointValues = {
            joint1: 0, joint2: 0, joint3: 0,
            joint4: 0, joint5: 0, joint6: 0
        };

        this.ikValues = { x: 50, y: 0, z: 20, yaw: 0, pitch: 0, roll: 0 };

        /* ===== Presets ===== */
        this.fkPresets = {
            home: {
                joint1: 0, joint2: 0, joint3: 0,
                joint4: 0, joint5: 0, joint6: 0
            },
            rest: {
                joint1: 0, joint2: -45, joint3: 90,
                joint4: 0, joint5: 45, joint6: 0
            }
        };

        this.ikPresets = {
            home: { x: 50, y: 0, z: 20, yaw: 0, pitch: 0, roll: 0 },
            rest: { x: 30, y: 20, z: 10, yaw: 0, pitch: -45, roll: 0 }
        };

        /* ===== WebSocket ===== */
        this.ws = new WebSocket(wsUrl);
        this.ws.onopen = () => console.log("[WS] Connected");
        this.ws.onmessage = msg => console.log("[WS] RX:", msg.data);
        this.ws.onclose = () => console.log("[WS] Closed");

        /* ===== ROS ===== */
        this.ros = null;
        this.jointPublisher = null;
        this.posePublisher = null;

        this.sliders = {};
        this.numberInputs = {};
        this.ikInputs = {};
    }

    /* ================= Render ================= */

    render() {
        this.container.innerHTML = this.getHTML();
        this.statusElement = this.container.querySelector("#joystickStatus");
        this.statusDot = this.container.querySelector(".status-dot");

        this.bindElements();
        this.tryConnectROS();
        this.updateModeUI();
    }

    /* ================= HTML ================= */

    getHTML() {
        return `
        <div class="arm-control-container">
            <h2 class="section-header">Robotic Arm Control</h2>

            <div class="status-bar">
                <div class="status-dot"></div>
                <div id="joystickStatus">ROS: Disconnected</div>
                <button id="modeSwitchButton">Switch to IK Mode</button>
            </div>

            <div class="controls-grid">
                <div id="fk_container" class="joint-control">
                    <p>Adjust individual joint angles manually.</p>
                    ${this.jointNames.map(joint => `
                        <div class="joint-slider-container">
                            <label style="width:80px">${joint.toUpperCase()}</label>
                            <input type="range" id="${joint}_slider" min="-180" max="180" value="0">
                            <span id="${joint}_val_display">0°</span>
                            <input type="number" id="${joint}_number" value="0" style="display:none">
                        </div>
                    `).join('')}
                </div>

                <div id="ik_container" class="pose-control" style="display:none">
                    <p>Define the target end-effector pose.</p>
                    <div style="display:grid;grid-template-columns:1fr 1fr;gap:15px">
                        ${['x','y','z','yaw','pitch','roll'].map(k => `
                            <div>
                                <strong>${k.toUpperCase()}</strong>
                                <input type="number" id="${k}_input" value="${this.ikValues[k]}" style="width:70px">
                                <span>${['x','y','z'].includes(k) ? 'cm' : '°'}</span>
                            </div>
                        `).join('')}
                    </div>
                </div>
            </div>

            <div class="preset-buttons">
                <button class="preset-button" id="homeBtn">Home Position</button>
                <button class="preset-button" id="restBtn">Rest Position</button>
            </div>
        </div>
        `;
    }

    /* ================= UI ================= */

    updateModeUI() {
        const fk = this.container.querySelector('#fk_container');
        const ik = this.container.querySelector('#ik_container');
        const btn = this.container.querySelector('#modeSwitchButton');

        if (this.mode === 'FK') {
            fk.style.display = 'flex';
            ik.style.display = 'none';
            btn.innerText = 'Switch to IK Mode';
        } else {
            fk.style.display = 'none';
            ik.style.display = 'flex';
            btn.innerText = 'Switch to FK Mode';
        }
    }

    bindElements() {
        /* FK sliders */
        this.jointNames.forEach(joint => {
            const s = this.container.querySelector(`#${joint}_slider`);
            const n = this.container.querySelector(`#${joint}_number`);

            this.sliders[joint] = s;
            this.numberInputs[joint] = n;

            const handler = val => {
                val = Number(val);
                this.jointValues[joint] = val;
                s.value = val;
                n.value = val;
                this.container.querySelector(`#${joint}_val_display`).innerText = `${val}°`;

                if (this.mode === 'FK') this.publishJointStates();
                this.sendWSUpdate();
            };

            s.oninput = () => handler(s.value);
            n.oninput = () => handler(n.value);
        });

        /* IK inputs */
        ['x','y','z','yaw','pitch','roll'].forEach(k => {
            const el = this.container.querySelector(`#${k}_input`);
            this.ikInputs[k] = el;

            el.oninput = () => {
                this.ikValues[k] = Number(el.value);
                if (this.mode === 'IK') this.publishPose();
                this.sendWSUpdate();
            };
        });

        /* Mode switch */
        this.container.querySelector('#modeSwitchButton').onclick = () => {
            this.mode = this.mode === 'FK' ? 'IK' : 'FK';
            this.updateModeUI();
        };

        /* Preset buttons */
        this.container.querySelector('#homeBtn').onclick = () => this.applyPreset('home');
        this.container.querySelector('#restBtn').onclick = () => this.applyPreset('rest');
    }

    /* ================= Presets ================= */

    applyPreset(type) {
        if (this.mode === 'FK') {
            const preset = this.fkPresets[type];
            this.jointNames.forEach(j => {
                const v = preset[j];
                this.jointValues[j] = v;
                this.sliders[j].value = v;
                this.numberInputs[j].value = v;
                this.container.querySelector(`#${j}_val_display`).innerText = `${v}°`;
            });
            this.publishJointStates();
        } else {
            const preset = this.ikPresets[type];
            Object.keys(preset).forEach(k => {
                this.ikValues[k] = preset[k];
                this.ikInputs[k].value = preset[k];
            });
            this.publishPose();
        }

        this.sendWSUpdate();
    }

    /* ================= WS ================= */

    sendWSUpdate() {
        if (this.ws.readyState !== WebSocket.OPEN) return;

        const data = this.mode === 'FK'
            ? Object.values(this.jointValues)
            : Object.values(this.ikValues);

        if (data.some(v => isNaN(v))) return;

        this.ws.send(JSON.stringify({
            type: 'joint_cmd',
            mode: this.mode,
            data
        }));
    }

    /* ================= ROS ================= */

    tryConnectROS() {
        if (typeof ROSLIB === 'undefined') {
            this.statusElement.innerText = 'ROSLIB not loaded';
            return;
        }

        this.ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

        this.ros.on('connection', () => {
            this.statusDot.classList.add('connected');
            this.statusElement.innerText = 'ROS: Connected';
            this.setupROS();
        });

        this.ros.on('error', () => {
            this.statusElement.innerText = 'ROS: Error';
        });

        this.ros.on('close', () => {
            this.statusElement.innerText = 'ROS: Closed';
        });
    }

    setupROS() {
        this.jointPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/fk_joint_states',
            messageType: 'sensor_msgs/JointState'
        });

        this.posePublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ik_target_pose',
            messageType: 'std_msgs/Float64MultiArray'
        });
    }

    publishJointStates() {
        if (!this.jointPublisher) return;

        const pos = this.jointNames.map(j => this.jointValues[j] * Math.PI / 180);
        this.jointPublisher.publish({
            name: this.jointNames,
            position: pos
        });
    }

    publishPose() {
        if (!this.posePublisher) return;
        this.posePublisher.publish({ data: Object.values(this.ikValues) });
    }

    destroy() {
        if (this.ws) this.ws.close();
        if (this.ros) this.ros.close();
    }
}

window.ArmControlView = ArmControlView;
