// plugins/Drilling-Control/DrillingControlView.js
(function () {
    class DrillingControlView {
        constructor(element, openmct) {
            this.element  = element;
            this.openmct  = openmct;

            // WebSocket
            this.ws               = null;
            this.reconnectInterval = null;
            this.wsConnected      = false;

            // Rover / drilling state
            this.currentRoverState = { rover_state: 'IDLE', active_mission: '' };
            this.last_known_height = 0.0;

            this.currentManualInputState = {
                manual_up:   false,
                manual_down: false,
                auger_on:    false,
                gate_open:   false
            };

            // DOM refs — populated in initializeUI()
            this.rosStatusDot            = null;
            this.rosStatus               = null;
            this.fsmStateDisplay         = null;
            this.platformDepthDisplay    = null;
            this.sampleWeightDisplay     = null;
            this.platformUpButton        = null;
            this.platformDownButton      = null;
            this.augerToggleSwitch       = null;
            this.gateToggleSwitch        = null;
            this.webcamImageElement      = null;
            this.webcamStatusMsgElement  = null;
            this.webcamSnapshotButton    = null;
        }

        // ─── WebSocket ──────────────────────────────────────────────────────

        initWS() {
            if (this.ws && this.ws.readyState !== WebSocket.CLOSED) return;

            this.ws = new WebSocket("ws://localhost:8080");

            this.ws.onopen = () => {
                console.log("[DrillingControlView] Connected to WS bridge");
                this.wsConnected = true;
                this.updateConnectionStatus(true);
                if (this.reconnectInterval) {
                    clearInterval(this.reconnectInterval);
                    this.reconnectInterval = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const msg  = JSON.parse(event.data);
                    const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;

                    switch (msg.type) {
                        case 'rover_status':
                            this.handleRoverStatus(data);
                            break;
                        case 'drilling_status':
                            this.handleDrillingStatus(data);
                            break;
                        case 'drilling_fsm_state':
                            this.handleFsmState(data);
                            break;
                        case 'camera_frame':
                            this.handleCameraFrame(data);
                            break;
                    }
                } catch (e) {
                    console.error("[DrillingControlView] Failed to parse message", e);
                }
            };

            this.ws.onclose = () => {
                console.warn("[DrillingControlView] Disconnected. Reconnecting in 3s...");
                this.wsConnected = false;
                this.updateConnectionStatus(false);
                this.stopWebcam();
                this.scheduleReconnect();
            };

            this.ws.onerror = (err) => {
                console.error("[DrillingControlView] WebSocket error", err);
                this.ws.close();
            };
        }

        scheduleReconnect() {
            if (this.reconnectInterval) return;
            this.reconnectInterval = setInterval(() => {
                console.log("[DrillingControlView] Attempting reconnect...");
                this.initWS();
            }, 3000);
        }

        // ─── Publish drilling command over WS ───────────────────────────────

        publishDrillingCommand() {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') {
                console.warn('Not in Teleoperation mode. Command not sent.');
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Manual controls are disabled outside of Teleoperation mode.');
                }
                return;
            }

            if (!this.wsConnected || !this.ws || this.ws.readyState !== WebSocket.OPEN) {
                console.warn('[DrillingControlView] WS not connected. Command not sent.');
                return;
            }

            // Use std_msgs/String-equivalent JSON payload
            // Bridge publishes this to /drilling/command_to_actuators as std_msgs/String
            this.ws.send(JSON.stringify({
                type: 'drilling_cmd',
                data: {
                    target_height_cm: (this.currentManualInputState.manual_up || this.currentManualInputState.manual_down)
                        ? 0.0
                        : this.last_known_height,
                    manual_up:   this.currentManualInputState.manual_up,
                    manual_down: this.currentManualInputState.manual_down,
                    auger_on:    this.currentManualInputState.auger_on,
                    gate_open:   this.currentManualInputState.gate_open
                }
            }));

            console.log('[DrillingControlView] Sent drilling command:', this.currentManualInputState);
        }

        // ─── Inbound message handlers ───────────────────────────────────────

        handleRoverStatus(data) {
            this.currentRoverState = {
                rover_state:    data.rover_state    || 'UNKNOWN',
                active_mission: data.active_mission || ''
            };
            this.updateManualControlUIState();
        }

        handleDrillingStatus(data) {
            // Expects { current_height: float, current_weight: float }
            const height = parseFloat(data.current_height) || 0.0;
            const weight = parseFloat(data.current_weight) || 0.0;
            this.last_known_height = height;

            if (this.platformDepthDisplay) {
                this.platformDepthDisplay.textContent = height.toFixed(1);
            }
            if (this.sampleWeightDisplay) {
                this.sampleWeightDisplay.textContent = weight.toFixed(0);
            }
        }

        handleFsmState(data) {
            // Expects { data: "STATE_STRING" }  (std_msgs/String equivalent)
            const state = typeof data === 'string' ? data : (data.data || '');
            if (this.fsmStateDisplay) {
                this.fsmStateDisplay.textContent = state;
            }
        }

        handleCameraFrame(data) {
            // Expects { data: "<base64 jpeg>" }
            if (!data || !data.data) return;
            const src = `data:image/jpeg;base64,${data.data}`;
            if (this.webcamImageElement) {
                this.webcamImageElement.src          = src;
                this.webcamImageElement.style.display = 'block';
            }
            this.hideWebcamStatus();
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.display = 'flex';
            }
        }

        // ─── Webcam helpers ─────────────────────────────────────────────────

        stopWebcam() {
            if (this.webcamImageElement) {
                this.webcamImageElement.src           = '';
                this.webcamImageElement.style.display = 'none';
            }
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.display = 'none';
            }
            this.displayWebcamStatus('Webcam stream paused/stopped.', 'info');
        }

        takeWebcamSnapshot() {
            if (!this.webcamImageElement || !this.webcamImageElement.src) {
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Webcam feed not available.');
                }
                return;
            }
            const link      = document.createElement('a');
            link.href       = this.webcamImageElement.src;
            link.download   = `drilling-snapshot-${Date.now()}.jpg`;
            link.click();

            if (this.openmct && this.openmct.notifications) {
                this.openmct.notifications.info('Snapshot captured!');
            }
        }

        displayWebcamStatus(message, type = 'info') {
            if (!this.webcamStatusMsgElement) return;
            this.webcamStatusMsgElement.textContent = message;
            this.webcamStatusMsgElement.className   = `drilling-webcam-status-message ${type}`;
            this.webcamStatusMsgElement.style.display = 'block';
        }

        hideWebcamStatus() {
            if (!this.webcamStatusMsgElement) return;
            this.webcamStatusMsgElement.style.display = 'none';
        }

        // ─── Render ─────────────────────────────────────────────────────────

        render() {
            fetch('./plugins/Drilling-Control/DrillingControlView.html')
                .then(response => {
                    if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
                    return response.text();
                })
                .then(html => {
                    this.element.innerHTML = html;

                    const link  = document.createElement('link');
                    link.rel    = 'stylesheet';
                    link.href   = './plugins/Drilling-Control/DrillingControlView.css';
                    document.head.appendChild(link);

                    this.initializeUI();
                    this.initWS();

                    if (this.openmct && this.openmct.editor) {
                        this.openmct.editor.on('isEditing', this.handleEditModeChange);
                    }
                })
                .catch(error => {
                    console.error('Error loading DrillingControlView.html:', error);
                    this.element.innerHTML = `<p style="color:red;">Error loading drilling control UI.</p>`;
                });
        }

        // ─── UI init ────────────────────────────────────────────────────────

        initializeUI() {
            this.rosStatusDot         = this.element.querySelector('#drillingRosStatusDot');
            this.rosStatus            = this.element.querySelector('#drillingRosStatus');
            this.fsmStateDisplay      = this.element.querySelector('#drillingFsmState');
            this.platformDepthDisplay = this.element.querySelector('#drillingPlatformDepth');
            this.sampleWeightDisplay  = this.element.querySelector('#drillingSampleWeight');
            this.platformUpButton     = this.element.querySelector('#drillingPlatformUpButton');
            this.platformDownButton   = this.element.querySelector('#drillingPlatformDownButton');
            this.augerToggleSwitch    = this.element.querySelector('#drillingAugerToggle');
            this.gateToggleSwitch     = this.element.querySelector('#drillingGateToggle');

            const webcamContainer = this.element.querySelector('#drillingWebcamContainer');
            if (webcamContainer) {
                this.webcamImageElement     = webcamContainer.querySelector('#drillingWebcamImage');
                this.webcamSnapshotButton   = webcamContainer.querySelector('#drillingSnapshotButton');
                this.webcamStatusMsgElement = webcamContainer.querySelector('#drillingWebcamStatusMessage');

                if (this.webcamImageElement)    this.webcamImageElement.style.display    = 'none';
                if (this.webcamSnapshotButton)  this.webcamSnapshotButton.style.display  = 'none';
                this.displayWebcamStatus('Waiting for WS connection...', 'info');
            }

            this.addEventListeners();
            this.updateManualControlUIState();
        }

        addEventListeners() {
            const addMomentary = (btn, cmd) => {
                if (!btn) return;
                btn.addEventListener('mousedown',  () => { if (!btn.disabled) this.handleManualButton(cmd, true);  });
                btn.addEventListener('mouseup',    () => { if (!btn.disabled) this.handleManualButton(cmd, false); });
                btn.addEventListener('mouseleave', () => { if (!btn.disabled) this.handleManualButton(cmd, false); });
            };

            addMomentary(this.platformUpButton,   'manual_up');
            addMomentary(this.platformDownButton, 'manual_down');

            if (this.augerToggleSwitch) {
                this.augerToggleSwitch.addEventListener('change', () => {
                    this.handleSwitchChange('auger_on', this.augerToggleSwitch.checked);
                });
            }
            if (this.gateToggleSwitch) {
                this.gateToggleSwitch.addEventListener('change', () => {
                    this.handleSwitchChange('gate_open', this.gateToggleSwitch.checked);
                });
            }
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.addEventListener('click',      () => this.takeWebcamSnapshot());
                this.webcamSnapshotButton.addEventListener('mousedown',  () => {
                    this.webcamSnapshotButton.style.transform  = 'translateX(-50%) scale(0.95)';
                    this.webcamSnapshotButton.style.boxShadow  = '0 2px 4px rgba(0,0,0,0.2)';
                });
                this.webcamSnapshotButton.addEventListener('mouseup',    () => {
                    this.webcamSnapshotButton.style.transform  = 'translateX(-50%) scale(1)';
                    this.webcamSnapshotButton.style.boxShadow  = '0 4px 8px rgba(0,0,0,0.3)';
                });
                this.webcamSnapshotButton.addEventListener('mouseleave', () => {
                    this.webcamSnapshotButton.style.transform  = 'translateX(-50%) scale(1)';
                    this.webcamSnapshotButton.style.boxShadow  = '0 4px 8px rgba(0,0,0,0.3)';
                });
            }
        }

        handleManualButton(cmd, value) {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') return;
            this.currentManualInputState[cmd] = value;
            if (cmd === 'manual_up'   && value) this.currentManualInputState.manual_down = false;
            if (cmd === 'manual_down' && value) this.currentManualInputState.manual_up   = false;
            this.publishDrillingCommand();
        }

        handleSwitchChange(cmd, value) {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') {
                // Revert toggle
                if (this.augerToggleSwitch) this.augerToggleSwitch.checked = this.currentManualInputState.auger_on;
                if (this.gateToggleSwitch)  this.gateToggleSwitch.checked  = this.currentManualInputState.gate_open;
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Manual controls are disabled outside of Teleoperation mode.');
                }
                return;
            }
            this.currentManualInputState[cmd] = value;
            this.publishDrillingCommand();
        }

        handleEditModeChange = (isEditing) => {
            if (isEditing) {
                this.stopWebcam();
                this.displayWebcamStatus('Webcam: In edit mode. Stream paused.', 'info');
            }
            // Stream resumes automatically when new frames arrive after edit mode exits
        };

        // ─── UI state ───────────────────────────────────────────────────────

        updateConnectionStatus(connected) {
            if (this.rosStatusDot) this.rosStatusDot.classList.toggle('connected', connected);
            if (this.rosStatusDot) this.rosStatusDot.classList.toggle('error',     !connected);
            if (this.rosStatus) {
                this.rosStatus.textContent = connected ? 'Connected to ROS' : 'Disconnected';
                this.rosStatus.classList.toggle('connected', connected);
                this.rosStatus.classList.toggle('error',     !connected);
            }
        }

        updateManualControlUIState() {
            const enabled = this.currentRoverState.active_mission.toLowerCase() === 'teleoperation';

            [this.platformUpButton, this.platformDownButton].forEach(btn => {
                if (!btn) return;
                btn.disabled = !enabled;
                btn.classList.toggle('disabled-manual-control', !enabled);
            });

            [this.augerToggleSwitch, this.gateToggleSwitch].forEach(sw => {
                if (!sw) return;
                sw.disabled = !enabled;
                const container = sw.closest('.drilling-switch-container');
                if (container) container.classList.toggle('disabled', !enabled);
            });

            const note = this.element.querySelector('.drilling-control-section p');
            if (note) {
                if (enabled) {
                    note.textContent  = 'These controls directly command the rig. Active in Teleoperation mode.';
                    note.style.color  = '#666';
                } else {
                    note.textContent  = `Manual controls disabled. Current mission: ${this.currentRoverState.active_mission || 'None'}.`;
                    note.style.color  = '#e74c3c';
                }
            }
        }

        // ─── Destroy ────────────────────────────────────────────────────────

        destroy() {
            console.log('[DrillingControlView] Destroying...');

            if (this.reconnectInterval) clearInterval(this.reconnectInterval);
            if (this.ws) this.ws.close();

            if (this.openmct && this.openmct.editor) {
                this.openmct.editor.off('isEditing', this.handleEditModeChange);
            }

            // Null out all refs
            this.ws = this.ros = this.openmct = null;
            this.platformUpButton = this.platformDownButton = null;
            this.augerToggleSwitch = this.gateToggleSwitch = null;
            this.webcamImageElement = this.webcamStatusMsgElement = this.webcamSnapshotButton = null;
            this.rosStatusDot = this.rosStatus = this.fsmStateDisplay = null;
            this.platformDepthDisplay = this.sampleWeightDisplay = null;
            this.currentRoverState = null;

            this.element.innerHTML = '';
        }
    }

    window.DrillingControlView = DrillingControlView;
})();