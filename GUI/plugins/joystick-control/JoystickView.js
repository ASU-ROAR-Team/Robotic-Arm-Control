// plugins/joystick-control/JoystickView.js
(function () {
    class JoystickView {
        constructor(element, openmct) {
            this.element = element;
            this.openmct = openmct;

            // Canvas / drawing
            this.canvas          = null;
            this.ctx             = null;
            this.joystickRadius  = 0;
            this.thumbRadius     = 20;
            this.joystickCenterX = 0;
            this.joystickCenterY = 0;
            this.thumbX          = 0;
            this.thumbY          = 0;
            this.isDragging      = false;

            // DOM refs
            this.linearSpeedSlider     = null;
            this.angularSpeedSlider    = null;
            this.linearSpeedValueSpan  = null;
            this.angularSpeedValueSpan = null;
            this.joystickStatus        = null;
            this.joystickControlMsg    = null;

            // WebSocket
            this.ws               = null;
            this.reconnectInterval = null;
            this.wsConnected      = false;

            // Rover state
            this.currentRoverState = { rover_state: 'IDLE', active_mission: '' };

            // Bind handlers
            this.onMouseDown      = this.onMouseDown.bind(this);
            this.onMouseMove      = this.onMouseMove.bind(this);
            this.onMouseUp        = this.onMouseUp.bind(this);
            this.updateSpeedValues = this.updateSpeedValues.bind(this);
        }

        // ─── Render ─────────────────────────────────────────────────────────

        render() {
            fetch('./plugins/joystick-control/JoystickView.html')
                .then(r => r.text())
                .then(html => {
                    this.element.innerHTML = html;
                    this.initializeUI();
                    this.initWS();
                })
                .catch(err => {
                    console.error('[JoystickView] Failed to load HTML:', err);
                    this.element.innerHTML = '<p style="color:red;">Error loading joystick UI.</p>';
                });
        }

        // ─── WebSocket ───────────────────────────────────────────────────────

        initWS() {
            if (this.ws && this.ws.readyState !== WebSocket.CLOSED) return;

            this.ws = new WebSocket("ws://localhost:8080");

            this.ws.onopen = () => {
                console.log('[JoystickView] Connected to WS bridge');
                this.wsConnected = true;
                if (this.reconnectInterval) {
                    clearInterval(this.reconnectInterval);
                    this.reconnectInterval = null;
                }
                this.updateJoystickUIState();
            };

            this.ws.onmessage = (event) => {
                try {
                    const msg  = JSON.parse(event.data);
                    const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;

                    if (msg.type === 'rover_status') {
                        this.currentRoverState = {
                            rover_state:    data.rover_state    || 'UNKNOWN',
                            active_mission: data.active_mission || ''
                        };
                        this.updateJoystickUIState();
                    }
                } catch (e) {
                    console.error('[JoystickView] Failed to parse message', e);
                }
            };

            this.ws.onclose = () => {
                console.warn('[JoystickView] Disconnected. Reconnecting in 3s...');
                this.wsConnected = false;
                this.updateJoystickUIState();
                this.scheduleReconnect();
            };

            this.ws.onerror = (err) => {
                console.error('[JoystickView] WebSocket error', err);
                this.ws.close();
            };
        }

        scheduleReconnect() {
            if (this.reconnectInterval) return;
            this.reconnectInterval = setInterval(() => {
                console.log('[JoystickView] Attempting reconnect...');
                this.initWS();
            }, 3000);
        }

        // ─── Publish geometry_msgs/Twist over WS ────────────────────────────

        publishTwist(normalizedX, normalizedY) {
            // Block if disconnected or in autonomous navigation
            if (!this.wsConnected || !this.ws || this.ws.readyState !== WebSocket.OPEN) return;
            if (this.currentRoverState.active_mission.toLowerCase() === 'navigation') return;

            const maxLinear  = parseFloat(this.linearSpeedSlider?.value  ?? 1.0);
            const maxAngular = parseFloat(this.angularSpeedSlider?.value ?? 0.5);

            // geometry_msgs/Twist equivalent as JSON
            // Bridge publishes this to /cmd_vel as geometry_msgs/Twist
            this.ws.send(JSON.stringify({
                type: 'cmd_vel',
                data: {
                    linear:  { x: normalizedY * maxLinear,   y: 0.0, z: 0.0 },
                    angular: { x: 0.0, y: 0.0, z: -normalizedX * maxAngular }
                }
            }));
        }

        // ─── UI init ─────────────────────────────────────────────────────────

        initializeUI() {
            this.canvas                = this.element.querySelector('#joystickCanvas');
            this.ctx                   = this.canvas.getContext('2d');
            this.linearSpeedSlider     = this.element.querySelector('#linearSpeed');
            this.angularSpeedSlider    = this.element.querySelector('#angularSpeed');
            this.linearSpeedValueSpan  = this.element.querySelector('#linearSpeedValue');
            this.angularSpeedValueSpan = this.element.querySelector('#angularSpeedValue');
            this.joystickStatus        = this.element.querySelector('#joystickStatus');
            this.joystickControlMsg    = this.element.querySelector('#joystickControlMessage');

            const wrapper        = this.canvas.parentElement;
            this.canvas.width    = wrapper.clientWidth;
            this.canvas.height   = wrapper.clientHeight;
            this.joystickRadius  = Math.min(this.canvas.width, this.canvas.height) / 2 - 10;
            this.joystickCenterX = this.canvas.width  / 2;
            this.joystickCenterY = this.canvas.height / 2;
            this.thumbX          = this.joystickCenterX;
            this.thumbY          = this.joystickCenterY;

            this.drawJoystick();
            this.addEventListeners();
            this.updateSpeedValues();
            this.updateJoystickUIState();
        }

        // ─── Event listeners ─────────────────────────────────────────────────

        addEventListeners() {
            this.canvas.addEventListener('mousedown', this.onMouseDown);
            document.addEventListener('mousemove',   this.onMouseMove);
            document.addEventListener('mouseup',     this.onMouseUp);

            this.canvas.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.onMouseDown(e.touches[0]);
            }, { passive: false });
            document.addEventListener('touchmove', (e) => {
                e.preventDefault();
                this.onMouseMove(e.touches[0]);
            }, { passive: false });
            document.addEventListener('touchend',    this.onMouseUp);
            document.addEventListener('touchcancel', this.onMouseUp);

            this.linearSpeedSlider?.addEventListener('input',  this.updateSpeedValues);
            this.angularSpeedSlider?.addEventListener('input', this.updateSpeedValues);

            this._resizeListener = () => {
                if (!this.canvas) return;
                const wrapper        = this.canvas.parentElement;
                this.canvas.width    = wrapper.clientWidth;
                this.canvas.height   = wrapper.clientHeight;
                this.joystickRadius  = Math.min(this.canvas.width, this.canvas.height) / 2 - 10;
                this.joystickCenterX = this.canvas.width  / 2;
                this.joystickCenterY = this.canvas.height / 2;
                this.thumbX          = this.joystickCenterX;
                this.thumbY          = this.joystickCenterY;
                this.drawJoystick();
                this.publishTwist(0, 0);
            };
            window.addEventListener('resize', this._resizeListener);
        }

        removeEventListeners() {
            if (this.canvas) {
                this.canvas.removeEventListener('mousedown', this.onMouseDown);
            }
            document.removeEventListener('mousemove',   this.onMouseMove);
            document.removeEventListener('mouseup',     this.onMouseUp);
            document.removeEventListener('touchend',    this.onMouseUp);
            document.removeEventListener('touchcancel', this.onMouseUp);
            this.linearSpeedSlider?.removeEventListener('input',  this.updateSpeedValues);
            this.angularSpeedSlider?.removeEventListener('input', this.updateSpeedValues);
            if (this._resizeListener) window.removeEventListener('resize', this._resizeListener);
        }

        // ─── Joystick interaction ────────────────────────────────────────────

        onMouseDown(event) {
            if (this.currentRoverState.active_mission.toLowerCase() === 'navigation') {
                if (this.openmct?.notifications) {
                    this.openmct.notifications.warn('Joystick is disabled during Navigation mission.');
                }
                return;
            }
            this.isDragging        = true;
            this.canvas.style.cursor = 'grabbing';
            this.updateThumbPosition(event);
        }

        onMouseMove(event) {
            if (!this.isDragging) return;
            this.updateThumbPosition(event);
        }

        onMouseUp() {
            this.isDragging          = false;
            this.canvas.style.cursor = 'grab';
            this.thumbX              = this.joystickCenterX;
            this.thumbY              = this.joystickCenterY;
            this.drawJoystick();
            this.publishTwist(0, 0);   // stop the rover
        }

        updateThumbPosition(event) {
            const rect = this.canvas.getBoundingClientRect();
            let dx     = event.clientX - rect.left  - this.joystickCenterX;
            let dy     = event.clientY - rect.top   - this.joystickCenterY;
            const dist = Math.sqrt(dx * dx + dy * dy);

            if (dist > this.joystickRadius) {
                const angle = Math.atan2(dy, dx);
                this.thumbX = this.joystickCenterX + this.joystickRadius * Math.cos(angle);
                this.thumbY = this.joystickCenterY + this.joystickRadius * Math.sin(angle);
            } else {
                this.thumbX = event.clientX - rect.left;
                this.thumbY = event.clientY - rect.top;
            }

            this.drawJoystick();

            const normX =  (this.thumbX - this.joystickCenterX) / this.joystickRadius;
            const normY = -(this.thumbY - this.joystickCenterY) / this.joystickRadius;
            this.publishTwist(normX, normY);
        }

        // ─── Drawing ─────────────────────────────────────────────────────────

        drawJoystick() {
            if (!this.ctx) return;
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

            // Base circle
            this.ctx.beginPath();
            this.ctx.arc(this.joystickCenterX, this.joystickCenterY, this.joystickRadius, 0, Math.PI * 2);
            this.ctx.strokeStyle = '#666';
            this.ctx.lineWidth   = 3;
            this.ctx.stroke();
            this.ctx.fillStyle   = 'rgba(0,0,0,0.2)';
            this.ctx.fill();

            // Thumb
            const isBlocked = this.currentRoverState.active_mission.toLowerCase() === 'navigation';
            this.ctx.beginPath();
            this.ctx.arc(this.thumbX, this.thumbY, this.thumbRadius, 0, Math.PI * 2);
            this.ctx.fillStyle   = isBlocked ? '#f84632' : '#4CAF50';
            this.ctx.strokeStyle = isBlocked ? '#E53935' : '#388E3C';
            this.ctx.fill();
            this.ctx.lineWidth   = 2;
            this.ctx.stroke();
        }

        // ─── Speed sliders ───────────────────────────────────────────────────

        updateSpeedValues() {
            if (this.linearSpeedValueSpan && this.linearSpeedSlider) {
                this.linearSpeedValueSpan.textContent = parseFloat(this.linearSpeedSlider.value).toFixed(1);
            }
            if (this.angularSpeedValueSpan && this.angularSpeedSlider) {
                this.angularSpeedValueSpan.textContent = parseFloat(this.angularSpeedSlider.value).toFixed(1);
            }
        }

        // ─── UI state ────────────────────────────────────────────────────────

        updateJoystickUIState() {
            const isNavigation = this.currentRoverState.active_mission.toLowerCase() === 'navigation';
            const isActive     = this.wsConnected && !isNavigation;

            this.drawJoystick();

            if (this.joystickStatus) {
                if (!this.wsConnected) {
                    this.joystickStatus.textContent = 'Disconnected';
                    this.joystickStatus.className   = 'joystick-status error';
                } else if (isActive) {
                    this.joystickStatus.textContent = 'Joystick Active';
                    this.joystickStatus.className   = 'joystick-status connected';
                } else {
                    this.joystickStatus.textContent = `Inactive — Navigation mission running`;
                    this.joystickStatus.className   = 'joystick-status error';
                }
            }

            if (this.linearSpeedSlider)  this.linearSpeedSlider.disabled  = !isActive;
            if (this.angularSpeedSlider) this.angularSpeedSlider.disabled = !isActive;

            if (this.joystickControlMsg) {
                this.joystickControlMsg.textContent = isActive
                    ? 'Use the joystick to control rover movement.'
                    : isNavigation
                        ? "Joystick disabled during 'Navigation' mission."
                        : 'Waiting for WS connection...';
                this.joystickControlMsg.style.color = isActive ? '' : 'var(--color-error)';
            }
        }

        // ─── Destroy ─────────────────────────────────────────────────────────

        destroy() {
            console.log('[JoystickView] Destroying...');
            this.removeEventListeners();
            if (this.reconnectInterval) clearInterval(this.reconnectInterval);
            if (this.ws) {
                this.publishTwist(0, 0);  // stop rover before closing
                this.ws.close();
            }

            this.canvas = this.ctx = null;
            this.linearSpeedSlider = this.angularSpeedSlider = null;
            this.linearSpeedValueSpan = this.angularSpeedValueSpan = null;
            this.joystickStatus = this.joystickControlMsg = null;
            this.ws = null;
            this.currentRoverState = null;
            this.element.innerHTML = '';
        }
    }

    window.JoystickView = JoystickView;
})();