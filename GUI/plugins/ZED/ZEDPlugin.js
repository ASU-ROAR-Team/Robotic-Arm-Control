// plugins/zed-camera/ZEDPlugin.js
(function () {
    'use strict';

    const ZED_CAMERA_KEY = 'zed-camera';

    window.ZEDPlugin = function ZEDPlugin() {
        return function install(openmct) {

            openmct.types.addType(ZED_CAMERA_KEY, {
                name: 'ZED Camera',
                description: 'Displays the ZED 2i depth feed via the ROS2 WS bridge.',
                creatable: true,
                cssClass: 'icon-camera',
                initialize(domainObject) {
                    domainObject.throttleRate = 200; // ms — controls client-side frame drop
                },
                form: [
                    {
                        key: 'throttleRate',
                        name: 'Min ms between frames (throttle)',
                        control: 'numberfield',
                        required: false,
                        cssClass: 'l-input'
                    }
                ]
            });

            openmct.objectViews.addProvider({
                key: 'zed-camera-view',
                name: 'ZED Camera View',
                canView: (domainObject) => domainObject.type === ZED_CAMERA_KEY,

                view: (domainObject) => {
                    let imgElement          = null;
                    let errorMsgElement     = null;
                    let viewContainer       = null;
                    let snapshotButton      = null;
                    let innerCircle         = null;

                    // WebSocket
                    let ws               = null;
                    let reconnectTimer   = null;
                    let wsConnected      = false;

                    // Frame-drop state
                    let isProcessingFrame = false;
                    let lastFrameTime     = 0;
                    const getThrottle     = () => Number(domainObject.throttleRate) || 200;

                    // ── helpers ──────────────────────────────────────────────

                    const displayMessage = (message, type = 'info') => {
                        if (errorMsgElement?.parentElement) {
                            errorMsgElement.parentElement.removeChild(errorMsgElement);
                            errorMsgElement = null;
                        }
                        if (imgElement) {
                            imgElement.style.display = (type === 'error' || type === 'warning') ? 'none' : 'block';
                        }

                        errorMsgElement = document.createElement('div');
                        Object.assign(errorMsgElement.style, {
                            textAlign: 'center', marginTop: '20px',
                            padding: '10px', borderRadius: '5px'
                        });

                        if (type === 'error') {
                            errorMsgElement.style.backgroundColor = '#d9534f';
                            errorMsgElement.style.color           = 'white';
                        } else if (type === 'warning') {
                            errorMsgElement.style.backgroundColor = '#f0ad4e';
                            errorMsgElement.style.color           = 'black';
                        } else {
                            errorMsgElement.style.backgroundColor = '#d9edf7';
                            errorMsgElement.style.color           = 'black';
                        }

                        errorMsgElement.textContent = message;
                        (viewContainer || document.body).appendChild(errorMsgElement);
                    };

                    const clearMessage = () => {
                        if (errorMsgElement?.parentElement) {
                            errorMsgElement.parentElement.removeChild(errorMsgElement);
                            errorMsgElement = null;
                        }
                    };

                    // ── WebSocket ─────────────────────────────────────────────

                    const initWS = () => {
                        if (ws && ws.readyState !== WebSocket.CLOSED) return;

                        ws = new WebSocket('ws://localhost:8080');

                        ws.onopen = () => {
                            console.log('[ZEDPlugin] Connected to WS bridge');
                            wsConnected = true;
                            if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
                            displayMessage('Connected. Waiting for ZED image stream...', 'info');
                        };

                        ws.onmessage = (event) => {
                            try {
                                const msg = JSON.parse(event.data);
                                if (msg.type !== 'zed_frame') return;

                                const now = Date.now();

                                // Client-side frame dropping
                                if (isProcessingFrame) return;
                                if ((now - lastFrameTime) < getThrottle() * 0.8) return;

                                isProcessingFrame = true;
                                lastFrameTime     = now;

                                try {
                                    const data = typeof msg.data === 'string'
                                        ? JSON.parse(msg.data) : msg.data;

                                    if (data?.data && imgElement) {
                                        imgElement.src          = `data:image/jpeg;base64,${data.data}`;
                                        imgElement.style.display = 'block';
                                        if (snapshotButton) snapshotButton.style.display = 'block';
                                        clearMessage();
                                    }
                                } finally {
                                    isProcessingFrame = false;
                                }

                            } catch (e) {
                                console.error('[ZEDPlugin] Frame parse error:', e);
                                isProcessingFrame = false;
                            }
                        };

                        ws.onclose = () => {
                            console.warn('[ZEDPlugin] Disconnected. Reconnecting in 3s...');
                            wsConnected = false;
                            isProcessingFrame = false;
                            if (imgElement)     imgElement.style.display     = 'none';
                            if (snapshotButton) snapshotButton.style.display = 'none';
                            displayMessage('ZED Camera: Disconnected. Reconnecting...', 'error');
                            scheduleReconnect();
                        };

                        ws.onerror = (err) => {
                            console.error('[ZEDPlugin] WS error', err);
                            ws.close();
                        };
                    };

                    const scheduleReconnect = () => {
                        if (reconnectTimer) return;
                        reconnectTimer = setInterval(() => {
                            console.log('[ZEDPlugin] Attempting reconnect...');
                            initWS();
                        }, 3000);
                    };

                    const stopWS = () => {
                        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
                        if (ws) { ws.close(); ws = null; }
                        wsConnected = false;
                    };

                    // ── Snapshot ──────────────────────────────────────────────

                    const takeSnapshot = () => {
                        if (!imgElement?.src) {
                            openmct.notifications.error('Snapshot failed: No image available.');
                            return;
                        }
                        try {
                            const link      = document.createElement('a');
                            link.href       = imgElement.src;
                            link.download   = `zed-snapshot-${Date.now()}.jpeg`;
                            document.body.appendChild(link);
                            link.click();
                            document.body.removeChild(link);
                            openmct.notifications.info('Snapshot captured!');
                        } catch (e) {
                            openmct.notifications.error('Snapshot failed: ' + e.message);
                        }
                    };

                    // ── View lifecycle ────────────────────────────────────────

                    return {
                        show(element) {
                            viewContainer = element;

                            const container = document.createElement('div');
                            Object.assign(container.style, {
                                width: '100%', height: '100%', position: 'relative'
                            });

                            imgElement = document.createElement('img');
                            Object.assign(imgElement.style, {
                                width: '100%', height: '100%',
                                objectFit: 'contain', display: 'none'
                            });
                            imgElement.loading  = 'eager';
                            imgElement.decoding = 'async';
                            container.appendChild(imgElement);

                            // Snapshot button (outer ring)
                            snapshotButton = document.createElement('button');
                            Object.assign(snapshotButton.style, {
                                position: 'absolute', bottom: '15px',
                                left: '50%', transform: 'translateX(-50%)',
                                width: '60px', height: '60px',
                                backgroundColor: 'transparent',
                                border: '2px solid white', borderRadius: '50%',
                                cursor: 'pointer', display: 'none',
                                boxShadow: '0 4px 8px rgba(0,0,0,0.3)', outline: 'none'
                            });

                            snapshotButton.addEventListener('mousedown',  () => {
                                snapshotButton.style.transform  = 'translateX(-50%) scale(0.95)';
                                snapshotButton.style.boxShadow  = '0 2px 4px rgba(0,0,0,0.2)';
                            });
                            snapshotButton.addEventListener('mouseup',    () => {
                                snapshotButton.style.transform  = 'translateX(-50%) scale(1)';
                                snapshotButton.style.boxShadow  = '0 4px 8px rgba(0,0,0,0.3)';
                            });
                            snapshotButton.addEventListener('mouseleave', () => {
                                snapshotButton.style.transform  = 'translateX(-50%) scale(1)';
                                snapshotButton.style.boxShadow  = '0 4px 8px rgba(0,0,0,0.3)';
                            });
                            snapshotButton.addEventListener('click', takeSnapshot);

                            // Inner white circle
                            innerCircle = document.createElement('div');
                            Object.assign(innerCircle.style, {
                                width: '45px', height: '45px',
                                backgroundColor: 'white', borderRadius: '50%',
                                position: 'absolute', top: '50%', left: '50%',
                                transform: 'translate(-50%, -50%)', boxSizing: 'border-box'
                            });
                            snapshotButton.appendChild(innerCircle);
                            container.appendChild(snapshotButton);

                            element.appendChild(container);
                            initWS();
                        },

                        onEditModeChange(editMode) {
                            if (editMode) {
                                stopWS();
                                if (imgElement)     imgElement.style.display     = 'none';
                                if (snapshotButton) snapshotButton.style.display = 'none';
                                displayMessage('ZED Camera: Edit mode — stream paused.', 'info');
                            } else {
                                initWS();
                            }
                        },

                        destroy() {
                            stopWS();
                            snapshotButton?.removeEventListener('click', takeSnapshot);
                            if (snapshotButton?.parentElement) snapshotButton.parentElement.removeChild(snapshotButton);
                            if (innerCircle?.parentElement)    innerCircle.parentElement.removeChild(innerCircle);
                            if (imgElement?.parentElement)     imgElement.parentElement.removeChild(imgElement);
                            if (errorMsgElement?.parentElement) errorMsgElement.parentElement.removeChild(errorMsgElement);

                            imgElement = snapshotButton = innerCircle = errorMsgElement = viewContainer = null;
                            console.log('[ZEDPlugin] View destroyed.');
                        }
                    };
                }
            });

            return { destroy: () => {} };
        };
    };

})();