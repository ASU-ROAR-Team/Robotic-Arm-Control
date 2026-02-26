// plugins/display/CostmapPlugin.js
(function () {
    'use strict';

    const COSTMAP_MAP_KEY = 'costmap-map';

    window.CostmapMapPlugin = function CostmapMapPlugin(options) {
        return function install(openmct) {

            openmct.types.addType(COSTMAP_MAP_KEY, {
                name: 'Costmap Map',
                description: 'Displays a map based on a costmap and visualizes robot state.',
                creatable: true,
                cssClass: 'icon-map',
                initialize(domainObject) {
                    domainObject.costmapSourceUrl  = options?.defaultCostmapSourceUrl  || '';
                    domainObject.pixelOffsetX      = options?.defaultPixelOffsetX      || 208;
                    domainObject.pixelOffsetY      = options?.defaultPixelOffsetY      || 761;
                    domainObject.pixelsPerMeter    = options?.defaultPixelsPerMeter    || 20.2;
                    domainObject.startPointData    = options?.startPointData           || [18.6625, 10.8159];
                    domainObject.checkpointsData   = options?.checkpointsData          || [];
                    domainObject.finalGoalData     = options?.finalGoalData            || null;
                    domainObject.landmarksData     = options?.landmarksData            || [];
                },
                form: [
                    { key: 'costmapSourceUrl', name: 'Costmap Source URL (Image or CSV)', control: 'textfield',   required: true,  cssClass: 'l-input' },
                    { key: 'pixelOffsetX',     name: 'Pixel Offset X',                   control: 'numberfield', required: true,  cssClass: 'l-input' },
                    { key: 'pixelOffsetY',     name: 'Pixel Offset Y',                   control: 'numberfield', required: true,  cssClass: 'l-input' },
                    { key: 'pixelsPerMeter',   name: 'Pixels Per Meter',                 control: 'numberfield', required: true,  cssClass: 'l-input' },
                    { key: 'startPointData',   name: 'Start Point (JSON array [x, y])',  control: 'textfield',   required: false, cssClass: 'l-input' },
                    { key: 'checkpointsData',  name: 'Checkpoints (JSON array of [x,y])',control: 'textarea',    required: false, cssClass: 'l-input' },
                    { key: 'finalGoalData',    name: 'Final Goal (JSON array [x, y])',   control: 'textfield',   required: false, cssClass: 'l-input' },
                    { key: 'landmarksData',    name: 'Landmarks (JSON array of [x, y])', control: 'textarea',    required: false, cssClass: 'l-input' }
                ]
            });

            openmct.objectViews.addProvider({
                key: 'costmap-map-view',
                name: 'Costmap Map View',
                canView: (domainObject) => domainObject.type === COSTMAP_MAP_KEY,
                view: (domainObject) => {
                    let mapComponent = null;
                    return {
                        show(element) {
                            mapComponent = new CostmapMapComponent(element, domainObject, openmct);
                            mapComponent.render();
                        },
                        onEditModeChange() {},
                        destroy() {
                            if (mapComponent) { mapComponent.destroy(); mapComponent = null; }
                        }
                    };
                }
            });

            return { destroy: () => {} };
        };
    };


    // ═══════════════════════════════════════════════════════════════════════
    //  CostmapMapComponent
    // ═══════════════════════════════════════════════════════════════════════

    class CostmapMapComponent {
        constructor(parentElement, domainObject, openmct) {
            this.parentElement  = parentElement;
            this.domainObject   = domainObject;
            this.openmct        = openmct;

            this.canvas              = null;
            this.ctx                 = null;
            this.costmapSourceType   = null;
            this.costmapImage        = null;
            this.costmapData         = null;
            this.mapWidthPixels      = 0;
            this.mapHeightPixels     = 0;
            this.minCost             = 0;
            this.maxCost             = 0;

            // Coordinate transform constants
            this.pixelOffsetX   = domainObject.pixelOffsetX;
            this.pixelOffsetY   = domainObject.pixelOffsetY;
            this.pixelsPerMeter = domainObject.pixelsPerMeter;

            // WebSocket
            this.ws               = null;
            this.reconnectInterval = null;
            this.wsConnected      = false;

            // Dynamic data
            this.robotPosition  = null;
            this.globalPath     = [];
            this.traversedPath  = [];
            this.lookaheadPoint = null;
            this.obstacles      = new Map();

            // Static points from properties
            this.startPoint  = this._parsePoint(domainObject.startPointData);
            this.checkpoints = this._parsePointArray(domainObject.checkpointsData);
            this.finalGoal   = this._parsePoint(domainObject.finalGoalData);
            this.landmarks   = this._parsePointArray(domainObject.landmarksData);

            // Rate limiting / staleness
            this.lastProcessedPoseTime        = 0;
            this.minProcessingIntervalMs      = 33;   // ~30 fps
            this.lastRobotPoseReceiveTime     = 0;
            this.lastGlobalPathReceiveTime    = 0;
            this.lastTraversedPathReceiveTime = 0;
            this.lastObstaclesReceiveTime     = 0;
            this.dataStaleTimeoutMs           = 2000;

            this.animationFrameId = null;
            this.resizeHandler    = this.handleResize.bind(this);
        }

        // ── Data parsers ─────────────────────────────────────────────────────

        _parsePoint(data) {
            try {
                const arr = typeof data === 'string' ? JSON.parse(data) : data;
                if (Array.isArray(arr) && arr.length === 2 &&
                    typeof arr[0] === 'number' && typeof arr[1] === 'number') {
                    return { x: arr[0], y: arr[1] };
                }
            } catch (e) { console.error('CostmapPlugin: Error parsing point:', e, data); }
            return null;
        }

        _parsePointArray(data) {
            try {
                const arr = typeof data === 'string' ? JSON.parse(data) : data;
                if (Array.isArray(arr) &&
                    arr.every(p => Array.isArray(p) && p.length === 2 &&
                                   typeof p[0] === 'number' && typeof p[1] === 'number')) {
                    return arr.map(p => ({ x: p[0], y: p[1] }));
                }
            } catch (e) { console.error('CostmapPlugin: Error parsing point array:', e, data); }
            return [];
        }

        // ── WebSocket ─────────────────────────────────────────────────────────

        initWS() {
            if (this.ws && this.ws.readyState !== WebSocket.CLOSED) return;

            this.ws = new WebSocket('ws://localhost:8080');

            this.ws.onopen = () => {
                console.log('[CostmapPlugin] Connected to WS bridge');
                this.wsConnected = true;
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
                        case 'robot_pose':      this.handleRobotPose(data);      break;
                        case 'global_path':     this.handleGlobalPath(data);     break;
                        case 'traversed_path':  this.handleTraversedPath(data);  break;
                        case 'obstacle':        this.handleObstacle(data);       break;
                    }
                } catch (e) {
                    console.error('[CostmapPlugin] Failed to parse message', e);
                }
            };

            this.ws.onclose = () => {
                console.warn('[CostmapPlugin] Disconnected. Reconnecting in 3s...');
                this.wsConnected = false;
                this.clearDynamicData();
                this.scheduleReconnect();
            };

            this.ws.onerror = (err) => {
                console.error('[CostmapPlugin] WS error', err);
                this.ws.close();
            };
        }

        scheduleReconnect() {
            if (this.reconnectInterval) return;
            this.reconnectInterval = setInterval(() => {
                console.log('[CostmapPlugin] Attempting reconnect...');
                this.initWS();
            }, 3000);
        }

        // ── Inbound message handlers ──────────────────────────────────────────

        handleRobotPose(msg) {
            // nav_msgs/Odometry equivalent JSON
            const now = performance.now();
            if (now - this.lastProcessedPoseTime < this.minProcessingIntervalMs) {
                this.lastRobotPoseReceiveTime = now;
                return;
            }
            this.lastProcessedPoseTime = now;

            try {
                if (msg?.pose?.pose) {
                    const pos = msg.pose.pose.position;
                    const q   = msg.pose.pose.orientation;

                    // Quaternion → yaw
                    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                    const yaw       = Math.atan2(siny_cosp, cosy_cosp);

                    this.robotPosition            = { x: pos.x, y: pos.y, theta: yaw };
                    this.lastRobotPoseReceiveTime = now;
                } else {
                    this.robotPosition            = null;
                    this.lastRobotPoseReceiveTime = 0;
                }
            } catch (e) {
                console.error('[CostmapPlugin] Error processing robot pose:', e);
                this.robotPosition = null;
            }
        }

        handleGlobalPath(msg) {
            // nav_msgs/Path equivalent JSON
            const now = performance.now();
            if (Array.isArray(msg?.poses) && msg.poses.length > 0) {
                this.globalPath                = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
                this.lastGlobalPathReceiveTime = now;
            } else {
                this.globalPath                = [];
                this.lastGlobalPathReceiveTime = 0;
            }
        }

        handleTraversedPath(msg) {
            // nav_msgs/Path equivalent JSON
            const now = performance.now();
            if (Array.isArray(msg?.poses) && msg.poses.length > 0) {
                this.traversedPath                = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
                this.lastTraversedPathReceiveTime = now;
            } else {
                this.traversedPath                = [];
                this.lastTraversedPathReceiveTime = 0;
            }
        }

        handleObstacle(msg) {
            // visualization_msgs/Marker equivalent JSON
            // replaces roar_msgs/Obstacle:
            //   msg.id.data        → msg.id
            //   msg.position.pose  → msg.pose
            //   msg.radius.data    → msg.scale.x / 2
            const now = performance.now();
            try {
                if (msg?.id !== undefined && msg?.pose?.position && msg?.scale) {
                    this.obstacles.set(msg.id, {
                        x:      msg.pose.position.x,
                        y:      msg.pose.position.y,
                        radius: msg.scale.x / 2      // Marker scale = diameter
                    });
                    this.lastObstaclesReceiveTime = now;
                } else {
                    console.warn('[CostmapPlugin] Received obstacle with missing fields:', msg);
                }
            } catch (e) {
                console.error('[CostmapPlugin] Error processing obstacle:', e);
            }
        }

        // ── Render ────────────────────────────────────────────────────────────

        render() {
            this.canvas             = document.createElement('canvas');
            this.canvas.style.width  = '100%';
            this.canvas.style.height = '100%';
            this.parentElement.appendChild(this.canvas);
            this.ctx = this.canvas.getContext('2d');

            this.handleResize();
            window.addEventListener('resize', this.resizeHandler);

            const url = this.domainObject.costmapSourceUrl;
            if (url?.toLowerCase().endsWith('.csv')) {
                this.costmapSourceType = 'csv';
                this.loadCostmapData(url);
            } else if (url) {
                this.costmapSourceType = 'image';
                this.loadCostmapImage();
            } else {
                console.error('[CostmapPlugin] Costmap Source URL not provided.');
                this.ctx.fillStyle = 'red';
                this.ctx.font      = '20px Arial';
                this.ctx.fillText('Costmap Source URL not set.', 10, 30);
            }

            this.initWS();
            this.animationFrameId = requestAnimationFrame(this.animate.bind(this));
        }

        animate() {
            this.checkStaleData();
            this.drawMap();
            this.animationFrameId = requestAnimationFrame(this.animate.bind(this));
        }

        // ── Map loading ───────────────────────────────────────────────────────

        handleResize() {
            const rect = this.canvas.parentElement.getBoundingClientRect();
            this.canvas.width = rect.width;
            if (this.mapWidthPixels > 0 && this.mapHeightPixels > 0) {
                this.canvas.height = rect.width * (this.mapHeightPixels / this.mapWidthPixels);
                if (this.canvas.height > rect.height) {
                    this.canvas.height = rect.height;
                    this.canvas.width  = rect.height * (this.mapWidthPixels / this.mapHeightPixels);
                }
            } else {
                this.canvas.height = rect.height;
            }
            this.drawMap();
        }

        loadCostmapImage() {
            this.costmapImage     = new Image();
            this.costmapImage.src = this.domainObject.costmapSourceUrl;
            this.costmapImage.onload = () => {
                this.mapWidthPixels  = this.costmapImage.width;
                this.mapHeightPixels = this.costmapImage.height;
                this.handleResize();
            };
            this.costmapImage.onerror = (e) => {
                console.error('[CostmapPlugin] Error loading costmap image:', e);
            };
        }

        async loadCostmapData(csvUrl) {
            try {
                const response = await fetch(csvUrl);
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                const text = await response.text();

                this.costmapData     = text.trim().split('\n').map(row => row.split(',').map(Number));
                this.mapHeightPixels = this.costmapData.length;
                this.mapWidthPixels  = this.mapHeightPixels > 0 ? this.costmapData[0].length : 0;

                this.minCost = Infinity; this.maxCost = -Infinity;
                for (const row of this.costmapData)
                    for (const v of row) {
                        if (v < this.minCost) this.minCost = v;
                        if (v > this.maxCost) this.maxCost = v;
                    }

                this.handleResize();
            } catch (e) {
                console.error('[CostmapPlugin] Error loading CSV:', e);
            }
        }

        getCostColor(v) {
            if (this.maxCost <= this.minCost) return 'rgb(128,128,128)';
            const n = (v - this.minCost) / (this.maxCost - this.minCost);
            return `rgb(${Math.floor(n * 255)},0,${Math.floor((1 - n) * 255)})`;
        }

        // ── Coordinate transforms ─────────────────────────────────────────────

        realToPixelX(x) { return this.pixelOffsetX + this.pixelsPerMeter * x; }
        realToPixelY(y) { return this.pixelOffsetY - this.pixelsPerMeter * y; }

        // ── Staleness check ───────────────────────────────────────────────────

        checkStaleData() {
            const now = performance.now();
            if (this.robotPosition && now - this.lastRobotPoseReceiveTime > this.dataStaleTimeoutMs) {
                this.robotPosition = null; this.lastRobotPoseReceiveTime = 0;
            }
            if (this.globalPath.length > 0 && now - this.lastGlobalPathReceiveTime > this.dataStaleTimeoutMs) {
                this.globalPath = []; this.lastGlobalPathReceiveTime = 0;
            }
            if (this.traversedPath.length > 0 && now - this.lastTraversedPathReceiveTime > this.dataStaleTimeoutMs) {
                this.traversedPath = []; this.lastTraversedPathReceiveTime = 0;
            }
            if (this.obstacles.size > 0 && now - this.lastObstaclesReceiveTime > this.dataStaleTimeoutMs) {
                this.obstacles.clear(); this.lastObstaclesReceiveTime = 0;
            }
        }

        clearDynamicData() {
            this.robotPosition = null; this.globalPath = []; this.traversedPath = [];
            this.lookaheadPoint = null; this.obstacles.clear();
            this.lastRobotPoseReceiveTime = this.lastGlobalPathReceiveTime =
            this.lastTraversedPathReceiveTime = this.lastObstaclesReceiveTime = 0;
        }

        // ── Drawing ───────────────────────────────────────────────────────────

        drawMap() {
            if (!this.ctx || !this.canvas) return;
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

            const scaleX      = this.mapWidthPixels  > 0 ? this.canvas.width  / this.mapWidthPixels  : 1;
            const scaleY      = this.mapHeightPixels > 0 ? this.canvas.height / this.mapHeightPixels : 1;
            const markerScale = Math.min(scaleX, scaleY);

            // Background
            if (this.costmapSourceType === 'image' && this.costmapImage?.complete) {
                this.ctx.drawImage(this.costmapImage, 0, 0, this.canvas.width, this.canvas.height);
            } else if (this.costmapSourceType === 'csv' && this.costmapData && this.mapWidthPixels > 0) {
                for (let y = 0; y < this.mapHeightPixels; y++)
                    for (let x = 0; x < this.mapWidthPixels; x++) {
                        this.ctx.fillStyle = this.getCostColor(this.costmapData[y][x]);
                        this.ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
                    }
            } else {
                this.ctx.fillStyle = 'gray';
                this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
                this.ctx.fillStyle = 'white'; this.ctx.font = '20px Arial';
                this.ctx.fillText('Loading map data...', 10, 30);
            }

            const px = (rx) => this.realToPixelX(rx) * scaleX;
            const py = (ry) => this.realToPixelY(ry) * scaleY;

            // Global path
            if (this.globalPath.length > 1) {
                this.ctx.strokeStyle = 'yellow'; this.ctx.lineWidth = 2;
                this.ctx.beginPath();
                this.ctx.moveTo(px(this.globalPath[0].x), py(this.globalPath[0].y));
                this.globalPath.slice(1).forEach(p => this.ctx.lineTo(px(p.x), py(p.y)));
                this.ctx.stroke();
            }

            // Traversed path
            if (this.traversedPath.length > 1) {
                this.ctx.strokeStyle = 'blue'; this.ctx.lineWidth = 2;
                this.ctx.beginPath();
                this.ctx.moveTo(px(this.traversedPath[0].x), py(this.traversedPath[0].y));
                this.traversedPath.slice(1).forEach(p => this.ctx.lineTo(px(p.x), py(p.y)));
                this.ctx.stroke();
            }

            // Robot
            if (this.robotPosition) {
                const rx = px(this.robotPosition.x), ry = py(this.robotPosition.y);
                this.ctx.fillStyle = 'red';
                this.ctx.beginPath();
                this.ctx.arc(rx, ry, 5 * markerScale, 0, Math.PI * 2);
                this.ctx.fill();
                const ll = 10 * markerScale;
                this.ctx.strokeStyle = 'black'; this.ctx.lineWidth = 2 * markerScale;
                this.ctx.beginPath();
                this.ctx.moveTo(rx, ry);
                this.ctx.lineTo(rx + ll * Math.cos(this.robotPosition.theta),
                                ry - ll * Math.sin(this.robotPosition.theta));
                this.ctx.stroke();
            }

            // Lookahead
            if (this.lookaheadPoint) {
                const s = 8 * markerScale;
                this.ctx.fillStyle = 'cyan';
                this.ctx.fillRect(px(this.lookaheadPoint.x) - s/2, py(this.lookaheadPoint.y) - s/2, s, s);
            }

            // Obstacles (visualization_msgs/Marker)
            this.obstacles.forEach(ob => {
                this.ctx.fillStyle = 'rgba(255,0,0,0.5)';
                this.ctx.beginPath();
                this.ctx.arc(px(ob.x), py(ob.y), ob.radius * this.pixelsPerMeter * markerScale, 0, Math.PI * 2);
                this.ctx.fill();
            });

            // Start point
            if (this.startPoint) {
                const r = Math.max(3, 6 * markerScale * 0.7);
                this.ctx.fillStyle = '#00FF00'; this.ctx.strokeStyle = 'black';
                this.ctx.lineWidth = Math.max(1, 2 * markerScale * 0.5);
                this.ctx.beginPath();
                this.ctx.arc(px(this.startPoint.x), py(this.startPoint.y), r, 0, Math.PI * 2);
                this.ctx.fill(); this.ctx.stroke();
            }

            // Checkpoints
            this.checkpoints.forEach(cp => {
                const s = Math.max(4, 10 * markerScale * 0.6);
                this.ctx.fillStyle = 'magenta';
                this.ctx.beginPath();
                this.ctx.moveTo(px(cp.x), py(cp.y) - s/2);
                this.ctx.lineTo(px(cp.x) - s/2, py(cp.y) + s/2);
                this.ctx.lineTo(px(cp.x) + s/2, py(cp.y) + s/2);
                this.ctx.closePath(); this.ctx.fill();
            });

            // Landmarks
            this.landmarks.forEach(lm => {
                const s = Math.max(3, 8 * markerScale * 0.6);
                this.ctx.fillStyle = 'green';
                this.ctx.fillRect(px(lm.x) - s/2, py(lm.y) - s/2, s, s);
            });

            // Final goal
            if (this.finalGoal) {
                const s  = Math.max(6, 14 * markerScale * 0.6);
                const fx = px(this.finalGoal.x), fy = py(this.finalGoal.y);
                this.ctx.strokeStyle = 'lime';
                this.ctx.lineWidth   = Math.max(1, 2 * markerScale * 0.4);
                this.ctx.beginPath(); this.ctx.moveTo(fx - s/2, fy - s/2); this.ctx.lineTo(fx + s/2, fy + s/2); this.ctx.stroke();
                this.ctx.beginPath(); this.ctx.moveTo(fx + s/2, fy - s/2); this.ctx.lineTo(fx - s/2, fy + s/2); this.ctx.stroke();
            }
        }

        // ── Destroy ───────────────────────────────────────────────────────────

        destroy() {
            window.removeEventListener('resize', this.resizeHandler);
            if (this.animationFrameId) cancelAnimationFrame(this.animationFrameId);
            if (this.reconnectInterval) clearInterval(this.reconnectInterval);
            if (this.ws) this.ws.close();
            this.clearDynamicData();
            if (this.canvas?.parentElement) this.canvas.parentElement.removeChild(this.canvas);
            this.canvas = this.ctx = null;
        }
    }

})();