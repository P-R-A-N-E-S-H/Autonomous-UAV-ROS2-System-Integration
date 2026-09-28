/**
 * mission_visualizer.js
 * Real-time 2D/3D Drone Flight Path & Geofence Canvas Renderer.
 * Role: Member 3 - ROS2 & System Integration Engineer.
 */

class MissionVisualizer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.isIsometric = false;
    this.scale = 24.0; // pixels per meter
    this.originX = this.canvas.width / 2;
    this.originY = this.canvas.height / 2;

    this.dronePos = { x: 0, y: 0, z: 0 };
    this.droneYaw = 0;
    this.flightPath = [];
    this.waypoints = [];
    this.rotorAngle = 0;
    this.targetPos = null;

    // Resize handling
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    this.canvas.width = rect.width;
    this.canvas.height = rect.height;
    this.originX = this.canvas.width / 2;
    this.originY = this.canvas.height / 2;
  }

  updateData(telemetry) {
    if (!telemetry) return;
    this.dronePos = telemetry.drone.position;
    this.droneYaw = (telemetry.drone.orientation.yaw_deg * Math.PI) / 180;
    this.flightPath = telemetry.flight_path || [];
    this.waypoints = telemetry.waypoints || [];
    this.targetPos = telemetry.fsm.target_pose;
    this.rotorAngle += 0.4;
  }

  worldToScreen(x, y, z = 0) {
    if (!this.isIsometric) {
      // Top-Down: X -> Screen Right, Y -> Screen Up
      return {
        sx: this.originX + x * this.scale,
        sy: this.originY - y * this.scale
      };
    } else {
      // Isometric 2.5D Projection
      const isoX = (x - y) * Math.cos(Math.PI / 6);
      const isoY = (x + y) * Math.sin(Math.PI / 6) - z * 0.8;
      return {
        sx: this.originX + isoX * this.scale,
        sy: this.originY + isoY * this.scale
      };
    }
  }

  render() {
    const { ctx, canvas } = this;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Draw Grid Background
    this.drawGrid();

    // 2. Draw Geofence Boundary Circle (25m radius)
    this.drawGeofence();

    // 3. Draw Waypoints & Objects
    this.drawWaypoints();

    // 4. Draw Flight Trail
    this.drawFlightPath();

    // 5. Draw Target Line
    this.drawTargetVector();

    // 6. Draw Drone Quadrotor Body & Camera Frustum
    this.drawDrone();
  }

  drawGrid() {
    const { ctx, canvas, scale } = this;
    ctx.save();
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.06)';
    ctx.lineWidth = 1;

    const step = scale * 2; // 2 meter grid
    for (let x = (this.originX % step); x < canvas.width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }
    for (let y = (this.originY % step); y < canvas.height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }

    // Origin Axes
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(this.originX, 0);
    ctx.lineTo(this.originX, canvas.height);
    ctx.moveTo(0, this.originY);
    ctx.lineTo(canvas.width, this.originY);
    ctx.stroke();
    ctx.restore();
  }

  drawGeofence() {
    const { ctx, scale } = this;
    const center = this.worldToScreen(0, 0);
    const radius = 20.0 * scale;

    ctx.save();
    ctx.strokeStyle = 'rgba(239, 68, 68, 0.35)';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.arc(center.sx, center.sy, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = 'rgba(239, 68, 68, 0.6)';
    ctx.font = '10px Rajdhani';
    ctx.fillText('GEOFENCE PERIMETER (R=20m)', center.sx + radius - 140, center.sy - 8);
    ctx.restore();
  }

  drawWaypoints() {
    const { ctx } = this;
    this.waypoints.forEach(wp => {
      const pt = this.worldToScreen(wp.x, wp.y, wp.z);

      ctx.save();
      // Outer glow ring
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(pt.sx, pt.sy, 8, 0, Math.PI * 2);
      ctx.stroke();

      // Inner icon dot
      ctx.fillStyle = '#38bdf8';
      ctx.beginPath();
      ctx.arc(pt.sx, pt.sy, 3, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px Rajdhani';
      ctx.fillText(`WP${wp.id}: ${wp.name}`, pt.sx + 12, pt.sy + 3);
      ctx.restore();
    });
  }

  drawFlightPath() {
    const { ctx } = this;
    if (this.flightPath.length < 2) return;

    ctx.save();
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();

    const start = this.worldToScreen(this.flightPath[0][0], this.flightPath[0][1], this.flightPath[0][2]);
    ctx.moveTo(start.sx, start.sy);

    for (let i = 1; i < this.flightPath.length; i++) {
      const pt = this.worldToScreen(this.flightPath[i][0], this.flightPath[i][1], this.flightPath[i][2]);
      ctx.lineTo(pt.sx, pt.sy);
    }
    ctx.stroke();
    ctx.restore();
  }

  drawTargetVector() {
    if (!this.targetPos) return;
    const { ctx } = this;
    const drone = this.worldToScreen(this.dronePos.x, this.dronePos.y, this.dronePos.z);
    const target = this.worldToScreen(this.targetPos.x, this.targetPos.y, this.targetPos.z);

    ctx.save();
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.5)';
    ctx.setLineDash([4, 4]);
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(drone.sx, drone.sy);
    ctx.lineTo(target.sx, target.sy);
    ctx.stroke();

    // Target reticle
    ctx.strokeStyle = '#f59e0b';
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.arc(target.sx, target.sy, 12, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }

  drawDrone() {
    const { ctx } = this;
    const center = this.worldToScreen(this.dronePos.x, this.dronePos.y, this.dronePos.z);

    ctx.save();
    ctx.translate(center.sx, center.sy);
    ctx.rotate(-this.droneYaw); // Canvas rotates clockwise

    // 1. Camera FOV Cone
    ctx.fillStyle = 'rgba(56, 189, 248, 0.12)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(40, -22);
    ctx.lineTo(40, 22);
    ctx.closePath();
    ctx.fill();

    // 2. Quadrotor X Arms
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 3;
    const armLen = 14;
    ctx.beginPath();
    ctx.moveTo(-armLen, -armLen);
    ctx.lineTo(armLen, armLen);
    ctx.moveTo(-armLen, armLen);
    ctx.lineTo(armLen, -armLen);
    ctx.stroke();

    // 3. Central Drone Body Hub
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Heading Arrow
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.moveTo(0, -6);
    ctx.lineTo(8, 0);
    ctx.lineTo(0, 6);
    ctx.fill();

    // 4. 4 Spinning Rotors
    const motorOffsets = [
      [-armLen, -armLen], [armLen, -armLen],
      [-armLen, armLen], [armLen, armLen]
    ];
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.lineWidth = 1.5;
    motorOffsets.forEach(([mx, my], idx) => {
      ctx.beginPath();
      const angle = this.rotorAngle * (idx % 2 === 0 ? 1 : -1);
      ctx.arc(mx, my, 5, angle, angle + Math.PI);
      ctx.stroke();
    });

    ctx.restore();
  }
}
