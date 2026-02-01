import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.widgets import Button

class Dashboard:
    
    def __init__(self, track, car, result):
        self.track = track
        self.car = car
        self.result = result
        if result is None: raise ValueError("Result is None")
        
        self._precompute()
        
        self.cursor_idx = 0
        self.markers = {}
        self.vlines = {}
        
    def _precompute(self):
        s_vals = self.result['s']
        n_vals = self.result['n']
        v_vals = self.result['v']
        a_vals = self.result['a'] # 运动学加速度 dv/dt
        kappa_vals = self.result['kappa']
        
        N = len(s_vals)
        self.N = N
        self.s = s_vals
        
        # 1. 补齐数组长度
        if len(a_vals) < N: a_vals = np.append(a_vals, a_vals[-1])
        if len(kappa_vals) < N: kappa_vals = np.append(kappa_vals, kappa_vals[-1])
        
        # 2. 轨迹坐标与几何
        self.path_x = np.zeros(N)
        self.path_y = np.zeros(N)
        self.path_z = np.zeros(N)
        self.slope = np.zeros(N)
        self.bank = np.zeros(N)
        
        for i, (s, n) in enumerate(zip(s_vals, n_vals)):
            x, y, z = self.track.get_position(s, n)
            self.path_x[i] = x
            self.path_y[i] = y
            self.path_z[i] = z
            self.slope[i] = np.degrees(self.track.get_slope(s))
            self.bank[i] = np.degrees(self.track.get_banking(s))
            
        # 3. 基础物理数据
        g = 9.81
        slope_rad = np.radians(self.slope)
        bank_rad = np.radians(self.bank)
        
        self.g_long = (a_vals + g * np.sin(slope_rad)) / g
        self.g_lat = (v_vals**2 * kappa_vals + g * np.sin(bank_rad)) / g
        self.speed_kmh = v_vals * 3.6
        self.lap_time = self.result['time']

        # 4. 估算油门和刹车开度 (F = ma 反推)
        p = self.car.params
        mass = p['mass']
        
        # 空气阻力
        F_aero_drag = 0.5 * p['rho'] * p['Cd'] * p['A'] * v_vals**2
        # 空气下压力
        F_aero_down = 0.5 * p['rho'] * p['Cl'] * p['A'] * v_vals**2
        F_z = mass * g + F_aero_down
        
        # 坡度重力分量
        F_grav = mass * g * np.sin(slope_rad)
        
        # 净需求力 (F_net = ma)
        F_net = mass * a_vals
        
        # 轮胎需要产生的纵向力 = 净需求力 + 阻力 + 重力
        F_req = F_net + F_aero_drag + F_grav
        
        self.throttle_pct = np.zeros(N)
        self.brake_pct = np.zeros(N)
        
        # 车辆能力限制
        v_safe = np.maximum(v_vals, 1.0)
        P_limit_force = p['P_max'] / v_safe
        F_drive_max = np.minimum(p['force_max'], P_limit_force)
        F_brake_max = p['mu'] * F_z # 简化的最大刹车力
        
        for i in range(N):
            f = F_req[i]
            if f > 0:
                # 油门
                self.throttle_pct[i] = min(100.0, (f / F_drive_max[i]) * 100.0)
            else:
                # 刹车
                self.brake_pct[i] = min(100.0, (-f / F_brake_max[i]) * 100.0)

        # 5. 统计极值
        self.stats = {
            'z_min': np.min(self.path_z), 'z_max': np.max(self.path_z),
            'v_max': np.max(self.speed_kmh),
            'g_lim': max(np.max(np.abs(self.g_lat)), np.max(np.abs(self.g_long))) * 1.1
        }
        
        # 6. 路径长度
        self.centerline_len = self.track.total_length
        dx = np.diff(self.path_x)
        dy = np.diff(self.path_y)
        dz = np.diff(self.path_z)
        self.racing_line_len = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))

    def show(self):
        """构建并显示界面"""
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 9), facecolor='white')
        self.fig.canvas.manager.set_window_title(f"Racing Analysis - {self.lap_time:.3f}s")
        
        # 顶部统计
        header_text = (
            f"⏱️ Total Time: {self.lap_time:.3f} s   |   "
            f"📏 Centerline: {self.centerline_len:.1f} m   |   "
            f"🏎️ Racing Line: {self.racing_line_len:.1f} m"
        )
        self.fig.suptitle(header_text, fontsize=16, fontweight='bold', y=0.96, color='#333333')
        
        gs = GridSpec(4, 3, figure=self.fig, width_ratios=[1.8, 1, 1], top=0.90, bottom=0.1, hspace=0.4)
        
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_speed = self.fig.add_subplot(gs[0, 1:])
        self.ax_elev = self.fig.add_subplot(gs[1, 1:], sharex=self.ax_speed)
        self.ax_gg = self.fig.add_subplot(gs[2:4, 1])
        self.ax_info = self.fig.add_subplot(gs[2:4, 2])
        self.ax_info.axis('off')

        self._plot_map()
        self._plot_speed_telemetry()
        self._plot_elevation()
        self._plot_gg_static()
        self._init_cursors()
        
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        
        # 返回按钮
        self.btn_ax = self.fig.add_axes([0.85, 0.02, 0.12, 0.05])
        self.btn_return = Button(self.btn_ax, 'Back to Menu', color='#e0e0e0', hovercolor='#c0c0c0')
        self.btn_return.label.set_color('black')
        self.btn_return.on_clicked(self._on_return_clicked)
        
        plt.show()

    def _on_return_clicked(self, event):
        print("[System] Closing dashboard, returning to menu...")
        plt.close(self.fig)

    def _plot_map(self):
        ax = self.ax_map
        ax.plot(self.track.inner[:,0], self.track.inner[:,1], 'k-', lw=1.0, alpha=0.6)
        ax.plot(self.track.outer[:,0], self.track.outer[:,1], 'k-', lw=1.0, alpha=0.6)
        
        pts = np.array([self.path_x, self.path_y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        norm = Normalize(vmin=0, vmax=self.stats['v_max'])
        lc = LineCollection(segs, cmap='turbo', norm=norm, lw=3.0)
        lc.set_array(self.speed_kmh)
        ax.add_collection(lc)
        
        ax.set_aspect('equal')
        
        ax.invert_yaxis()
        
        ax.set_title("Track Map", fontsize=12, fontweight='bold')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True, linestyle=':', alpha=0.5)

    def _plot_speed_telemetry(self):
        ax = self.ax_speed
        
        # 1. 速度曲线
        line, = ax.plot(self.s, self.speed_kmh, color='#1f77b4', lw=2.0, label="Speed", zorder=5)
        ax.set_ylabel("Speed (km/h)", fontsize=9, color='#1f77b4')
        ax.tick_params(axis='y', labelcolor='#1f77b4')
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title("Telemetry (Speed / Throttle / Brake)", fontsize=10, fontweight='bold')

        # 2. 油门刹车
        self.ax_input = ax.twinx()
        self.ax_input.set_ylim(0, 300) 
        self.ax_input.set_yticks([]) 
        
        self.ax_input.fill_between(self.s, self.throttle_pct, 0, 
                                   color='#2ca02c', alpha=0.3, step='post', label="Throttle")
        self.ax_input.fill_between(self.s, self.brake_pct, 0, 
                                   color='#d62728', alpha=0.4, step='post', label="Brake")

    def _plot_elevation(self):
        ax = self.ax_elev
        ax.fill_between(self.s, self.path_z, self.stats['z_min']-5, color='#d3d3d3', alpha=0.5)
        ax.plot(self.s, self.path_z, color='#555555', lw=2.0)
        ax.set_ylabel("Elevation (m)", fontsize=9)
        ax.set_xlabel("Distance (m)", fontsize=9)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title("Elevation Profile", fontsize=10, fontweight='bold')

    def _plot_gg_static(self):
        ax = self.ax_gg
        for r in [1.0, 2.0, 3.0]:
            circle = plt.Circle((0, 0), r, color='#999999', fill=False, ls='--')
            ax.add_artist(circle)
        
        ax.scatter(self.g_lat, self.g_long, c=self.speed_kmh, cmap='turbo', s=3, alpha=0.6)
        
        lim = self.stats['g_lim']
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.set_xlabel("Lat G", fontsize=9)
        ax.set_ylabel("Long G", fontsize=9)
        ax.set_title("G-G Diagram", fontsize=10, fontweight='bold')
        ax.axhline(0, c='black', lw=0.8, alpha=0.3)
        ax.axvline(0, c='black', lw=0.8, alpha=0.3)
        ax.grid(True, linestyle=':', alpha=0.3)

    def _init_cursors(self):
        cursor_style = dict(c='#ff7f0e', ms=8, mec='black', zorder=20)
        line_style = dict(c='black', ls='--', alpha=0.6)
        
        self.markers['map'], = self.ax_map.plot([], [], 'o', **cursor_style)
        
        self.markers['speed'], = self.ax_speed.plot([], [], 'o', **cursor_style)
        self.vlines['speed'] = self.ax_speed.axvline(0, **line_style)
        
        self.markers['elev'], = self.ax_elev.plot([], [], 'o', **cursor_style)
        self.vlines['elev'] = self.ax_elev.axvline(0, **line_style)
        
        self.markers['gg'], = self.ax_gg.plot([], [], 'o', **cursor_style)
        
        self.info_text = self.ax_info.text(0.05, 0.95, "", transform=self.ax_info.transAxes,
                                           fontsize=11, va='top', family='monospace', color='black')

    def _on_mouse_move(self, event):
        if not event.inaxes: return
        
        target_ax = event.inaxes
        if target_ax == self.ax_input: target_ax = self.ax_speed

        idx = 0
        if target_ax == self.ax_map:
            dist = (self.path_x - event.xdata)**2 + (self.path_y - event.ydata)**2
            idx = np.argmin(dist)
        elif target_ax in [self.ax_speed, self.ax_elev]:
            idx = np.searchsorted(self.s, event.xdata)
            if idx >= self.N: idx = self.N - 1
        elif target_ax == self.ax_gg:
            dist = (self.g_lat - event.xdata)**2 + (self.g_long - event.ydata)**2
            idx = np.argmin(dist)
        else:
            return
        self._update_dashboard(idx)

    def _update_dashboard(self, idx):
        if idx < 0 or idx >= self.N: return
        s = self.s[idx]
        x, y = self.path_x[idx], self.path_y[idx]
        z = self.path_z[idx]
        v = self.speed_kmh[idx]
        glat, glong = self.g_lat[idx], self.g_long[idx]
        slope = self.slope[idx]
        bank = self.bank[idx]
        
        thr = self.throttle_pct[idx]
        brk = self.brake_pct[idx]
        
        self.markers['map'].set_data([x], [y])
        self.markers['speed'].set_data([s], [v])
        self.vlines['speed'].set_xdata([s, s])
        self.markers['elev'].set_data([s], [z])
        self.vlines['elev'].set_xdata([s, s])
        self.markers['gg'].set_data([glat], [glong])
        
        txt = (
            f"LOCATION\n"
            f"────────\n"
            f"Dist : {s:.1f} m\n"
            f"Speed: {v:.1f} km/h\n"
            f"Elev : {z:.1f} m\n"
            f"\n"
            f"INPUTS\n"
            f"──────\n"
            f"Thr  : {thr:.0f} %\n"
            f"Brake: {brk:.0f} %\n"
            f"G-Lat: {glat:+.2f} G\n"
            f"G-Lon: {glong:+.2f} G\n"
            f"\n"
            f"GEOMETRY\n"
            f"────────\n"
            f"Slope: {slope:+.1f} °\n"
            f"Bank : {bank:+.1f} °"
        )
        self.info_text.set_text(txt)
        self.fig.canvas.draw_idle()

def quick_visualize(track, car, result):
    db = Dashboard(track, car, result)
    db.show()