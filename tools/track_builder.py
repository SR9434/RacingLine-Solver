import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox, RadioButtons, Slider
from scipy.interpolate import splprep, splev, PchipInterpolator
from scipy.spatial import cKDTree
from enum import Enum, auto

# 添加项目根目录
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# === 配置参数 ===
CONFIG = {
    'snap_threshold_px': 25.0,   
    'mesh_min_points': 200,      
    'point_duplicate_threshold': 0.01,
    'spline_smoothing_factor': 10,
    'pick_tolerance_m': 30.0,    # 3D 节点选中的距离阈值(米)
}

class BuilderState(Enum):
    INIT = auto()
    CALIBRATING = auto()
    DRAWING = auto()
    EDIT_3D = auto()

class TrackBuilderV19:
    def __init__(self, image_path=None):
        # 1. 布局初始化
        self.fig = plt.figure(figsize=(16, 10))
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1], bottom=0.1, right=0.80, left=0.05, top=0.95, hspace=0.3)
        
        self.ax = self.fig.add_subplot(gs[0])       
        self.ax_elev = self.fig.add_subplot(gs[1])  
        self.ax_bank = self.ax_elev.twinx()         
        
        # 2. 核心数据
        self.layers = {
            'INNER': {'segments': [], 'color': 'r', 'closed': False, 'z_nodes': []},
            'OUTER': {'segments': [], 'color': 'b', 'closed': False, 'z_nodes': []}
        }
        self.active_layer = 'INNER'
        self.draw_tool = 'CURVE'
        self.scale_factor = None 
        self.state = BuilderState.INIT
        
        # 交互变量
        self.temp_points = []
        self.calib_points = []
        self.calib_markers = []
        self.panning = False
        self.pan_start = None
        self.tangent_snap = True
        
        # 3D & Mesh 缓存
        self.current_mesh_length = 1000.0
        self.final_mesh_data = None
        self.mesh_s_map = None
        self.mesh_xy_in = None
        self.mesh_xy_out = None
        self.kd_tree_in = None
        self.kd_tree_out = None
        
        # 3D 编辑状态
        self.hover_s = None           
        self.selected_node_idx = None 

        # 3. 绘图元素
        self.line_preview, = self.ax.plot([], [], 'k--', lw=1.5, alpha=0.8, zorder=20)
        self.line_control, = self.ax.plot([], [], 'g:', lw=1.0, marker='o', markersize=4, zorder=20)
        self.line_tangent, = self.ax.plot([], [], 'c--', lw=1.2, alpha=0.8, zorder=19)
        self.marker_snap, = self.ax.plot([], [], 'rx', markersize=10, markeredgewidth=3, zorder=30)
        self.marker_close, = self.ax.plot([], [], 'go', markersize=15, fillstyle='none', markeredgewidth=3, zorder=30)
        self.marker_start, = self.ax.plot([], [], 'gs', markersize=6, zorder=15)
        self.mesh_lines = []
        self.preview_boundary_in, = self.ax.plot([], [], 'r-', lw=0.5, alpha=0.5, zorder=10) 
        self.preview_boundary_out, = self.ax.plot([], [], 'b-', lw=0.5, alpha=0.5, zorder=10)

        # 3D Interactive
        self.plot_elev_in, = self.ax_elev.plot([], [], 'r.-', lw=1.5, label='Inner Z')
        self.plot_elev_out, = self.ax_elev.plot([], [], 'b.-', lw=1.5, label='Outer Z')
        self.plot_bank, = self.ax_bank.plot([], [], 'k:', lw=1.0, alpha=0.5, label='Bank Angle')
        
        self.cursor_map_snap, = self.ax.plot([], [], 'mx', markersize=10, markeredgewidth=2, zorder=50) 
        self.cursor_sync_elev, = self.ax_elev.plot([], [], 'mx', markersize=10, markeredgewidth=2, zorder=50) 
        self.marker_z_nodes_map, = self.ax.plot([], [], 'ko', markersize=5, markerfacecolor='white', zorder=40) 
        
        self.highlight_map, = self.ax.plot([], [], 'yo', markersize=14, fillstyle='none', markeredgewidth=2.5, zorder=55)
        self.highlight_elev, = self.ax_elev.plot([], [], 'yo', markersize=14, fillstyle='none', markeredgewidth=2.5, zorder=55)

        self.ax.set_aspect('equal'); self.ax.grid(True, linestyle=':', alpha=0.3)
        self.ax_elev.set_title("Elevation Profile (PCHIP Smooth)"); self.ax_elev.set_xlabel("Distance (m)"); self.ax_elev.set_ylabel("Z (m)")
        self.ax_elev.grid(True, alpha=0.3)
        self.ax_bank.set_ylabel("Bank (deg)"); self.ax_bank.set_ylim(-20, 20)
        
        if image_path and os.path.exists(image_path):
            img = plt.imread(image_path)
            self.ax.imshow(img, alpha=0.6, zorder=0)
        else:
            self.ax.set_xlim(0, 1500); self.ax.set_ylim(0, 1000)

        self._setup_gui()
        self._bind_events()
        self.update_status("STEP 1: Set Scale by clicking 2 points", 'red')
        plt.show()

    def _setup_gui(self):
        ax_panel = plt.axes([0.82, 0.05, 0.16, 0.9]); ax_panel.set_axis_off()
        self.btn_scale = Button(plt.axes([0.82, 0.90, 0.14, 0.04]), '1. Set Scale', color='#ffcc99')
        self.btn_scale.on_clicked(self.cb_start_calibrate)
        self.txt_dist = TextBox(plt.axes([0.82, 0.85, 0.14, 0.03]), 'Dist (m): ', initial="500")

        plt.text(0.82, 0.82, "2D Draw Tools", transform=self.fig.transFigure, weight='bold')
        self.rad_layer = RadioButtons(plt.axes([0.82, 0.74, 0.14, 0.07]), ('INNER', 'OUTER'), active=0)
        self.rad_layer.on_clicked(self.cb_set_layer)
        self.rad_tool = RadioButtons(plt.axes([0.82, 0.66, 0.14, 0.07]), ('Curve', 'Line'), active=0)
        self.rad_tool.on_clicked(self.cb_set_tool)
        self.btn_undo = Button(plt.axes([0.82, 0.58, 0.14, 0.04]), 'Undo Point')
        self.btn_undo.on_clicked(self.cb_undo)

        plt.text(0.82, 0.53, "3D Edit Tools", transform=self.fig.transFigure, weight='bold', color='blue')
        self.btn_mode_3d = Button(plt.axes([0.82, 0.48, 0.14, 0.04]), 'Edit 3D: OFF', color='#e0e0e0')
        self.btn_mode_3d.on_clicked(self.cb_toggle_3d)
        
        plt.text(0.82, 0.43, "Edit Selected Z (m):", transform=self.fig.transFigure, fontsize=9, color='blue')
        self.txt_z_val = TextBox(plt.axes([0.82, 0.39, 0.14, 0.04]), '', initial="")
        self.txt_z_val.on_submit(self.cb_z_submit) 
        
        self.btn_clear_z = Button(plt.axes([0.82, 0.33, 0.14, 0.04]), 'Clear Layer Z')
        self.btn_clear_z.on_clicked(self.cb_clear_z)

        # 补回了上一版遗漏的 Slider 控件
        plt.text(0.82, 0.28, "Mesh Resolution", transform=self.fig.transFigure, weight='bold')
        self.slider_res = Slider(plt.axes([0.82, 0.25, 0.14, 0.03]), 'Res', 0.5, 10.0, valinit=2.0)
        self.slider_res.on_changed(self.cb_update_mesh)

        plt.text(0.82, 0.20, "Save Options", transform=self.fig.transFigure, weight='bold')
        self.txt_name = TextBox(plt.axes([0.82, 0.16, 0.14, 0.03]), 'Name: ', initial="track_3d")
        self.btn_save = Button(plt.axes([0.82, 0.08, 0.14, 0.05]), 'Save 3D CSV', color='#ccffcc')
        self.btn_save.on_clicked(self.cb_save)
        
        self.txt_status = plt.text(0.02, 0.01, "Ready", transform=self.fig.transFigure, weight='bold')

    def _bind_events(self):
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)

    # =======================================================
    # INTERACTION LOGIC
    # =======================================================
    def on_click(self, event):
        if event.button == 2 and event.inaxes == self.ax:
            self.panning = True; self.pan_start = (event.xdata, event.ydata); return

        if self.state == BuilderState.CALIBRATING:
            if event.button == 1 and event.inaxes == self.ax: self._handle_calibration_step(event)
            return

        if self.state == BuilderState.DRAWING:
            if event.inaxes == self.ax:
                if event.button == 1: self._handle_draw_click(event)
                elif event.button == 3: self.cb_undo(None)
            return

        if self.state == BuilderState.EDIT_3D:
            if event.button == 1:
                if event.inaxes == self.ax:
                    if not self._try_select_map_node(event.xdata, event.ydata):
                        if self.hover_s is not None: self._add_node(self.hover_s, 0.0)
                        else: self._deselect_node()
                elif event.inaxes == self.ax_elev:
                    if not self._try_select_graph_node(event.xdata, event.ydata):
                        self._deselect_node()
            elif event.button == 3 and event.inaxes == self.ax_elev:
                self._delete_nearest_node(event.xdata, event.ydata)

    def _deselect_node(self):
        self.selected_node_idx = None
        self.highlight_map.set_data([], [])
        self.highlight_elev.set_data([], [])
        self.txt_z_val.set_val("")
        self.update_status("Selection cleared", 'black')
        self.fig.canvas.draw_idle()

    def _select_node(self, idx):
        self.selected_node_idx = idx
        nodes = self.layers[self.active_layer]['z_nodes']
        s, z = nodes[idx]
        self.txt_z_val.set_val(f"{z:.2f}")
        
        # UI Highlights
        self.highlight_elev.set_data([s], [z])
        idx_mesh = min(np.searchsorted(self.mesh_s_map, s), len(self.mesh_s_map)-1)
        xy = self.mesh_xy_in[idx_mesh] if self.active_layer=='INNER' else self.mesh_xy_out[idx_mesh]
        self.highlight_map.set_data([xy[0]], [xy[1]])
        
        self.update_status(f"Selected Node: s={s:.1f}m, z={z:.2f}m. Type new Z and press Enter.", 'blue')
        self.fig.canvas.draw_idle()

    def cb_z_submit(self, text):
        if self.selected_node_idx is not None and self.state == BuilderState.EDIT_3D:
            try:
                new_z = float(text)
                nodes = self.layers[self.active_layer]['z_nodes']
                s, _ = nodes[self.selected_node_idx]
                nodes[self.selected_node_idx] = (s, new_z)
                self._update_mesh()
                self._select_node(self.selected_node_idx) 
            except ValueError: self.update_status("Error: Input a number", 'red')

    # =======================================================
    # 3D CORE (PCHIP + PERIODIC PADDING)
    # =======================================================
    def _calc_z(self, layer_name, s_array):
        """核心改进：使用 PCHIP 插值防止样条震荡"""
        nodes = self.layers[layer_name]['z_nodes']
        if not nodes: return np.zeros_like(s_array)
        
        ns = np.array([x[0] for x in nodes])
        nz = np.array([x[1] for x in nodes])
        is_closed = self.layers[layer_name]['closed']
        L = self.current_mesh_length
        
        if is_closed:
            # 镜像填充保证平滑
            ns_padded = np.concatenate([ns - L, ns, ns + L])
            nz_padded = np.concatenate([nz, nz, nz])
        else:
            ns_padded, nz_padded = ns, nz
            if ns[0] > 0.01:
                ns_padded = np.insert(ns_padded, 0, 0); nz_padded = np.insert(nz_padded, 0, nz[0])
            if ns[-1] < L - 0.01:
                ns_padded = np.append(ns_padded, L); nz_padded = np.append(nz_padded, nz[-1])

        try:
            interp = PchipInterpolator(ns_padded, nz_padded)
            return interp(s_array)
        except:
            return np.interp(s_array, ns, nz)

    # =======================================================
    # REUSED HELPERS
    # =======================================================
    def cb_toggle_3d(self, event):
        if not self.final_mesh_data:
            self.update_status("Draw 2D track first!", 'red')
            return
        if self.state == BuilderState.EDIT_3D:
            self.state = BuilderState.DRAWING
            self.btn_mode_3d.label.set_text('Edit 3D: OFF')
            self.btn_mode_3d.color = '#e0e0e0'
            self._deselect_node()
            self.cursor_map_snap.set_data([], [])
            self.cursor_sync_elev.set_data([], [])  # 修复点1
        else:
            self.state = BuilderState.EDIT_3D
            self.btn_mode_3d.label.set_text('Edit 3D: ON')
            self.btn_mode_3d.color = '#ffff99'
            self._build_kdtree()
        self.fig.canvas.draw_idle()

    def _handle_3d_hover(self, event):
        if event.inaxes == self.ax and self.kd_tree_in:
            tree = self.kd_tree_in if self.active_layer == 'INNER' else self.kd_tree_out
            d, idx = tree.query([event.xdata, event.ydata])
            if d < CONFIG['snap_threshold_px']:
                self.hover_s = self.mesh_s_map[idx]
                pt = tree.data[idx]
                self.cursor_map_snap.set_data([pt[0]], [pt[1]])
                # 修复点2：画垂直线需要2个点，x和y长度必须一致
                ylim = self.ax_elev.get_ylim()
                self.cursor_sync_elev.set_data([self.hover_s, self.hover_s], [ylim[0], ylim[1]])
            else:
                self.cursor_map_snap.set_data([], [])
                self.cursor_sync_elev.set_data([], [])  # 修复点3：同时清空x和y
                self.hover_s = None
        elif event.inaxes == self.ax_elev:
            s_val = event.xdata
            if s_val is not None:  # 修复点4：添加空值检查
                ylim = self.ax_elev.get_ylim()
                # 修复点5：使用set_data确保x,y长度一致
                self.cursor_sync_elev.set_data([s_val, s_val], [ylim[0], ylim[1]])
                if self.mesh_s_map is not None:
                    idx = max(0, min(np.searchsorted(self.mesh_s_map, s_val), len(self.mesh_s_map) - 1))
                    xy = self.mesh_xy_in[idx] if self.active_layer == 'INNER' else self.mesh_xy_out[idx]
                    self.cursor_map_snap.set_data([xy[0]], [xy[1]])
        self.fig.canvas.draw_idle()

    def _try_select_map_node(self, x, y):
        nodes = self.layers[self.active_layer]['z_nodes']
        if not nodes: return False
        for i, (s, z) in enumerate(nodes):
            im = min(np.searchsorted(self.mesh_s_map, s), len(self.mesh_s_map)-1)
            xy = self.mesh_xy_in[im] if self.active_layer=='INNER' else self.mesh_xy_out[im]
            if np.hypot(x-xy[0], y-xy[1]) < CONFIG['pick_tolerance_m']:
                self._select_node(i); return True
        return False

    def _try_select_graph_node(self, s, z):
        nodes = self.layers[self.active_layer]['z_nodes']
        if not nodes: return False
        dists = [np.hypot(n[0]-s, (n[1]-z)*20) for n in nodes]
        idx = np.argmin(dists)
        if dists[idx] < 80: self._select_node(idx); return True
        return False

    def _add_node(self, s, z):
        nodes = self.layers[self.active_layer]['z_nodes']
        nodes.append((s, z)); nodes.sort(key=lambda x: x[0])
        self._update_mesh()
        self._select_node(nodes.index((s, z)))

    def _delete_nearest_node(self, s, z):
        nodes = self.layers[self.active_layer]['z_nodes']
        if not nodes: return
        idx = np.argmin([abs(n[0]-s) for n in nodes])
        if abs(nodes[idx][0] - s) < 50:
            nodes.pop(idx); self._deselect_node(); self._update_mesh()

    def cb_start_calibrate(self, event):
        self.state = BuilderState.CALIBRATING; self.calib_points = []
        [m.remove() for m in self.calib_markers]; self.calib_markers = []
        self.update_status("Click Point 1...", 'orange')

    def _handle_calibration_step(self, event):
        self.calib_points.append([event.xdata, event.ydata])
        m, = self.ax.plot(event.xdata, event.ydata, 'mx', markersize=10, markeredgewidth=2)
        self.calib_markers.append(m); self.fig.canvas.draw()
        if len(self.calib_points) == 2:
            try:
                dist = float(self.txt_dist.text)
                self.scale_factor = dist / np.linalg.norm(np.array(self.calib_points[1])-np.array(self.calib_points[0]))
                [m.remove() for m in self.calib_markers]; self.calib_markers=[]
                self.state = BuilderState.DRAWING; self.update_status(f"Scale: {self.scale_factor:.3f} m/px. Start Drawing!", 'green')
            except: self.update_status("Error: Invalid Distance", 'red'); self.calib_points=[]

    def _handle_draw_click(self, event):
        if self.layers[self.active_layer]['closed']: return
        pos = self._calc_tangent_pos(event.xdata, event.ydata)
        snap, stype = self._check_snap(pos[0], pos[1])
        final = snap if snap is not None else pos
        if stype == 'CLOSE_LOOP' and not self.temp_points: self._close_loop(); return
        if not self.temp_points:
            segs = self.layers[self.active_layer]['segments']
            if segs:
                start = segs[-1]['points'][-1]; self.temp_points.append(start)
                if np.linalg.norm(np.array(final)-np.array(start))>0.1: self.temp_points.append(final)
            else: self.temp_points.append(final)
        else: self.temp_points.append(final)
        req = 2 if self.draw_tool=='LINE' else 3
        if len(self.temp_points)==req:
            self.layers[self.active_layer]['segments'].append({'type':self.draw_tool.lower(), 'points':list(self.temp_points)})
            self.temp_points=[]; self._update_view_2d(); self._update_mesh()

    def _update_draw_preview(self, event):
        if self.layers[self.active_layer]['closed']: return
        pos = self._calc_tangent_pos(event.xdata, event.ydata)
        snap, stype = self._check_snap(pos[0], pos[1])
        self.marker_close.set_alpha(1 if stype=='CLOSE_LOOP' else 0); self.marker_snap.set_alpha(1 if stype=='PREV_END' else 0)
        if snap: 
            if stype=='CLOSE_LOOP': self.marker_close.set_data([snap[0]], [snap[1]])
            else: self.marker_snap.set_data([snap[0]], [snap[1]])
        target = snap if snap is not None else pos
        pts = self.temp_points + [target]
        if not pts: return
        if self.draw_tool=='LINE': self.line_preview.set_data([p[0] for p in pts],[p[1] for p in pts]); self.line_control.set_data([],[])
        elif len(pts)==2: self.line_control.set_data([p[0] for p in pts],[p[1] for p in pts]); self.line_preview.set_data([],[])
        elif len(pts)==3:
            p=np.array(pts); t=np.linspace(0,1,30).reshape(-1,1)
            c=(1-t)**2*p[0]+2*(1-t)*t*p[1]+t**2*p[2]
            self.line_preview.set_data(c[:,0],c[:,1]); self.line_control.set_data(p[:,0],p[:,1])
        self.fig.canvas.draw_idle()

    def _calc_tangent_pos(self, x, y):
        if self.tangent_snap and len(self.temp_points)==1 and self.layers[self.active_layer]['segments']:
            prev = self.layers[self.active_layer]['segments'][-1]; p_end = np.array(prev['points'][-1])
            tan = p_end-(prev['points'][0] if prev['type']=='line' else prev['points'][1] if prev['type']=='curve' else prev['points'][-2])
            norm=np.linalg.norm(tan)
            if norm>1e-6:
                u=tan/norm; pos=p_end+u*np.dot(np.array([x,y])-p_end,u)
                self.line_tangent.set_data([p_end[0]-u[0]*3000, p_end[0]+u[0]*3000], [p_end[1]-u[1]*3000, p_end[1]+u[1]*3000])
                return pos.tolist()
        self.line_tangent.set_data([],[]); return [x,y]

    def _check_snap(self, x, y):
        segs = self.layers[self.active_layer]['segments']
        if not segs: return None, None
        thr = (self.ax.get_xlim()[1]-self.ax.get_xlim()[0])*0.03
        if not self.temp_points:
            if np.hypot(x-segs[0]['points'][0][0], y-segs[0]['points'][0][1]) < thr: return segs[0]['points'][0], 'CLOSE_LOOP'
            if np.hypot(x-segs[-1]['points'][-1][0], y-segs[-1]['points'][-1][1]) < thr: return segs[-1]['points'][-1], 'PREV_END'
        return None, None

    def _close_loop(self):
        segs=self.layers[self.active_layer]['segments']; p0=np.array(segs[-1]['points'][-1]); p3=np.array(segs[0]['points'][0])
        v0 = p0 - (segs[-1]['points'][-2] if len(segs[-1]['points'])>2 else segs[-1]['points'][0])
        v3 = (segs[0]['points'][1] if len(segs[0]['points'])>1 else segs[0]['points'][-1]) - p3
        v0/=np.linalg.norm(v0); v3/=np.linalg.norm(v3); d=np.linalg.norm(p3-p0)
        seg={'type':'cubic', 'points':[p0.tolist(), (p0+v0*d*0.4).tolist(), (p3-v3*d*0.4).tolist(), p3.tolist()]}
        self.layers[self.active_layer]['segments'].append(seg); self.layers[self.active_layer]['closed']=True
        self.temp_points=[]; self._update_view_2d(); self._update_mesh()

    def _update_view_2d(self):
        self.line_preview.set_data([],[]); self.line_control.set_data([],[]); self.line_tangent.set_data([],[]); self.marker_close.set_alpha(0); self.marker_snap.set_alpha(0)
        [l.remove() for l in self.ax.lines if l.get_label()=='segment']
        for L in self.layers.values():
            for s in L['segments']: self.ax.plot(self._eval_segment(s)[:,0], self._eval_segment(s)[:,1], color=L['color'], label='segment')
        segs = self.layers[self.active_layer]['segments']
        if segs and not self.layers[self.active_layer]['closed']: self.marker_start.set_data([segs[0]['points'][0][0]],[segs[0]['points'][0][1]])
        else: self.marker_start.set_data([],[])
        self.fig.canvas.draw_idle()

    # === MESH & UTILS ===
    def _update_mesh(self, val=None):
        for l in self.mesh_lines: l.remove()
        self.mesh_lines=[]
        if not (self.layers['INNER']['segments'] and self.layers['OUTER']['segments']): return
        try:
            pts_in=self._clean(self._get_pts('INNER')); pts_out=self._clean(self._get_pts('OUTER'))
            if len(pts_in)<4 or len(pts_out)<4: return
            tck_in,_=splprep(pts_in.T, s=self.slider_res.val*10, per=self.layers['INNER']['closed'], k=3)
            tck_out,_=splprep(pts_out.T, s=self.slider_res.val*10, per=self.layers['OUTER']['closed'], k=3)
            xy=np.array(splev(np.linspace(0,1,1000),tck_in)).T
            L = np.sum(np.linalg.norm(np.diff(xy,axis=0),axis=1))*self.scale_factor; self.current_mesh_length=L
            n=max(200, int(L/self.slider_res.val)); u=np.linspace(0,1,n)
            gi=np.array(splev(u,tck_in)).T; go=np.array(splev(u,tck_out)).T
            self.mesh_s_map=u*L; self.mesh_xy_in=gi; self.mesh_xy_out=go
            if self.state==BuilderState.EDIT_3D: self._build_kdtree()
            zi=self._calc_z('INNER', self.mesh_s_map); zo=self._calc_z('OUTER', self.mesh_s_map)
            bank=np.degrees(np.arctan((zo-zi)/np.maximum(np.linalg.norm(go-gi,axis=1)*self.scale_factor, 0.1)))
            step=max(1,n//50)
            for i in range(0,n,step): self.mesh_lines.append(self.ax.plot([gi[i,0],go[i,0]],[gi[i,1],go[i,1]],'y-',lw=0.5,alpha=0.4)[0])
            self.preview_boundary_in.set_data(gi[:,0],gi[:,1]); self.preview_boundary_out.set_data(go[:,0],go[:,1])
            self.plot_elev_in.set_data(self.mesh_s_map, zi); self.plot_elev_out.set_data(self.mesh_s_map, zo)
            self.plot_bank.set_data(self.mesh_s_map, bank)
            self.ax_elev.set_xlim(0,L); self.ax_elev.set_ylim(min(zi.min(),zo.min())-5, max(zi.max(),zo.max())+5)
            self.final_mesh_data=(gi,zi,go,zo)
            
            map_z_pts = []
            for name in ['INNER', 'OUTER']:
                if self.active_layer == name:
                    for n_pt in self.layers[name]['z_nodes']:
                        idx = min(np.searchsorted(self.mesh_s_map, n_pt[0]), len(self.mesh_s_map)-1)
                        map_z_pts.append(self.mesh_xy_in[idx] if name=='INNER' else self.mesh_xy_out[idx])
            if map_z_pts: p=np.array(map_z_pts); self.marker_z_nodes_map.set_data(p[:,0], p[:,1])
            else: self.marker_z_nodes_map.set_data([],[])

        except Exception as e: print(e)

    def _get_pts(self, name): return np.vstack([self._eval_segment(s) for s in self.layers[name]['segments']])
    def _clean(self, pts): return pts[np.concatenate([[True], np.linalg.norm(np.diff(pts,axis=0),axis=1)>1e-4])]
    def _eval_segment(self, s):
        p=np.array(s['points']); t=np.linspace(0,1,20).reshape(-1,1)
        if s['type']=='line': return (1-t)*p[0]+t*p[1]
        elif s['type']=='curve': return (1-t)**2*p[0]+2*(1-t)*t*p[1]+t**2*p[2]
        return (1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t**2*p[2]+t**3*p[3]
    def _build_kdtree(self):
        if self.mesh_xy_in is not None: self.kd_tree_in=cKDTree(self.mesh_xy_in); self.kd_tree_out=cKDTree(self.mesh_xy_out)
    def on_release(self, event): 
        self.panning = False
    def on_move(self, event):
        """处理鼠标移动事件"""
        # 平移处理
        if self.panning and event.inaxes == self.ax and self.pan_start is not None:
            if event.xdata is None or event.ydata is None:
                return
            dx = self.pan_start[0] - event.xdata
            dy = self.pan_start[1] - event.ydata
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
            self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)
            self.fig.canvas.draw_idle()
            return

        # 根据状态处理不同的悬停逻辑
        if self.state == BuilderState.DRAWING:
            if event.inaxes == self.ax and event.xdata is not None:
                self._update_draw_preview(event)
        elif self.state == BuilderState.EDIT_3D:
            self._handle_3d_hover(event)
    def on_scroll(self, event): pass # Handled in dispatch
    def cb_set_layer(self, l): self.active_layer=l; self.temp_points=[]; self._update_view_2d(); self._update_mesh(); self._deselect_node()
    def cb_set_tool(self, t): self.draw_tool='LINE' if 'Line' in t else 'CURVE'; self.temp_points=[]
    def cb_toggle_tangent(self, e): self.tangent_snap=not self.tangent_snap; self.btn_tangent.label.set_text(f"Tangent: {self.tangent_snap}")
    def cb_undo(self, e): 
        if self.temp_points: self.temp_points=[]
        elif self.layers[self.active_layer]['segments']: self.layers[self.active_layer]['segments'].pop(); self.layers[self.active_layer]['closed']=False; self._update_view_2d(); self._update_mesh()
    def cb_clear_z(self, e): self.layers[self.active_layer]['z_nodes']=[]; self._update_mesh(); self._deselect_node()
    def cb_update_mesh(self, e): self._update_mesh()
    def cb_save(self, e):
        if not self.final_mesh_data: return
        gi,zi,go,zo=self.final_mesh_data; origin=gi[0]
        pi=(gi-origin)*self.scale_factor; po=(go-origin)*self.scale_factor
        path=os.path.join('tracks', self.txt_name.text+'.csv'); os.makedirs('tracks', exist_ok=True)
        np.savetxt(path, np.column_stack([pi,zi,po,zo]), delimiter=',', header="xi,yi,zi,xo,yo,zo", comments='')
        self.update_status(f"Saved: {path}", 'blue')
    def update_status(self, t, c): self.txt_status.set_text(t); self.txt_status.set_color(c); self.fig.canvas.draw_idle()

if __name__ == "__main__":
    img = sys.argv[1] if len(sys.argv) > 1 else None
    builder = TrackBuilderV19(img)