import numpy as np
from scipy.interpolate import splprep, splev, CubicSpline, interp1d
from scipy.spatial import cKDTree
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection 

class Track:

    def __init__(self, inner_points: np.ndarray, outer_points: np.ndarray):
        self.inner = np.array(inner_points, dtype=float)
        self.outer = np.array(outer_points, dtype=float)
        
        if self.inner.shape[1] < 3:
            self.inner = np.column_stack([self.inner[:,:2], np.zeros(len(self.inner))])
        if self.outer.shape[1] < 3:
            self.outer = np.column_stack([self.outer[:,:2], np.zeros(len(self.outer))])

        print(f"[Track v12.2] 初始化 3D 赛道...")
        
        self._preprocess_boundaries()
        self.N = len(self.inner_dense)

        # 初始生成几何中心线
        self._generate_centerline()
        
        # 射线求交计算宽度
        self._compute_ray_intersection_widths()
        
        # 构建查找表
        self._build_lookup_tables()

        print(f"[Track v12.2] ✓ 初始化完成. 长度={self.total_length:.2f}m, 高差={np.ptp(self.z_dense):.1f}m")

    def _preprocess_boundaries(self):
        def clean_and_resample(pts):
            diff = np.linalg.norm(np.diff(pts[:, :2], axis=0, prepend=pts[0:1,:2]-1.0), axis=1)
            pts = pts[diff > 1e-4]
            
            dist = np.linalg.norm(pts[0, :2] - pts[-1, :2])
            total = np.sum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))
            is_closed = dist < total * 0.01
            
            if is_closed and dist > 0.1:
                pts = np.vstack([pts, pts[0]])
            
            d_3d = np.cumsum(np.linalg.norm(np.diff(pts, axis=0, prepend=pts[0:1]), axis=1))
            _, idx = np.unique(d_3d, return_index=True); idx.sort()
            d_3d = d_3d[idx]; pts = pts[idx]
            
            d_new = np.linspace(0, d_3d[-1], int(max(d_3d[-1]/0.2, 10)))
            
            new_cols = []
            for i in range(3):
                f = interp1d(d_3d, pts[:, i], kind='linear')
                new_cols.append(f(d_new))
            
            return np.column_stack(new_cols), is_closed

        self.inner_dense, self.is_closed = clean_and_resample(self.inner)
        self.outer_dense, _ = clean_and_resample(self.outer)
        
        self.per_flag = 1 if self.is_closed else 0
        
        self.tree_inner = cKDTree(self.inner_dense[:, :2])
        self.tree_outer = cKDTree(self.outer_dense[:, :2])

    def _generate_centerline(self):
        d, idx = self.tree_outer.query(self.inner_dense[:, :2])
        mid_pts = (self.inner_dense + self.outer_dense[idx]) / 2.0
        
        window = 51
        if len(mid_pts) > window:
            mode = 'wrap' if self.is_closed else 'interp'
            for i in range(3):
                mid_pts[:, i] = savgol_filter(mid_pts[:, i], window, 3, mode=mode)

        if self.is_closed: mid_pts[-1] = mid_pts[0]
        self._fit_centerline_spline(mid_pts)

    def _fit_centerline_spline(self, pts_3d):
        """核心样条拟合提取，方便 MCL 复用"""
        try:
            self.tck, u = splprep(pts_3d.T, u=None, s=len(pts_3d)*0.2, k=3, per=self.per_flag)
        except:
            self.tck, u = splprep(pts_3d.T, u=None, s=len(pts_3d)*1.0, k=3, per=self.per_flag)

        u_map = np.linspace(0, 1, 5000)
        out = splev(u_map, self.tck)
        diffs = np.sqrt(np.diff(out[0])**2 + np.diff(out[1])**2 + np.diff(out[2])**2)
        self.s_map = np.concatenate([[0], np.cumsum(diffs)])
        self.total_length = self.s_map[-1]
        
        _, idx = np.unique(self.s_map, return_index=True); idx.sort()
        self.s_map = self.s_map[idx]; self.u_map = u_map[idx]
        self.s_to_u = interp1d(self.s_map, self.u_map, kind='linear', fill_value="extrapolate")

    def update_reference_line(self, x_new: np.ndarray, y_new: np.ndarray):
        """[核心更新] 接收优化后的最小曲率参考线 (MCL)，彻底重建坐标系以消除 Frenet 奇异性"""
        print(f"  [Track] 正在重建赛道坐标系至最小曲率参考线...")
        
        # 1. 估算新坐标的 Z 值
        old_pts = np.column_stack([self.x_dense, self.y_dense])
        tree_old = cKDTree(old_pts)
        _, idx = tree_old.query(np.column_stack([x_new, y_new]))
        z_new = self.z_dense[idx]
        
        mid_pts = np.column_stack([x_new, y_new, z_new])
        if self.is_closed:
            mid_pts[-1] = mid_pts[0]
            
        # 2. 重新拟合基准线
        self._fit_centerline_spline(mid_pts)
        
        # 3. 基于新基准线重新计算赛道宽度和物理参数
        self._compute_ray_intersection_widths()
        self._build_lookup_tables()
        print(f"  [Track] ✓ 坐标系重建完成. 新基准长度={self.total_length:.2f}m")

    def _compute_ray_intersection_widths(self):
        if self.is_closed:
            s_vals = np.linspace(0, self.total_length, int(max(self.total_length/0.2, 10)), endpoint=False)
        else:
            s_vals = np.linspace(0, self.total_length, int(max(self.total_length/0.2, 10)), endpoint=True)
        
        u_vals = self.s_to_u(s_vals)
        coords = splev(u_vals, self.tck)
        cx, cy = coords[0], coords[1]
        
        dx, dy = splev(u_vals, self.tck, der=1)[0:2]
        norms = np.hypot(dx, dy)
        nx, ny = -dy/norms, dx/norms 
        
        wl_list, wr_list = [], []
        R_SEARCH = 30.0 
        
        I_pts = self.inner_dense[:, :2]
        O_pts = self.outer_dense[:, :2]
        
        for i in range(len(cx)):
            P = np.array([cx[i], cy[i]])
            N = np.array([nx[i], ny[i]])
            
            # Left
            idx_candidates = self.tree_inner.query_ball_point(P, R_SEARCH)
            min_dist = 5.0
            found = False
            if len(idx_candidates) > 1:
                cand = np.array(idx_candidates)
                cand = cand[cand < len(I_pts) - 1]
                if len(cand) > 0:
                    A = I_pts[cand]; B = I_pts[cand + 1]
                    dists = self._batch_ray_segment_intersect(P, N, A, B)
                    valid = dists[dists > 0.1]
                    if len(valid) > 0:
                        min_dist = np.min(valid); found = True
            if not found: min_dist, _ = self.tree_inner.query(P)
            wl_list.append(min_dist)

            # Right
            N_right = -N
            idx_candidates = self.tree_outer.query_ball_point(P, R_SEARCH)
            min_dist_r = 5.0
            found_r = False
            if len(idx_candidates) > 1:
                cand = np.array(idx_candidates)
                cand = cand[cand < len(O_pts) - 1]
                if len(cand) > 0:
                    A = O_pts[cand]; B = O_pts[cand + 1]
                    dists = self._batch_ray_segment_intersect(P, N_right, A, B)
                    valid = dists[dists > 0.1]
                    if len(valid) > 0:
                        min_dist_r = np.min(valid); found_r = True
            if not found_r: min_dist_r, _ = self.tree_outer.query(P)
            wr_list.append(min_dist_r)

        wl_raw = np.array(wl_list); wr_raw = np.array(wr_list)
        mode = 'wrap' if self.is_closed else 'interp'
        self.wl_smooth = savgol_filter(wl_raw, 11, 2, mode=mode) if len(wl_raw)>11 else wl_raw
        self.wr_smooth = savgol_filter(wr_raw, 11, 2, mode=mode) if len(wr_raw)>11 else wr_raw
        self.wl_smooth = np.maximum(self.wl_smooth, 0.1)
        self.wr_smooth = np.maximum(self.wr_smooth, 0.1)

        if self.is_closed:
            s_final = np.append(s_vals, self.total_length)
            wl_final = np.append(self.wl_smooth, self.wl_smooth[0])
            wr_final = np.append(self.wr_smooth, self.wr_smooth[0])
            self.spline_wl = CubicSpline(s_final, wl_final, bc_type='periodic')
            self.spline_wr = CubicSpline(s_final, wr_final, bc_type='periodic')
        else:
            self.spline_wl = CubicSpline(s_vals, self.wl_smooth)
            self.spline_wr = CubicSpline(s_vals, self.wr_smooth)

    def _batch_ray_segment_intersect(self, O, D, A, B):
        v1 = O - A; v2 = B - A; v3 = np.array([-D[1], D[0]])
        dot = v2[:,0] * v3[0] + v2[:,1] * v3[1]
        denom = D[0] * (B[:,1] - A[:,1]) - D[1] * (B[:,0] - A[:,0])
        mask_valid = np.abs(denom) > 1e-6
        t = np.full(len(A), -1.0)
        
        if np.any(mask_valid):
            numer_t = (A[:,0] - O[0]) * (B[:,1] - A[:,1]) - (A[:,1] - O[1]) * (B[:,0] - A[:,0])
            t_vals = numer_t[mask_valid] / denom[mask_valid]
            numer_u = (A[:,0] - O[0]) * D[1] - (A[:,1] - O[1]) * D[0]
            u_vals = numer_u[mask_valid] / denom[mask_valid]
            hits = (t_vals > 0) & (u_vals >= 0.0) & (u_vals <= 1.0)
            indices = np.where(mask_valid)[0]
            t[indices[hits]] = t_vals[hits]
        return t

    def _build_lookup_tables(self):
        self.s_dense = np.linspace(0, self.total_length, int(max(self.total_length/0.1, 10)))
        u_dense = self.s_to_u(self.s_dense)
        
        coords = splev(u_dense, self.tck)
        self.x_dense, self.y_dense, self.z_dense = coords[0], coords[1], coords[2]
        
        dx, dy = splev(u_dense, self.tck, der=1)[0:2]
        ddx, ddy = splev(u_dense, self.tck, der=2)[0:2]
        
        denom = (dx**2 + dy**2)**1.5
        numer = dx*ddy - dy*ddx
        self.kappa_dense = numer / np.maximum(denom, 1e-8)
        self.psi_dense = np.unwrap(np.arctan2(dy, dx))
        
        dz = splev(u_dense, self.tck, der=1)[2]
        ds_du = np.sqrt(dx**2 + dy**2 + dz**2)
        dz_ds = dz / ds_du
        self.slope_dense = np.arctan(dz_ds)
        
        wl = self.spline_wl(self.s_dense)
        wr = self.spline_wr(self.s_dense)
        
        nx, ny = -np.sin(self.psi_dense), np.cos(self.psi_dense)
        lx = self.x_dense + nx * wl
        ly = self.y_dense + ny * wl
        rx = self.x_dense - nx * wr
        ry = self.y_dense - ny * wr
        
        _, idx_l = self.tree_inner.query(np.column_stack([lx, ly]))
        _, idx_r = self.tree_outer.query(np.column_stack([rx, ry]))
        
        z_l = self.inner_dense[idx_l, 2]
        z_r = self.outer_dense[idx_r, 2]
        
        self.bank_dense = np.arctan2(z_l - z_r, wl + wr)
        
        window = 21
        if len(self.slope_dense) > window:
            mode = 'wrap' if self.is_closed else 'interp'
            self.slope_dense = savgol_filter(self.slope_dense, window, 2, mode=mode)
            self.bank_dense = savgol_filter(self.bank_dense, window, 2, mode=mode)

        self.wl_dense = wl; self.wr_dense = wr
        limit = 0.95
        mask_l = (self.kappa_dense > 0) & (self.kappa_dense * self.wl_dense > limit)
        if np.any(mask_l): self.wl_dense[mask_l] = limit / self.kappa_dense[mask_l]
        mask_r = (self.kappa_dense < 0) & (np.abs(self.kappa_dense) * self.wr_dense > limit)
        if np.any(mask_r): self.wr_dense[mask_r] = limit / np.abs(self.kappa_dense[mask_r])

    def diagnose(self):
        print(f"  赛道长度: {self.total_length:.2f}m")
        print(f"  高程范围: {np.min(self.z_dense):.1f}m ~ {np.max(self.z_dense):.1f}m")
        print(f"  最大曲率: {np.max(np.abs(self.kappa_dense)):.3f}")
        print(f"  最大坡度: {np.max(np.abs(np.degrees(self.slope_dense))):.1f} deg")
        print(f"  最大倾角: {np.max(np.abs(np.degrees(self.bank_dense))):.1f} deg")

    def plot_3d_preview(self):
        try:
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            fig.canvas.manager.set_window_title(f"3D Track Preview: {self.total_length:.1f}m")
            
            step = max(1, len(self.inner_dense) // 1000) 
            xi, yi, zi = self.inner_dense[::step].T
            xo, yo, zo = self.outer_dense[::step].T
            ax.plot(xi, yi, zi, 'r-', lw=1.5, label='Inner Wall')
            ax.plot(xo, yo, zo, 'b-', lw=1.5, label='Outer Wall')
            
            step_c = max(1, len(self.x_dense) // 500)
            ax.plot(self.x_dense[::step_c], self.y_dense[::step_c], self.z_dense[::step_c], 
                    'g-', lw=2, alpha=0.8, label='Reference Line')

            segments = []
            s_vals = self.s_dense[::step_c]
            for s in s_vals:
                xl, yl, zl = self.get_solver_boundary_point(s, 'left')
                xr, yr, zr = self.get_solver_boundary_point(s, 'right')
                segments.append([(xl, yl, zl), (xr, yr, zr)])
            
            lc = Line3DCollection(segments, colors='gray', alpha=0.3, linewidths=0.5)
            ax.add_collection3d(lc)

            max_range = np.array([
                self.x_dense.max()-self.x_dense.min(), 
                self.y_dense.max()-self.y_dense.min(), 
                self.z_dense.max()-self.z_dense.min()
            ]).max() / 2.0
            mid_x = (self.x_dense.max()+self.x_dense.min()) * 0.5
            mid_y = (self.y_dense.max()+self.y_dense.min()) * 0.5
            mid_z = (self.z_dense.max()+self.z_dense.min()) * 0.5
            
            z_scale = 3.0 
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range/z_scale, mid_z + max_range/z_scale)
            
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Elevation')
            ax.set_title("Track Geometry (Z-axis Exaggerated 3x)")
            ax.legend()
            plt.show()
            
        except Exception as e:
            print(f"[Error] 3D Preview Failed: {e}")

    def get_track_length(self): return self.total_length
    def get_curvature(self, s): return np.interp(s, self.s_dense, self.kappa_dense, period=self.total_length)
    def get_track_width(self, s): 
        return (np.interp(s, self.s_dense, self.wl_dense, period=self.total_length),
                np.interp(s, self.s_dense, self.wr_dense, period=self.total_length))
    def get_heading(self, s): return np.interp(s, self.s_dense, self.psi_dense, period=self.total_length)
    def get_position(self, s, n=0):
        x0 = np.interp(s, self.s_dense, self.x_dense, period=self.total_length)
        y0 = np.interp(s, self.s_dense, self.y_dense, period=self.total_length)
        z0 = np.interp(s, self.s_dense, self.z_dense, period=self.total_length)
        psi = np.interp(s, self.s_dense, self.psi_dense, period=self.total_length)
        return x0 - n*np.sin(psi), y0 + n*np.cos(psi), z0

    def get_slope(self, s): return np.interp(s, self.s_dense, self.slope_dense, period=self.total_length)
    def get_banking(self, s): return np.interp(s, self.s_dense, self.bank_dense, period=self.total_length)
    def get_solver_boundary_point(self, s, side='left'):
        wl, wr = self.get_track_width(s)
        return self.get_position(s, wl if side == 'left' else -wr)