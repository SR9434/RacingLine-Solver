import numpy as np
import casadi as ca
import time
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d, PchipInterpolator

class TrackOptimizer:
    
    PRECISION_CONFIGS = {
        'ultra_low': {'N': 100, 'description': '超低精度'},
        'low':       {'N': 200, 'description': '低精度'},
        'medium':    {'N': 400, 'description': '中精度'},
        'high':      {'N': 800, 'description': '高精度'},
        'ultra':     {'N': 1500, 'description': '超高精度'},
    }
    
    def __init__(self, track, car, config=None):
        self.track = track
        self.car = car
        self.mcl_optimized = False  # MCL 优化标志
        
        self.config = {
            'margin': 0.8,
            'v_min': 5.0,
            'v_max': 100.0,
            'friction_utilization': 0.95,
            'ipopt_tol': 1e-4,
            'ipopt_max_iter': 3000,
            'ipopt_print_level': 5,
            'use_multi_resolution': True,
        }
        if config: self.config.update(config)
        self._precompute_track_properties()

    def _precompute_track_properties(self):
        test_points = max(500, int(self.track.total_length / 2.0))
        s_test = np.linspace(0, self.track.total_length, test_points)
        k = self.track.get_curvature(s_test)
        self.track_max_curvature = np.max(np.abs(k))
        self.kappa_max = max(0.1, self.track_max_curvature * 1.5 + 0.1)

    def _optimize_mcl(self):
        """核心修复: 求解最小曲率参考线 (MCL)，彻底消除 Frenet 奇异性"""
        print("[Optimizer] 开始预计算最小曲率参考线 (MCL)...")
        N = min(800, max(200, int(self.track.total_length / 1.0)))
        s_vals = np.linspace(0, self.track.total_length, N + 1)
        
        x_ref, y_ref, psi_ref, w_l, w_r = (np.zeros(N+1) for _ in range(5))
        for i, s in enumerate(s_vals):
            x_ref[i], y_ref[i], _ = self.track.get_position(s)
            psi_ref[i] = self.track.get_heading(s)
            wl, wr = self.track.get_track_width(s)
            w_l[i], w_r[i] = wl, wr
            
        opti = ca.Opti()
        alpha = opti.variable(N + 1)
        
        # 安全裕度：保证新参考线不会紧贴赛道边缘
        m_l = np.maximum(w_l - 0.5, 0.1)
        m_r = np.maximum(w_r - 0.5, 0.1)
        opti.subject_to(alpha <= m_l)
        opti.subject_to(alpha >= -m_r)
        
        if self.track.is_closed:
            opti.subject_to(alpha[0] == alpha[N])
            
        # 笛卡尔坐标系的绝对坐标
        X = x_ref - alpha * ca.sin(psi_ref)
        Y = y_ref + alpha * ca.cos(psi_ref)
        
        obj = 0
        for i in range(1, N):
            dX = X[i+1] - 2*X[i] + X[i-1]
            dY = Y[i+1] - 2*Y[i] + Y[i-1]
            obj += dX**2 + dY**2
            
        if self.track.is_closed:
            dX = X[1] - 2*X[0] + X[N-1]
            dY = Y[1] - 2*Y[0] + Y[N-1]
            obj += dX**2 + dY**2
            
        opti.minimize(obj + 1e-4 * ca.sumsqr(alpha)) # 附加微小惩罚防止直道漂移
        
        opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes"}
        opti.solver("ipopt", opts)
        
        try:
            sol = opti.solve()
            alpha_opt = sol.value(alpha)
            
            # 平滑过滤后注入 Track 对象
            mode = 'wrap' if self.track.is_closed else 'nearest'
            alpha_smooth = gaussian_filter1d(alpha_opt, sigma=2, mode=mode)
            X_opt = x_ref - alpha_smooth * np.sin(psi_ref)
            Y_opt = y_ref + alpha_smooth * np.cos(psi_ref)
            
            # 回调底层赛道对象，更新所有坐标系！
            self.track.update_reference_line(X_opt, Y_opt)
            self._precompute_track_properties()
            print("[Optimizer] ✓ MCL 参考线预优化成功，奇异性已消除。")
        except Exception as e:
            print(f"[Optimizer] ⚠ MCL 求解失败，使用原始中心线继续: {e}")

    def solve(self, N_points=None, precision='medium'):
        if not self.mcl_optimized:
            self._optimize_mcl()
            self.mcl_optimized = True

        if N_points is None:
            N_points = self.PRECISION_CONFIGS.get(precision, {'N': 400})['N']
        
        if self.track.total_length < 10:
            print(f"[Optimizer] 错误: 赛道长度异常 ({self.track.total_length:.2f}m)")
            return None
        
        if self.config['use_multi_resolution'] and N_points > 60:
            return self._solve_multi_resolution(N_points)
        else:
            return self._solve_single(N_points, initial_guess=None)

    def _solve_multi_resolution(self, target_N):
        resolutions = []
        current_N = max(50, min(100, int(target_N * 0.6)))
        while current_N < target_N:
            resolutions.append(current_N)
            next_N = int(current_N * 1.8)
            current_N = target_N if next_N >= target_N * 0.9 else next_N
        
        if not resolutions or resolutions[-1] != target_N:
            resolutions.append(target_N)
        
        if len(resolutions) == 1 and resolutions[0] == target_N:
             return self._solve_single(target_N, initial_guess=None)

        result = None
        for i, N in enumerate(resolutions):
            print(f"  Stage {i+1}/{len(resolutions)}: N={N}")
            initial_guess = self._interpolate_solution(result, N) if result and result.get('success') else None
            result = self._solve_single(N, initial_guess)
            if not result or not result.get('success'):
                if i < len(resolutions) - 1:
                     print("  ⚠ Stage 求解失败，尝试重置猜测继续...")
                     result = None 
                else: break
        return result

    def _interpolate_solution(self, coarse_result, fine_N):
        s_coarse = coarse_result['s']
        s_fine = np.linspace(0, self.track.total_length, fine_N + 1)
    
        w_left_fine = np.zeros(fine_N + 1)
        w_right_fine = np.zeros(fine_N + 1)
        for i, s in enumerate(s_fine):
            wl, wr = self.track.get_track_width(s)
            w_left_fine[i] = wl; w_right_fine[i] = wr
    
        interp_n = PchipInterpolator(s_coarse, coarse_result['n'], extrapolate=True)
        interp_xi = PchipInterpolator(s_coarse, coarse_result['xi'], extrapolate=True)
        interp_v = PchipInterpolator(s_coarse, coarse_result['v'], extrapolate=True)
    
        s_control = (s_coarse[:-1] + s_coarse[1:]) / 2
        s_fine_control = (s_fine[:-1] + s_fine[1:]) / 2
        interp_a = interp1d(s_control, coarse_result['a'], kind='linear', fill_value='extrapolate')
        interp_kappa = interp1d(s_control, coarse_result['kappa'], kind='linear', fill_value='extrapolate')
    
        safe_margin = self.config['margin'] * 1.05
        n_interp = interp_n(s_fine)
        n_clipped = np.clip(n_interp, -w_right_fine + safe_margin, w_left_fine - safe_margin)
        v_smooth = gaussian_filter1d(interp_v(s_fine), sigma=max(2, fine_N//200))
        v_clipped = np.clip(v_smooth, self.config['v_min'], self.config['v_max'])
    
        return {
            's': s_fine, 'n': n_clipped, 'xi': np.clip(interp_xi(s_fine), -np.pi/3, np.pi/3),
            'v': v_clipped, 'a': interp_a(s_fine_control),
            'kappa': np.clip(interp_kappa(s_fine_control), -self.kappa_max, self.kappa_max)
        }

    def _solve_single(self, N_points, initial_guess=None):
        start_time = time.time()
        cfg = self.config
        
        s_vals = np.linspace(0, self.track.total_length, N_points + 1)
        ds = s_vals[1] - s_vals[0]
        
        k_ref, w_left, w_right, slope_ref, bank_ref = (np.zeros(N_points + 1) for _ in range(5))

        for i, s in enumerate(s_vals):
            k_ref[i] = self.track.get_curvature(s)
            w_left[i], w_right[i] = self.track.get_track_width(s)
            slope_ref[i] = self.track.get_slope(s)
            bank_ref[i] = self.track.get_banking(s)
            
        temp_margin = min(w_left.min(), w_right.min()) * 0.8 if min(w_left.min(), w_right.min()) < cfg['margin'] * 1.1 else cfg['margin']
        
        opti = ca.Opti()

        n = opti.variable(N_points + 1)
        xi = opti.variable(N_points + 1)
        v = opti.variable(N_points + 1)
        a_long_kin = opti.variable(N_points)
        kappa = opti.variable(N_points)
        
        p = self.car.params
        m, g = p['mass'], p['g']
        mu, rho = p['mu'], p['rho']
        Cd, Cl, A = p['Cd'], p['Cl'], p['A']
        P_max, F_drive_max = p['P_max'], p['force_max']

        T_total = 0
        for i in range(N_points):
            v_mid = (v[i] + v[i+1]) / 2
            xi_mid = (xi[i] + xi[i+1]) / 2
            n_mid = (n[i] + n[i+1]) / 2
            spatial_factor = ca.fmax(1 - n_mid * k_ref[i], 0.05)
            v_along = ca.fmax(v_mid * ca.cos(xi_mid), 1.0)
            T_total += (ds * spatial_factor) / v_along
        
        opti.minimize(T_total + 1e-4 * ca.sumsqr(kappa) + 1e-5 * ca.sumsqr(a_long_kin))

        opti.subject_to(v >= cfg['v_min'])
        opti.subject_to(v <= cfg['v_max'])
        opti.subject_to(xi >= -np.pi / 3)
        opti.subject_to(xi <= np.pi / 3)
        opti.subject_to(kappa >= -self.kappa_max)
        opti.subject_to(kappa <= self.kappa_max)
        
        for i in range(N_points):
            k_r, k_r_next = k_ref[i], k_ref[i + 1]
            v_curr, v_next = v[i], v[i+1]
            sf_curr, sf_next = 1 - n[i]*k_r, 1 - n[i+1]*k_r_next
            
            dn_ds = (sf_curr * ca.tan(xi[i]) + sf_next * ca.tan(xi[i+1])) / 2
            opti.subject_to(n[i+1] == n[i] + ds * dn_ds)
            
            dxi_ds_curr = kappa[i] * sf_curr / ca.cos(xi[i]) - k_r
            dxi_ds_next = kappa[i] * sf_next / ca.cos(xi[i+1]) - k_r_next 
            opti.subject_to(xi[i+1] == xi[i] + 0.5 * ds * (dxi_ds_curr + dxi_ds_next))
            
            dv_ds_curr = a_long_kin[i] * sf_curr / (v_curr * ca.cos(xi[i]))
            dv_ds_next = a_long_kin[i] * sf_next / (v_next * ca.cos(xi[i+1]))
            opti.subject_to(v[i+1] == v[i] + 0.5 * ds * (dv_ds_curr + dv_ds_next))

            opti.subject_to(n[i] <= w_left[i] - temp_margin)
            opti.subject_to(n[i] >= -w_right[i] + temp_margin)
            
            # 【核心修复】用宽松条件代替原有的硬阻挡限制 (MCL极大缓解了奇异性)
            opti.subject_to(1 - n[i]*k_r >= 0.05)
            
            v_safe = ca.fmax(v_curr, cfg['v_min'])
            theta, phi = slope_ref[i], bank_ref[i]
            q = 0.5 * rho * A * v_safe**2
            
            F_z = m * g * ca.cos(theta) * ca.cos(phi) + q * Cl
            F_lat_tire = m * v_safe**2 * kappa[i] + m * g * ca.sin(phi)
            F_long_tire = m * a_long_kin[i] + q * Cd + m * g * ca.sin(theta)
            
            friction_usage = (F_lat_tire**2 + F_long_tire**2) / ((mu * F_z)**2 + 1.0)
            opti.subject_to(friction_usage <= cfg['friction_utilization']**2)
            opti.subject_to(F_long_tire <= F_drive_max)
            opti.subject_to(F_long_tire * v_safe <= P_max)

        if self.track.is_closed:
            opti.subject_to(n[0] == n[N_points])
            opti.subject_to(xi[0] == xi[N_points])
            opti.subject_to(v[0] == v[N_points])

        if initial_guess is not None:
            opti.set_initial(n, np.clip(initial_guess['n'], -w_right + temp_margin, w_left - temp_margin))
            opti.set_initial(xi, initial_guess['xi'])
            opti.set_initial(v, np.clip(initial_guess['v'], cfg['v_min'], cfg['v_max']))
            opti.set_initial(a_long_kin, initial_guess['a'])
            opti.set_initial(kappa, initial_guess['kappa'])
        else:
            n_g, xi_g, v_g, a_g, k_g = self._generate_initial_guess(s_vals, k_ref, w_left, w_right, ds, cfg)
            opti.set_initial(n, n_g); opti.set_initial(xi, xi_g); opti.set_initial(v, v_g)
            opti.set_initial(a_long_kin, a_g); opti.set_initial(kappa, k_g)

        opts = {"expand": True, "ipopt.max_iter": cfg['ipopt_max_iter'], "ipopt.tol": cfg['ipopt_tol'], 
                "ipopt.print_level": cfg['ipopt_print_level'], "ipopt.sb": "yes", "ipopt.linear_solver": "mumps", 
                "ipopt.bound_relax_factor": 1e-6}
        opti.solver("ipopt", opts)

        try:
            sol = opti.solve()
            
            # 【重要】为了兼容绘图和存档，提取并附加全局 X, Y 坐标
            n_opt = sol.value(n)
            x_opt, y_opt = np.zeros(N_points+1), np.zeros(N_points+1)
            for j, s_val in enumerate(s_vals):
                x_opt[j], y_opt[j], _ = self.track.get_position(s_val, n_opt[j])
                
            return {
                'success': True, 's': s_vals, 'n': n_opt, 'xi': sol.value(xi), 
                'v': sol.value(v), 'a': sol.value(a_long_kin), 'kappa': sol.value(kappa), 
                'time': sol.value(T_total), 'N_points': N_points, 'x': x_opt, 'y': y_opt
            }
        except Exception as e:
            status = opti.debug.return_status()
            print(f"  ⚠ 求解异常: {status}, 详情: {e}")
            return None

    def _generate_initial_guess(self, s_vals, k_ref, w_left, w_right, ds, cfg):
        N = len(s_vals) - 1
        p, mu, g = self.car.params, self.car.params['mu'], self.car.params['g']
        v_guess = np.zeros(N + 1)
        for i in range(N + 1):
            k_abs = abs(k_ref[i])
            v_guess[i] = cfg['v_max'] * 0.9 if k_abs < 1e-4 else min(cfg['v_max'], np.sqrt(mu * g / (k_abs + 1e-5)) * 0.85)
        
        max_dec = p['force_max'] / p['mass']
        for i in range(N - 1, -1, -1):
            v_guess[i] = min(v_guess[i], np.sqrt(v_guess[i+1]**2 + 2 * max_dec * ds))
        for i in range(1, N + 1):
            v_guess[i] = min(v_guess[i], np.sqrt(v_guess[i-1]**2 + 2 * (p['force_max'] / p['mass']) * ds))

        v_guess = np.clip(gaussian_filter1d(v_guess, sigma=5), cfg['v_min'], cfg['v_max'])
        n_guess = np.zeros(N + 1)
        for i in range(N + 1):
            if abs(k_ref[i]) > 0.002:
                target_side = -np.sign(k_ref[i])
                limit = w_right[i] if target_side < 0 else w_left[i]
                n_guess[i] = target_side * (limit - cfg['margin']) * 0.6
        n_guess = np.clip(gaussian_filter1d(n_guess, sigma=10), -w_right + cfg['margin'], w_left - cfg['margin'])

        return n_guess, np.zeros(N + 1), v_guess, np.zeros(N), np.zeros(N)