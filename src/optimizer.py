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
        
        # 默认配置
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
        
        if config:
            self.config.update(config)
        
        self._precompute_track_properties()

    def _precompute_track_properties(self):
        test_points = max(500, int(self.track.total_length / 2.0))
        s_test = np.linspace(0, self.track.total_length, test_points)
        k = self.track.get_curvature(s_test)
        
        self.track_max_curvature = np.max(np.abs(k))
        self.kappa_max = max(0.1, self.track_max_curvature * 1.5 + 0.1)

    def solve(self, N_points=None, precision='medium'):
        if N_points is None:
            if precision in self.PRECISION_CONFIGS:
                N_points = self.PRECISION_CONFIGS[precision]['N']
            else:
                N_points = 400
        
        if self.track.total_length < 10:
            print(f"[Optimizer] 错误: 赛道长度异常 ({self.track.total_length:.2f}m)")
            return None
        
        if self.config['use_multi_resolution'] and N_points > 60:
            print(f"[Optimizer] 策略: 多分辨率求解 (目标 N={N_points})")
            return self._solve_multi_resolution(N_points)
        else:
            print(f"[Optimizer] 策略: 单次直接求解 (N={N_points})")
            return self._solve_single(N_points, initial_guess=None)

    def _solve_multi_resolution(self, target_N):
        resolutions = []
        current_N = max(50, min(100, int(target_N * 0.6)))
        
        while current_N < target_N:
            resolutions.append(current_N)
            next_N = int(current_N * 1.8)
            if next_N >= target_N * 0.9:
                current_N = target_N
            else:
                current_N = next_N
        
        if not resolutions or resolutions[-1] != target_N:
            resolutions.append(target_N)
        
        if len(resolutions) == 1 and resolutions[0] == target_N:
             return self._solve_single(target_N, initial_guess=None)

        result = None
        for i, N in enumerate(resolutions):
            print(f"  Stage {i+1}/{len(resolutions)}: N={N}")
            
            initial_guess = None
            if result is not None and result.get('success'):
                initial_guess = self._interpolate_solution(result, N)
            
            result = self._solve_single(N, initial_guess)
            
            if result is None or not result.get('success'):
                if i < len(resolutions) - 1:
                     print("  ⚠ Stage 求解失败，尝试重置猜测继续...")
                     result = None 
                else:
                     break
        return result

    def _interpolate_solution(self, coarse_result, fine_N):
        s_coarse = coarse_result['s']
        s_fine = np.linspace(0, self.track.total_length, fine_N + 1)
    
        w_left_fine = np.zeros(fine_N + 1)
        w_right_fine = np.zeros(fine_N + 1)
        for i, s in enumerate(s_fine):
            wl, wr = self.track.get_track_width(s)
            w_left_fine[i] = wl
            w_right_fine[i] = wr
    
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
            's': s_fine,
            'n': n_clipped,
            'xi': np.clip(interp_xi(s_fine), -np.pi/3, np.pi/3),
            'v': v_clipped,
            'a': interp_a(s_fine_control),
            'kappa': np.clip(interp_kappa(s_fine_control), -self.kappa_max, self.kappa_max),
        }

    def _solve_single(self, N_points, initial_guess=None):
        start_time = time.time()
        cfg = self.config
        
        s_vals = np.linspace(0, self.track.total_length, N_points + 1)
        ds = s_vals[1] - s_vals[0]
        
        k_ref = np.zeros(N_points + 1)
        w_left = np.zeros(N_points + 1)
        w_right = np.zeros(N_points + 1)
        slope_ref = np.zeros(N_points + 1)
        bank_ref = np.zeros(N_points + 1)

        for i, s in enumerate(s_vals):
            k_ref[i] = self.track.get_curvature(s)
            wl, wr = self.track.get_track_width(s)
            w_left[i] = wl; w_right[i] = wr
            slope_ref[i] = self.track.get_slope(s)
            bank_ref[i] = self.track.get_banking(s)
            
        min_width = min(w_left.min(), w_right.min())
        if min_width < cfg['margin'] * 1.1:
            temp_margin = min_width * 0.8
        else:
            temp_margin = cfg['margin']
        
        opti = ca.Opti()

        # Decision Variables
        n = opti.variable(N_points + 1)
        xi = opti.variable(N_points + 1)
        v = opti.variable(N_points + 1)
        a_long_kin = opti.variable(N_points)
        kappa = opti.variable(N_points)
        
        # Parameters
        p = self.car.params
        m = p['mass']; g = p['g']
        mu = p['mu']; rho = p['rho']
        Cd = p['Cd']; Cl = p['Cl']
        A = p['A']; P_max = p['P_max']
        F_drive_max = p['force_max']

        # Objective
        T_total = 0
        for i in range(N_points):
            v_mid = (v[i] + v[i+1]) / 2
            xi_mid = (xi[i] + xi[i+1]) / 2
            n_mid = (n[i] + n[i+1]) / 2
            
            spatial_factor = ca.fmax(1 - n_mid * k_ref[i], 0.05)
            v_along = ca.fmax(v_mid * ca.cos(xi_mid), 1.0)
            
            dt = (ds * spatial_factor) / v_along
            T_total += dt
        
        opti.minimize(T_total + 1e-4 * ca.sumsqr(kappa) + 1e-5 * ca.sumsqr(a_long_kin))

        # Basic Constraints
        opti.subject_to(v >= cfg['v_min'])
        opti.subject_to(v <= cfg['v_max'])
        opti.subject_to(xi >= -np.pi / 3)
        opti.subject_to(xi <= np.pi / 3)
        opti.subject_to(kappa >= -self.kappa_max)
        opti.subject_to(kappa <= self.kappa_max)
        
        # Kinematics Integration (Trapezoidal)
        for i in range(N_points):
            k_r = k_ref[i]
            k_r_next = k_ref[i + 1]
            
            v_curr = v[i]; v_next = v[i+1]
            sf_curr = 1 - n[i]*k_r
            sf_next = 1 - n[i+1]*k_r_next
            
            # n'
            dn_ds = (sf_curr * ca.tan(xi[i]) + sf_next * ca.tan(xi[i+1])) / 2
            opti.subject_to(n[i+1] == n[i] + ds * dn_ds)
            
            # xi'
            dxi_ds_curr = kappa[i] * sf_curr / ca.cos(xi[i]) - k_r
            dxi_ds_next = kappa[i] * sf_next / ca.cos(xi[i+1]) - k_r_next 
            opti.subject_to(xi[i+1] == xi[i] + 0.5 * ds * (dxi_ds_curr + dxi_ds_next))
            
            # v'
            dv_ds_curr = a_long_kin[i] * sf_curr / (v_curr * ca.cos(xi[i]))
            dv_ds_next = a_long_kin[i] * sf_next / (v_next * ca.cos(xi[i+1]))
            opti.subject_to(v[i+1] == v[i] + 0.5 * ds * (dv_ds_curr + dv_ds_next))

            # Boundaries
            opti.subject_to(n[i] <= w_left[i] - temp_margin)
            opti.subject_to(n[i] >= -w_right[i] + temp_margin)
            
            # 3D Physics
            v_safe = ca.fmax(v_curr, cfg['v_min'])
            theta = slope_ref[i]
            phi = bank_ref[i]
            
            q = 0.5 * rho * A * v_safe**2
            F_drag = q * Cd
            F_downforce = q * Cl
            
            F_z = m * g * ca.cos(theta) * ca.cos(phi) + F_downforce
            F_grip_max = mu * F_z
            
            F_lat_tire = m * v_safe**2 * kappa[i] + m * g * ca.sin(phi)
            F_long_tire = m * a_long_kin[i] + F_drag + m * g * ca.sin(theta)
            
            # Friction Circle
            friction_usage = (F_lat_tire**2 + F_long_tire**2) / (F_grip_max**2 + 1.0)
            opti.subject_to(friction_usage <= cfg['friction_utilization']**2)
            
            # Power
            opti.subject_to(F_long_tire <= F_drive_max)
            opti.subject_to(F_long_tire * v_safe <= P_max)
            
            if abs(k_r) > 0.01:
                opti.subject_to(n[i] * k_r <= 0.85)

        # Loop Closure
        opti.subject_to(n[0] == n[N_points])
        opti.subject_to(xi[0] == xi[N_points])
        opti.subject_to(v[0] == v[N_points])

        # Initialization
        if initial_guess is not None:
            n_guess = np.clip(initial_guess['n'], -w_right + temp_margin, w_left - temp_margin)
            v_guess = np.clip(initial_guess['v'], cfg['v_min'], cfg['v_max'])
            
            opti.set_initial(n, n_guess)
            opti.set_initial(xi, initial_guess['xi'])
            opti.set_initial(v, v_guess)
            opti.set_initial(a_long_kin, initial_guess['a'])
            opti.set_initial(kappa, initial_guess['kappa'])
        else:
            n_g, xi_g, v_g, a_g, k_g = self._generate_initial_guess(
                s_vals, k_ref, w_left, w_right, ds, cfg
            )
            opti.set_initial(n, n_g)
            opti.set_initial(xi, xi_g)
            opti.set_initial(v, v_g)
            opti.set_initial(a_long_kin, a_g)
            opti.set_initial(kappa, k_g)

        opts = {
            "expand": True,
            "ipopt.max_iter": cfg['ipopt_max_iter'],
            "ipopt.tol": cfg['ipopt_tol'],
            "ipopt.print_level": cfg['ipopt_print_level'],
            "ipopt.sb": "yes",
            "ipopt.linear_solver": "mumps",
            "ipopt.bound_relax_factor": 1e-6 
        }
        opti.solver("ipopt", opts)

        try:
            sol = opti.solve()
            return {
                'success': True,
                's': s_vals,
                'n': sol.value(n),
                'xi': sol.value(xi),
                'v': sol.value(v),
                'a': sol.value(a_long_kin),
                'kappa': sol.value(kappa),
                'time': sol.value(T_total),
                'N_points': N_points
            }
        except:
            status = opti.debug.return_status()
            print(f"  ⚠ 求解异常: {status}")
            return None

    def _generate_initial_guess(self, s_vals, k_ref, w_left, w_right, ds, cfg):
        N = len(s_vals) - 1
        p = self.car.params
        mu = p['mu']
        g = p['g']
        
        v_guess = np.zeros(N + 1)
        for i in range(N + 1):
            k_abs = abs(k_ref[i])
            if k_abs < 1e-4:
                v_guess[i] = cfg['v_max'] * 0.9
            else:
                v_lim = np.sqrt(mu * g / (k_abs + 1e-5)) * 0.85
                v_guess[i] = min(cfg['v_max'], v_lim)
        
        max_dec = p['force_max'] / p['mass']
        for i in range(N - 1, -1, -1):
            v_next = v_guess[i+1]
            v_allow = np.sqrt(v_next**2 + 2 * max_dec * ds)
            v_guess[i] = min(v_guess[i], v_allow)
            
        for i in range(1, N + 1):
            v_prev = v_guess[i-1]
            a_avail = p['force_max'] / p['mass']
            v_allow = np.sqrt(v_prev**2 + 2 * a_avail * ds)
            v_guess[i] = min(v_guess[i], v_allow)

        v_guess = gaussian_filter1d(v_guess, sigma=5)
        v_guess = np.clip(v_guess, cfg['v_min'], cfg['v_max'])

        n_guess = np.zeros(N + 1)
        for i in range(N + 1):
            if abs(k_ref[i]) > 0.002:
                target_side = -np.sign(k_ref[i])
                limit = w_right[i] if target_side < 0 else w_left[i]
                n_guess[i] = target_side * (limit - cfg['margin']) * 0.6
        
        n_guess = gaussian_filter1d(n_guess, sigma=10)
        n_guess = np.clip(n_guess, -w_right + cfg['margin'], w_left - cfg['margin'])

        return (
            n_guess,
            np.zeros(N + 1),
            v_guess,
            np.zeros(N),
            np.zeros(N)
        )