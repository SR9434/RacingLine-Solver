import numpy as np
import json
import os

class RaceCar:
    
    def __init__(self, params=None):
        self.params = params if params is not None else {}

    def validate(self):
        required_keys = [
            'mass', 'g', 'mu', 
            'rho', 'Cd', 'Cl', 'A', 
            'P_max', 'force_max'
        ]
        missing = [key for key in required_keys if key not in self.params]
        if missing:
            print(f"[RaceCar] Warning: Missing parameters: {missing}")
            return False
        return True

    def to_dict(self):
        return self.params.copy()

    def save_to_file(self, filepath):
        try:
            directory = os.path.dirname(filepath)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
                
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self.params, f, indent=4)
            print(f"[RaceCar] Model saved to {filepath}")
            return True
        except Exception as e:
            print(f"[RaceCar] Error saving model: {e}")
            return False

    @classmethod
    def from_file(cls, filepath):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                params = json.load(f)
            print(f"[RaceCar] Model loaded from {filepath}")
            return cls(params)
        except Exception as e:
            print(f"[RaceCar] Error loading model: {e}")
            return cls()

    def get_limits(self, v, kappa):
        """
        计算物理极限
        输入: v (m/s), kappa (1/m)
        输出: acc_min (m/s^2), acc_max (m/s^2)
        """
        # 确保参数完整性
        if not self.params:
            raise ValueError("RaceCar parameters are empty. Load a vehicle file first.")

        p = self.params
        v_safe = np.maximum(np.abs(v), 1.0)
        
        # 1. 垂直载荷
        F_aero = 0.5 * p['rho'] * p['Cl'] * p['A'] * v**2
        F_z = p['mass'] * p['g'] + F_aero
        
        # 2. 最大抓地力
        F_tire_max = p['mu'] * F_z
        
        # 3. 横向力需求
        F_lat = p['mass'] * v**2 * np.abs(kappa)
        
        # 4. 剩余纵向抓地力 (摩擦圆)
        F_long_available = np.sqrt(np.maximum(F_tire_max**2 - F_lat**2, 0.0))
        
        # 5. 空气阻力
        F_drag = 0.5 * p['rho'] * p['Cd'] * p['A'] * v**2
        
        # 6. 引擎力限制
        F_eng_max = np.minimum(p['P_max'] / v_safe, p['force_max'])
        
        # --- 计算加速度限制 ---
        acc_max = (np.minimum(F_eng_max, F_long_available) - F_drag) / p['mass']
        acc_min = (-F_long_available - F_drag) / p['mass']
        
        return acc_min, acc_max