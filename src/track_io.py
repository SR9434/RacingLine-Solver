import numpy as np
import os
from .track import Track

class TrackIO:
    """
    赛道文件读写工具 (v2.1 - Robust Loading)
    支持格式: CSV
    列定义: xi, yi, [zi], xo, yo, [zo]
    """

    @staticmethod
    def load_track(file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Track file not found: {file_path}")

        try:
            # 1. 简单的表头检测策略
            # 读取第一行，如果包含非数字字符（排除 'e', 'E', '.', '-', '+' 等科学计数法字符），则认为有表头
            with open(file_path, 'r', encoding='utf-8') as f:
                first_line = f.readline().strip()
            
            # 简单的启发式检查：如果有字母且不是科学计数法标记，通常是表头
            # 赛道构建器生成的头部是 "xi,yi,zi,xo,yo,zo"，包含 x, i, y, z, o
            has_header = False
            for char in first_line:
                if char.isalpha() and char.lower() not in ['e']: 
                    has_header = True
                    break
            
            # 2. 尝试读取数据
            try:
                data = np.loadtxt(file_path, delimiter=',', skiprows=1 if has_header else 0)
            except ValueError:
                # 如果判定无表头但读取失败，可能是判定错了（比如第一行数据损坏），或者反之
                # 尝试反向操作
                data = np.loadtxt(file_path, delimiter=',', skiprows=0 if has_header else 1)

            # 处理空文件或单行情况
            if data.ndim == 1:
                data = data.reshape(1, -1)
            
            # 3. 解析列
            cols = data.shape[1]
            
            if cols == 4:
                # 2D Data: xi, yi, xo, yo
                inner = data[:, 0:2]
                outer = data[:, 2:4]
                # Auto-fill Z=0
                inner_3d = np.column_stack([inner, np.zeros(len(inner))])
                outer_3d = np.column_stack([outer, np.zeros(len(outer))])
                print(f"[TrackIO] Loaded 2D track: {os.path.basename(file_path)}")
                
            elif cols == 6:
                # 3D Data: xi, yi, zi, xo, yo, zo
                inner_3d = data[:, 0:3]
                outer_3d = data[:, 3:6]
                print(f"[TrackIO] Loaded 3D track: {os.path.basename(file_path)}")
                
            else:
                raise ValueError(f"Invalid column count: {cols}. Expected 4 or 6.")

            return Track(inner_3d, outer_3d)

        except Exception as e:
            raise RuntimeError(f"Failed to load track '{file_path}': {e}")