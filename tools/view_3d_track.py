import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 添加路径以复用 TrackIO
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.track_io import TrackIO

def view_track_3d(csv_path):
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        return

    print(f"Loading {csv_path}...")
    # 手动读取以便验证原始数据
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    
    inner = data[:, 0:3]
    outer = data[:, 3:6]
    
    center = (inner + outer) / 2
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制边线
    ax.plot(inner[:,0], inner[:,1], inner[:,2], 'r-', label='Inner Wall', lw=1)
    ax.plot(outer[:,0], outer[:,1], outer[:,2], 'b-', label='Outer Wall', lw=1)
    ax.plot(center[:,0], center[:,1], center[:,2], 'g--', label='Centerline', lw=0.5)
    
    # 绘制路面网格线 (每隔 10 个点)
    step = max(1, len(inner) // 50)
    for i in range(0, len(inner), step):
        ax.plot([inner[i,0], outer[i,0]], 
                [inner[i,1], outer[i,1]], 
                [inner[i,2], outer[i,2]], 'k-', alpha=0.3)
        
    # 计算统计信息
    z_min, z_max = np.min(inner[:,2]), np.max(inner[:,2])
    print(f"Elevation Range: {z_min:.2f}m to {z_max:.2f}m")
    
    ax.set_title(f"3D Track Preview: {os.path.basename(csv_path)}")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # 设置比例 (强制 xy 等比例，z 轴可能需要放大看清楚)
    # Matplotlib 3D 轴比例比较难搞，这里简单设置
    xy_mid = np.mean(center[:, :2], axis=0)
    max_range = np.max(center[:, :2]) - np.min(center[:, :2])
    ax.set_xlim(xy_mid[0] - max_range/2, xy_mid[0] + max_range/2)
    ax.set_ylim(xy_mid[1] - max_range/2, xy_mid[1] + max_range/2)
    ax.set_zlim(z_min - 10, z_max + 10)
    
    plt.legend()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # 默认尝试找最新的 csv
        track_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tracks')
        files = [f for f in os.listdir(track_dir) if f.endswith('.csv')]
        if files:
            latest_file = max([os.path.join(track_dir, f) for f in files], key=os.path.getctime)
            view_track_3d(latest_file)
        else:
            print("Usage: python view_3d_track.py <path_to_csv>")
    else:
        view_track_3d(sys.argv[1])