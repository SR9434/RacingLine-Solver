"""
Racing Line Optimizer - Result I/O
===================================
保存和加载优化结果
"""

import os
import json
import numpy as np
from datetime import datetime


class ResultIO:
    """优化结果的保存与加载"""
    
    RESULTS_DIR = "results"
    
    @classmethod
    def save(cls, track_name: str, result: dict, track_info: dict = None, 
             car_info: dict = None) -> str:
        """
        保存优化结果
        
        :param track_name: 赛道名称（不含路径）
        :param result: optimizer.solve() 返回的字典
        :param track_info: 赛道信息（可选）
        :param car_info: 车辆信息（可选）
        :return: 保存的文件路径
        """
        # 创建结果目录
        os.makedirs(cls.RESULTS_DIR, exist_ok=True)
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = os.path.splitext(track_name)[0]
        filename = f"{base_name}_{timestamp}.json"
        filepath = os.path.join(cls.RESULTS_DIR, filename)
        
        # 准备保存数据
        save_data = {
            'meta': {
                'version': '2.0',
                'timestamp': datetime.now().isoformat(),
                'track_name': track_name,
            },
            'track_info': track_info or {},
            'car_info': car_info or {},
            'result': {
                'success': result.get('success', False),
                'lap_time': float(result.get('time', 0)),
                's': result['s'].tolist() if isinstance(result.get('s'), np.ndarray) else result.get('s', []),
                'n': result['n'].tolist() if isinstance(result.get('n'), np.ndarray) else result.get('n', []),
                'xi': result['xi'].tolist() if isinstance(result.get('xi'), np.ndarray) else result.get('xi', []),
                'v': result['v'].tolist() if isinstance(result.get('v'), np.ndarray) else result.get('v', []),
                'a': result['a'].tolist() if isinstance(result.get('a'), np.ndarray) else result.get('a', []),
                'kappa': result['kappa'].tolist() if isinstance(result.get('kappa'), np.ndarray) else result.get('kappa', []),
            },
            'statistics': cls._compute_statistics(result)
        }
        
        # 保存为 JSON
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(save_data, f, indent=2, ensure_ascii=False)
        
        print(f"[ResultIO] ✓ 结果已保存: {filepath}")
        return filepath
    
    @classmethod
    def _compute_statistics(cls, result: dict) -> dict:
        """计算统计信息"""
        if not result.get('success', False):
            return {}
        
        v = np.array(result.get('v', []))
        a = np.array(result.get('a', []))
        kappa = np.array(result.get('kappa', []))
        
        if len(v) == 0:
            return {}
        
        # 计算 G 力
        g = 9.81
        g_long = a / g if len(a) > 0 else np.array([0])
        g_lat = (v[:-1]**2 * np.abs(kappa)) / g if len(kappa) > 0 and len(v) > 1 else np.array([0])
        
        return {
            'v_max_ms': float(np.max(v)),
            'v_min_ms': float(np.min(v)),
            'v_avg_ms': float(np.mean(v)),
            'v_max_kmh': float(np.max(v) * 3.6),
            'v_min_kmh': float(np.min(v) * 3.6),
            'v_avg_kmh': float(np.mean(v) * 3.6),
            'g_long_max': float(np.max(g_long)) if len(g_long) > 0 else 0,
            'g_long_min': float(np.min(g_long)) if len(g_long) > 0 else 0,
            'g_lat_max': float(np.max(np.abs(g_lat))) if len(g_lat) > 0 else 0,
        }
    
    @classmethod
    def load(cls, filepath: str) -> dict:
        """
        加载保存的结果
        
        :param filepath: 结果文件路径
        :return: 结果字典
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Result file not found: {filepath}")
        
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # 转换回 numpy 数组
        result = data.get('result', {})
        for key in ['s', 'n', 'xi', 'v', 'a', 'kappa']:
            if key in result and isinstance(result[key], list):
                result[key] = np.array(result[key])
        
        result['time'] = result.pop('lap_time', 0)
        
        return {
            'meta': data.get('meta', {}),
            'track_info': data.get('track_info', {}),
            'car_info': data.get('car_info', {}),
            'result': result,
            'statistics': data.get('statistics', {})
        }
    
    @classmethod
    def list_results(cls, track_filter: str = None) -> list:
        """
        列出所有保存的结果
        
        :param track_filter: 可选的赛道名称过滤
        :return: 结果文件列表
        """
        if not os.path.exists(cls.RESULTS_DIR):
            return []
        
        results = []
        for f in os.listdir(cls.RESULTS_DIR):
            if f.endswith('.json'):
                if track_filter is None or track_filter in f:
                    filepath = os.path.join(cls.RESULTS_DIR, f)
                    try:
                        with open(filepath, 'r') as file:
                            data = json.load(file)
                        results.append({
                            'filename': f,
                            'filepath': filepath,
                            'timestamp': data.get('meta', {}).get('timestamp', 'unknown'),
                            'track': data.get('meta', {}).get('track_name', 'unknown'),
                            'lap_time': data.get('result', {}).get('lap_time', 0),
                            'success': data.get('result', {}).get('success', False),
                        })
                    except:
                        pass
        
        # 按时间排序
        results.sort(key=lambda x: x['timestamp'], reverse=True)
        return results


def save_result(track_name, result, track=None, car=None):
    """便捷保存函数"""
    track_info = {}
    if track is not None:
        track_info = {
            'total_length': float(track.total_length),
            'n_points': int(track.N),
            'is_closed': bool(track.is_closed),
        }
    
    car_info = {}
    if car is not None:
        car_info = car.params.copy()
    
    return ResultIO.save(track_name, result, track_info, car_info)