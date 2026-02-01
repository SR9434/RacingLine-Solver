import sys
import os
import argparse
import time
import traceback

from src.track_io import TrackIO
from src.model import RaceCar
from src.optimizer import TrackOptimizer
from src.dashboard import quick_visualize
from src.result_io import save_result
from src.launcher import launch_gui, PRECISION_PRESETS

def calculate_grid_points(track_length, precision_key):
    preset = PRECISION_PRESETS[precision_key]
    N = int(track_length / preset['density'])
    return max(preset['min_points'], min(N, preset['max_points']))

def run_simulation(config):
    track_path = config['track_path']
    car_path = config['car_path']
    precision = config['precision']
    
    print(f"\n{'='*60}")
    print(f"  RACING LINE OPTIMIZER")
    print(f"{'='*60}")
    print(f"  Track: {os.path.basename(track_path)}")
    print(f"  Car:   {os.path.basename(car_path) if car_path else 'Default (F1 2026)'}")
    print(f"  Mode:  {PRECISION_PRESETS[precision]['name']}")
    print(f"{'='*60}\n")
    
    # 1. Load Track
    try:
        track = TrackIO.load_track(track_path)
    except Exception as e:
        print(f"[Error] Failed to load track: {e}")
        return None
        
    # 2. Track Preview (Optional)
    if config.get('show_boundary', False):
        print("[System] Generating 3D track preview...")
        try:
            track.diagnose()
            track.plot_3d_preview()
        except Exception as e:
            print(f"[Warning] Preview failed: {e}")

    # 3. Load Car
    if car_path and os.path.exists(car_path):
        car = RaceCar.from_file(car_path)
    else:
        print("[System] Using default vehicle parameters")
        car = RaceCar()
        if not car.params:
             car = RaceCar(RaceCar.default_params())

    # 4. Optimizer Setup
    N_points = calculate_grid_points(track.total_length, precision)
    optimizer = TrackOptimizer(track, car)
    
    # 5. Solve
    start_time = time.time()
    try:
        result = optimizer.solve(N_points=N_points)
    except Exception as e:
        print(f"[Error] Optimizer crashed: {e}")
        traceback.print_exc()
        return None
        
    elapsed = time.time() - start_time
    print(f"[System] Calculation time: {elapsed:.2f}s")
    
    if result is None or not result.get('success', False):
        print("[Error] Optimization failed to converge.")
        return None
    
    # 6. Save & Return
    if config.get('save_result', True):
        track_name = os.path.basename(track_path)
        save_result(track_name, result, track, car)
        
    return track, car, result

def main():
    parser = argparse.ArgumentParser(description="Racing Line Optimizer")
    parser.add_argument("--track", "-t", type=str, help="Path to track CSV file")
    parser.add_argument("--car", "-c", type=str, help="Path to car JSON file")
    parser.add_argument("--precision", "-p", choices=list(PRECISION_PRESETS.keys()), default='medium')
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI")
    
    args = parser.parse_args()
    
    # CLI Mode
    if args.no_gui:
        if not args.track:
            print("[Error] --track argument is required in no-gui mode.")
            return
        
        config = {
            'track_path': args.track,
            'car_path': args.car,
            'precision': args.precision,
            'save_result': True,
            'show_boundary': False
        }
        
        ret = run_simulation(config)
        if ret:
            track, car, result = ret
            quick_visualize(track, car, result)
        return

    # GUI Mode
    while True:
        config = launch_gui()
        if config is None:
            print("[System] Exiting...")
            break
            
        sim_data = run_simulation(config)
        
        if sim_data:
            track, car, result = sim_data
            print("\n[System] Visualizing results... (Close window to return to menu)")
            quick_visualize(track, car, result)
            print("[System] Returning to menu...\n")
        else:
            print("[System] Simulation failed. Returning to menu...")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[System] Interrupted by user.")
    except Exception as e:
        print(f"\n[Fatal Error] {e}")
        traceback.print_exc()