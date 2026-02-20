import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import subprocess

# Configuration Presets
PRECISION_PRESETS = {
    'test': {'name': 'Test (Fast)', 'density': 25.0, 'min_points': 30, 'max_points': 80, 'time': '~5s'},
    'low': {'name': 'Low Precision', 'density': 15.0, 'min_points': 50, 'max_points': 200, 'time': '~15s'},
    'medium': {'name': 'Medium (Standard)', 'density': 8.0, 'min_points': 150, 'max_points': 500, 'time': '~1min'},
    'high': {'name': 'High Precision', 'density': 4.0, 'min_points': 300, 'max_points': 1000, 'time': '~3min'},
    'ultra': {'name': 'Ultra (Production)', 'density': 2.0, 'min_points': 500, 'max_points': 2000, 'time': '~10min+'}
}

class LauncherGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Racing Line Solver")
        self.root.geometry("600x650")
        self.root.resizable(False, False)
        
        # Theme
        self.bg_color = "#1a1a2e"
        self.panel_color = "#16213e"
        self.fg_color = "#ffffff"
        self.accent_color = "#4ecdc4"
        self.root.configure(bg=self.bg_color)
        
        # State
        self.selected_track_path = None
        self.selected_car_path = None
        self.config = None
        
        # Ensure directories exist
        self.project_root = os.getcwd()
        os.makedirs(os.path.join(self.project_root, "tracks"), exist_ok=True)
        os.makedirs(os.path.join(self.project_root, "cars"), exist_ok=True)
        os.makedirs(os.path.join(self.project_root, "images"), exist_ok=True) # Ensure images folder exists
        
        self._build_ui()
        self._center_window()

    def _center_window(self):
        self.root.update_idletasks()
        w, h = self.root.winfo_width(), self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (w // 2)
        y = (self.root.winfo_screenheight() // 2) - (h // 2)
        self.root.geometry(f'{w}x{h}+{x}+{y}')

    def _build_ui(self):
        tk.Label(self.root, text="🏎️ Racing Line Solver", font=("Arial", 20, "bold"), 
                 bg=self.bg_color, fg=self.accent_color).pack(pady=(20, 5))
        
        main_frame = tk.Frame(self.root, bg=self.bg_color)
        main_frame.pack(fill='both', expand=True, padx=30, pady=10)
        
        self._create_section(main_frame, "📍 Track Selection", self._build_track_ui)
        self._create_section(main_frame, "🚘 Vehicle Selection", self._build_car_ui)
        self._create_section(main_frame, "⚙️ Simulation Precision", self._build_precision_ui)
        self._create_section(main_frame, "📋 Options", self._build_options_ui)
        
        btn_frame = tk.Frame(self.root, bg=self.bg_color)
        btn_frame.pack(side='bottom', fill='x', padx=30, pady=20)
        
        self.btn_start = tk.Button(btn_frame, text="▶ START SIMULATION", font=("Arial", 12, "bold"),
                                   bg=self.accent_color, fg="black", height=2, cursor="hand2",
                                   command=self._on_start)
        self.btn_start.pack(side='left', fill='x', expand=True, padx=(0, 10))
        
        tk.Button(btn_frame, text="Exit", font=("Arial", 12), bg="#555", fg="white", 
                  height=2, width=10, command=self._on_quit).pack(side='right')

    def _create_section(self, parent, title, builder_func):
        frame = tk.LabelFrame(parent, text=f" {title} ", font=("Arial", 10, "bold"),
                              bg=self.panel_color, fg=self.fg_color, padx=10, pady=10)
        frame.pack(fill='x', pady=8)
        builder_func(frame)

    def _build_track_ui(self, parent):
        btn_frame = tk.Frame(parent, bg=self.panel_color)
        btn_frame.pack(fill='x', pady=(0, 5))
        
        tk.Button(btn_frame, text="🛠️ New Track", bg="#333", fg="white", width=12,
                  command=self._launch_track_builder).pack(side='left', padx=(0, 5))
        tk.Button(btn_frame, text="📂 Load CSV...", bg="#444", fg="white", width=12,
                  command=self._load_track_file).pack(side='left')
        
        self.lbl_track = tk.Label(parent, text="No track selected", bg=self.panel_color, fg="#888", anchor="w")
        self.lbl_track.pack(fill='x', pady=(5, 0))

    def _build_car_ui(self, parent):
        btn_frame = tk.Frame(parent, bg=self.panel_color)
        btn_frame.pack(fill='x', pady=(0, 5))
        
        tk.Button(btn_frame, text="🛠️ Edit Car", bg="#333", fg="white", width=12,
                  command=self._launch_vehicle_editor).pack(side='left', padx=(0, 5))
        tk.Button(btn_frame, text="📂 Load JSON...", bg="#444", fg="white", width=12,
                  command=self._load_car_file).pack(side='left')
        
        self.lbl_car = tk.Label(parent, text="Default (F1 2026)", bg=self.panel_color, fg="#888", anchor="w")
        self.lbl_car.pack(fill='x', pady=(5, 0))

    def _build_precision_ui(self, parent):
        self.var_precision = tk.StringVar(value='medium')
        frame = tk.Frame(parent, bg=self.panel_color)
        frame.pack(fill='x')
        
        col = 0
        for key in ['test', 'low', 'medium', 'high', 'ultra']:
            rb = tk.Radiobutton(frame, text=key.capitalize(), variable=self.var_precision, value=key,
                                bg=self.panel_color, fg="white", selectcolor=self.panel_color,
                                activebackground=self.panel_color, activeforeground=self.accent_color)
            rb.grid(row=0, column=col, sticky='w')
            col += 1
            
        self.lbl_precision = tk.Label(parent, text="", bg=self.panel_color, fg="#888", font=("Arial", 9))
        self.lbl_precision.pack(anchor='w', pady=(5,0))
        self.var_precision.trace('w', self._update_precision_info)
        self._update_precision_info()

    def _build_options_ui(self, parent):
        self.var_save = tk.BooleanVar(value=True)
        self.var_boundary = tk.BooleanVar(value=False)
        
        tk.Checkbutton(parent, text="Save Results to /results", variable=self.var_save,
                       bg=self.panel_color, fg="white", selectcolor=self.panel_color, 
                       activebackground=self.panel_color).pack(anchor='w')
        tk.Checkbutton(parent, text="Show 3D Track Preview", variable=self.var_boundary,
                       bg=self.panel_color, fg="white", selectcolor=self.panel_color, 
                       activebackground=self.panel_color).pack(anchor='w')

    def _update_precision_info(self, *args):
        key = self.var_precision.get()
        info = PRECISION_PRESETS[key]
        self.lbl_precision.config(text=f"Details: {info['name']} | Approx Time: {info['time']}")

    def _launch_tool(self, tool_name, args=None):
        script_path = os.path.join(self.project_root, "tools", tool_name)
        if os.path.exists(script_path):
            cmd = [sys.executable, script_path]
            if args:
                cmd.extend(args)
            subprocess.Popen(cmd)
        else:
            messagebox.showerror("Error", f"Tool not found: {script_path}")

    def _launch_track_builder(self):
        # 询问用户是否加载背景图
        use_image = messagebox.askyesno(
            "New Track Setup", 
            "Do you want to load a background image (e.g., map screenshot)?\n\n"
            "YES: Select an image from /images folder.\n"
            "NO: Start with a blank canvas."
        )
        
        image_args = []
        if use_image:
            img_dir = os.path.join(self.project_root, "images")
            if not os.path.exists(img_dir): os.makedirs(img_dir)
            
            # 打开文件选择器
            img_path = filedialog.askopenfilename(
                initialdir=img_dir,
                title="Select Background Image",
                filetypes=[("Images", "*.png;*.jpg;*.jpeg;*.bmp")]
            )
            
            if img_path:
                image_args = [img_path]
            else:
                # 用户点了 Yes 但取消了文件选择，则不启动
                return

        self._launch_tool("track_builder.py", image_args)

    def _launch_vehicle_editor(self): 
        self._launch_tool("vehicle_editor.py")

    def _load_track_file(self):
        initial_dir = os.path.join(self.project_root, "tracks")
        path = filedialog.askopenfilename(initialdir=initial_dir, title="Select Track CSV", filetypes=[("CSV Files", "*.csv")])
        if path:
            self.selected_track_path = path
            self.lbl_track.config(text=f"✅ {os.path.basename(path)}", fg="#4ecdc4")

    def _load_car_file(self):
        initial_dir = os.path.join(self.project_root, "cars")
        path = filedialog.askopenfilename(initialdir=initial_dir, title="Select Car JSON", filetypes=[("JSON Files", "*.json")])
        if path:
            self.selected_car_path = path
            self.lbl_car.config(text=f"✅ {os.path.basename(path)}", fg="#4ecdc4")

    def _on_start(self):
        if not self.selected_track_path:
            messagebox.showwarning("Warning", "Please select a track file first!")
            return
        
        self.config = {
            'track_path': self.selected_track_path,
            'car_path': self.selected_car_path,
            'precision': self.var_precision.get(),
            'save_result': self.var_save.get(),
            'show_boundary': self.var_boundary.get()
        }
        self.root.destroy()

    def _on_quit(self):
        if messagebox.askyesno("Exit", "Are you sure you want to quit?"):
            self.config = None
            self.root.destroy()
            sys.exit(0)

    def run(self):
        self.root.mainloop()
        return self.config

def launch_gui():
    return LauncherGUI().run()

if __name__ == "__main__":
    launch_gui()