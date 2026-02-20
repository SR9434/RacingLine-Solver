import sys
import os
import json
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Adjust path to find src module
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from src.model import RaceCar

class VehicleEditor:
    CAR_PRESETS = {
        "F1 2026 (Base)": {
            'mass': 798.0, 'g': 9.81, 'mu': 1.8, 'rho': 1.225, 
            'Cd': 1.0, 'Cl': 3.5, 'A': 1.6, 'P_max': 750000.0, 'force_max': 15000.0
        },
        "GT3 (911 GT3 R)": {
            'mass': 1350.0, 'g': 9.81, 'mu': 1.6, 'rho': 1.225, 
            'Cd': 0.45, 'Cl': 1.2, 'A': 2.0, 'P_max': 410000.0, 'force_max': 10000.0
        },
        "Road Car (Civic Type R)": {
            'mass': 1520.0, 'g': 9.81, 'mu': 1.1, 'rho': 1.225, 
            'Cd': 0.33, 'Cl': 0.2, 'A': 2.2, 'P_max': 235000.0, 'force_max': 6000.0
        },
        "Kart": {
            'mass': 170.0, 'g': 9.81, 'mu': 1.1 ,'rho': 1.225, 
            'Cd': 0.80, 'Cl': 0.2, 'A': 0.6, 'P_max': 4800.0, 'force_max': 1100.0
        }
    }

    GROUPS = {
        "Mass & Environment": {'mass': "Mass (kg)", 'g': "Gravity (m/s^2)"},
        "Tires": {'mu': "Friction Coeff (mu)"},
        "Aerodynamics": {'rho': "Air Density", 'Cd': "Drag Coeff", 'Cl': "Lift/Downforce Coeff", 'A': "Frontal Area"},
        "Powertrain": {'P_max': "Max Power (W)", 'force_max': "Max Force (N)"}
    }

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Vehicle Editor")
        self.root.geometry("800x600")
        
        self.cars_dir = os.path.join(project_root, "cars")
        os.makedirs(self.cars_dir, exist_ok=True)
        
        self.car = RaceCar(self.CAR_PRESETS["F1 2026 (Base)"].copy())
        self.entries = {}
        self._setup_ui()
        self._load_params_to_ui()

    def _setup_ui(self):
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left Panel (Editor)
        left = ttk.Frame(paned)
        paned.add(left, weight=2)
        
        toolbar = ttk.Frame(left)
        toolbar.pack(fill=tk.X, pady=5)
        ttk.Button(toolbar, text="Open JSON", command=self.load_vehicle).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Save As...", command=self.save_vehicle).pack(side=tk.LEFT, padx=2)
        
        # Scrollable form
        canvas = tk.Canvas(left)
        scrollbar = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)
        
        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        r = 0
        for group, items in self.GROUPS.items():
            ttk.Label(scroll_frame, text=group, font=("Arial", 10, "bold")).grid(row=r, column=0, sticky="w", pady=(15,5), columnspan=2)
            r+=1
            for k, label in items.items():
                ttk.Label(scroll_frame, text=label).grid(row=r, column=0, sticky="w", padx=10)
                e = ttk.Entry(scroll_frame)
                e.grid(row=r, column=1, sticky="ew", padx=10)
                self.entries[k] = e
                r+=1

        # Right Panel (Presets)
        right = ttk.Frame(paned, padding=10)
        paned.add(right, weight=1)
        
        lf = ttk.LabelFrame(right, text="Quick Presets", padding=10)
        lf.pack(fill=tk.X)
        for name in self.CAR_PRESETS:
            ttk.Button(lf, text=name, command=lambda n=name: self.apply_preset(n)).pack(fill=tk.X, pady=2)

    def _load_params_to_ui(self):
        for k, e in self.entries.items():
            e.delete(0, tk.END)
            if k in self.car.params: e.insert(0, str(self.car.params[k]))

    def apply_preset(self, name):
        if messagebox.askyesno("Confirm", "Load preset? Unsaved changes will be lost."):
            self.car = RaceCar(self.CAR_PRESETS[name].copy())
            self._load_params_to_ui()

    def load_vehicle(self):
        path = filedialog.askopenfilename(initialdir=self.cars_dir, filetypes=[("JSON", "*.json")])
        if path:
            try:
                self.car = RaceCar.from_file(path)
                self._load_params_to_ui()
            except Exception as e:
                messagebox.showerror("Error", str(e))

    def save_vehicle(self):
        try:
            new_p = {}
            for k, e in self.entries.items(): new_p[k] = float(e.get())
            self.car.params = new_p
        except ValueError:
            messagebox.showerror("Error", "Invalid number format.")
            return

        path = filedialog.asksaveasfilename(initialdir=self.cars_dir, defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            self.car.save_to_file(path)

    def run(self): self.root.mainloop()

if __name__ == "__main__":
    VehicleEditor().run()