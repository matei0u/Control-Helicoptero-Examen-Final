import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time
import csv
import pygame 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIGURACIÓN VISUAL ---
plt.style.use('dark_background')
COLOR_BG = '#101010'
COLOR_MANUAL = '#00AAFF'
COLOR_PID = '#FFAA00'
COLOR_FG = '#00FFCC'
COLOR_BTN_UPD = '#0066CC'

# --- VARIABLES ---
BAUD_RATE = 115200
serial_conn = None
is_running = True
start_time_offset = None

# SENSIBILIDAD DEL JOYSTICK
JOY_SPEED = 0.4

# FACTOR DE SUAVIZADO DE GANANCIAS (0.01 = Muy lento, 1.0 = Instantáneo)
SMOOTH_FACTOR = 0.1 

# Listas de datos
times = []
val_target = []
val_angle = []
val_output = []

# Estado
current_mode = 0 
current_angle = 0.0
current_target = 0.0

# VALORES REALES (Leídos del ESP32 - Telemetría)
real_kp = 0.0
real_ki = 0.0
real_kd = 0.0

is_recording = False; csv_writer = None; csv_file = None
data_lock = threading.Lock()

def serial_worker():
    global start_time_offset, current_angle, current_target, current_mode, real_kp, real_ki, real_kd
    while is_running:
        if serial_conn and serial_conn.is_open:
            try:
                if serial_conn.in_waiting:
                    line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line and (line[0].isdigit() or line[0] == '-'):
                        parts = line.split(',')
                        if len(parts) >= 8:
                            ms = float(parts[0])
                            tgt = float(parts[1])
                            ang = float(parts[2])
                            out = float(parts[3])
                            mode = int(parts[4])
                            kp_in = float(parts[5])
                            ki_in = float(parts[6])
                            kd_in = float(parts[7])
                            
                            if start_time_offset is None: start_time_offset = ms
                            t_sec = (ms - start_time_offset) / 1000.0
                            
                            with data_lock:
                                times.append(t_sec)
                                val_target.append(tgt)
                                val_angle.append(ang)
                                val_output.append(out)
                                
                                current_angle = ang
                                current_target = tgt
                                current_mode = mode
                                real_kp = kp_in
                                real_ki = ki_in
                                real_kd = kd_in
                                
                                if is_recording and csv_writer:
                                    csv_writer.writerow([f"{t_sec:.3f}", f"{tgt:.2f}", f"{ang:.2f}", f"{out:.2f}", f"{mode}"])
            except: pass
        else: time.sleep(0.01)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("HELICÓPTERO 1-DOF | GAIN SCHEDULING REAL")
        self.root.geometry("1300x850")
        self.root.configure(bg=COLOR_BG)
        style = ttk.Style(); style.theme_use('clam')
        
        # --- VARIABLES PARA SUAVIZADO ---
        self.target_kp_val = 0.030
        self.target_ki_val = 0.925
        self.target_kd_val = 0.003
        
        self.sent_kp_val = 0.030
        self.sent_ki_val = 0.925
        self.sent_kd_val = 0.003
        # --------------------------------

        self.joystick = None
        self.init_joystick()

        # 1. HEADER
        f_top = tk.Frame(root, bg="#202020", pady=10)
        f_top.pack(fill=tk.X)
        tk.Label(f_top, text="PUERTO:", fg="white", bg="#202020").pack(side=tk.LEFT, padx=10)
        self.cbox = ttk.Combobox(f_top, width=10); self.cbox.pack(side=tk.LEFT)
        tk.Button(f_top, text="CONECTAR", command=self.conn_toggle, bg="green", fg="white").pack(side=tk.LEFT, padx=10)
        self.btn_rec = tk.Button(f_top, text="GRABAR CSV", command=self.rec_toggle, bg="#444", fg="white", state="disabled")
        self.btn_rec.pack(side=tk.LEFT, padx=20)
        
        self.lbl_joy = tk.Label(f_top, text="JOYSTICK: ---", fg="#555", bg="#202020", font=("Arial", 10, "bold"))
        self.lbl_joy.pack(side=tk.LEFT, padx=20)
        if self.joystick: self.lbl_joy.config(text="JOYSTICK: OK", fg=COLOR_FG)

        f_disp = tk.Frame(f_top, bg="#202020")
        f_disp.pack(side=tk.RIGHT, padx=20)
        self.lbl_ang = tk.Label(f_disp, text="ANG: 0.0°", font=("Consolas", 22, "bold"), fg=COLOR_FG, bg="#202020")
        self.lbl_ang.pack(side=tk.LEFT, padx=15)
        self.lbl_pwm = tk.Label(f_disp, text="OUT: 0%", font=("Consolas", 22, "bold"), fg="#AAA", bg="#202020")
        self.lbl_pwm.pack(side=tk.LEFT)

        # 2. GRÁFICA
        self.fig, self.ax = plt.subplots(facecolor=COLOR_BG)
        self.ax.set_facecolor(COLOR_BG); self.ax.grid(True, color='#333', linestyle='--')
        self.line_tgt, = self.ax.plot([], [], color=COLOR_MANUAL, label='Setpoint', linestyle='--')
        self.line_ang, = self.ax.plot([], [], color=COLOR_FG, label='Angulo', linewidth=1.5)
        self.ax.legend(loc='upper left', facecolor='#222', labelcolor='white')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 3. CONTROLES
        f_bot = tk.Frame(root, bg="#151515", pady=10)
        f_bot.pack(fill=tk.X)

        # A. MODOS
        f_mode = tk.LabelFrame(f_bot, text="MODO", bg="#151515", fg="white", padx=10)
        f_mode.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        self.var_mode = tk.StringVar(value="MANUAL")
        tk.Radiobutton(f_mode, text="MANUAL", variable=self.var_mode, value="MANUAL", command=self.set_mode_manual, bg="#151515", fg=COLOR_MANUAL, selectcolor="#333", font=("Arial", 11, "bold")).pack(anchor="w", pady=2)
        tk.Radiobutton(f_mode, text="AUTO PID", variable=self.var_mode, value="PID", command=self.set_mode_pid, bg="#151515", fg=COLOR_PID, selectcolor="#333", font=("Arial", 11, "bold")).pack(anchor="w", pady=2)

        # B. SLIDER
        self.f_slider = tk.LabelFrame(f_bot, text="CONTROL (0-100%)", bg="#151515", fg=COLOR_MANUAL, padx=10)
        self.f_slider.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        self.lbl_val = tk.Label(self.f_slider, text="0.0", font=("Arial", 20, "bold"), bg="#151515", fg="white")
        self.lbl_val.pack()
        self.slider = ttk.Scale(self.f_slider, from_=0, to=100, orient=tk.HORIZONTAL, command=self.on_slide)
        self.slider.pack(fill=tk.X, pady=5)
        
        f_inp = tk.Frame(self.f_slider, bg="#151515"); f_inp.pack(pady=2)
        self.ent_target = tk.Entry(f_inp, width=6, justify='center', font=("Arial", 12))
        self.ent_target.pack(side=tk.LEFT, padx=5)
        self.ent_target.bind('<Return>', self.set_target_from_entry)
        tk.Button(f_inp, text="IR", command=self.set_target_from_entry, bg="#333", fg="white", width=4).pack(side=tk.LEFT)
        
        tk.Button(self.f_slider, text="CORTAR (0)", command=self.reset_slider, bg="#555", fg="white").pack(pady=5)

        # C. SINTONÍA PID
        f_pid = tk.LabelFrame(f_bot, text="SINTONÍA PID", bg="#151515", fg="white", padx=10)
        f_pid.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        self.var_scheduling = tk.BooleanVar(value=True) 
        self.chk_sched = tk.Checkbutton(f_pid, text="Auto-Ajuste (Ángulo Real)", variable=self.var_scheduling, 
                                        bg="#151515", fg="orange", selectcolor="#333", font=("Arial", 9, "bold"))
        self.chk_sched.grid(row=0, column=0, columnspan=4, pady=5)

        self.ent_kp, self.lbl_real_kp = self.make_pid_row(f_pid, "Kp", "0.030", 1)
        self.ent_ki, self.lbl_real_ki = self.make_pid_row(f_pid, "Ki", "0.925", 2)
        self.ent_kd, self.lbl_real_kd = self.make_pid_row(f_pid, "Kd", "0.003", 3)
        
        tk.Button(f_pid, text="ACTUALIZAR\nSINTONÍA", command=self.manual_pid_update, 
                  bg=COLOR_BTN_UPD, fg="white", font=("Arial", 10, "bold"), width=15, height=2).grid(row=4, column=0, columnspan=3, pady=10)

        # D. EMERGENCIA
        tk.Button(f_bot, text="PARADA\nEMERGENCIA", command=self.emergency, bg="red", fg="white", font=("Arial", 12, "bold"), width=12, height=4).pack(side=tk.RIGHT, padx=10)

        self.scan_ports()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        threading.Thread(target=serial_worker, daemon=True).start()
        
        # INICIAR BUCLES
        self.poll_joystick()
        self.smooth_pid_update() 

    def init_joystick(self):
        try:
            pygame.init(); pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0); self.joystick.init()
        except: pass

    def poll_joystick(self):
        if self.joystick:
            pygame.event.pump()
            try: axis_val = self.joystick.get_axis(1)
            except: axis_val = 0.0
            if abs(axis_val) > 0.1:
                direction = -axis_val 
                step = direction * JOY_SPEED 
                current_val = self.slider.get()
                new_val = current_val + step
                limit = 100.0 if self.var_mode.get() == "MANUAL" else 70.0
                if new_val < 0: new_val = 0
                if new_val > limit: new_val = limit
                if new_val != current_val:
                    self.slider.set(new_val)
                    self.on_slide(new_val)
        self.root.after(50, self.poll_joystick)

    # --- SUAVIZADO + GAIN SCHEDULING (BASADO EN ÁNGULO REAL) ---
    def smooth_pid_update(self):
        # 1. ACTUALIZACIÓN AUTOMÁTICA BASADA EN ÁNGULO REAL (current_angle)
        if self.var_mode.get() == "PID" and self.var_scheduling.get():
            # Calcula ganancias según el ángulo REAL del sensor
            kp, ki, kd = self.calculate_gains(current_angle)
            
            # Establece los nuevos objetivos
            self.target_kp_val = kp
            self.target_ki_val = ki
            self.target_kd_val = kd
            
            # Actualiza visualmente las cajas si no las estás editando
            if self.root.focus_get() not in (self.ent_kp, self.ent_ki, self.ent_kd):
                self.ent_kp.delete(0, tk.END); self.ent_kp.insert(0, f"{kp:.5f}")
                self.ent_ki.delete(0, tk.END); self.ent_ki.insert(0, f"{ki:.5f}")
                self.ent_kd.delete(0, tk.END); self.ent_kd.insert(0, f"{kd:.6f}")

        # 2. LÓGICA DE SUAVIZADO (Acercar 'Sent' a 'Target')
        changed = False
        
        diff_p = self.target_kp_val - self.sent_kp_val
        if abs(diff_p) > 0.00001:
            self.sent_kp_val += diff_p * SMOOTH_FACTOR; changed = True
        else: self.sent_kp_val = self.target_kp_val

        diff_i = self.target_ki_val - self.sent_ki_val
        if abs(diff_i) > 0.00001:
            self.sent_ki_val += diff_i * SMOOTH_FACTOR; changed = True
        else: self.sent_ki_val = self.target_ki_val

        diff_d = self.target_kd_val - self.sent_kd_val
        if abs(diff_d) > 0.000001:
            self.sent_kd_val += diff_d * SMOOTH_FACTOR; changed = True
        else: self.sent_kd_val = self.target_kd_val

        if changed and serial_conn and serial_conn.is_open:
            self.send_pid_group(self.sent_kp_val, self.sent_ki_val, self.sent_kd_val)

        self.root.after(50, self.smooth_pid_update)

    def calculate_gains(self, angle):
        ang = angle
        if ang < 0: ang = 0
        if ang > 70: ang = 70
        new_kp = (-0.0004393 * ang) + 0.03082
        new_ki = (-0.013179 * ang) + 0.92464
        new_kd = (-0.0000492 * ang) + 0.003452
        return max(0, new_kp), max(0, new_ki), max(0, new_kd)

    def manual_pid_update(self):
        try:
            self.target_kp_val = float(self.ent_kp.get())
            self.target_ki_val = float(self.ent_ki.get())
            self.target_kd_val = float(self.ent_kd.get())
        except ValueError: pass

    def on_slide(self, val):
        val_float = float(val)
        self.lbl_val.config(text=f"{val_float:.1f}")
        if self.root.focus_get() != self.ent_target:
            self.ent_target.delete(0, tk.END); self.ent_target.insert(0, f"{val_float:.1f}")
        self.send('V', val)

    def make_pid_row(self, p, l, d, r):
        tk.Label(p, text=l, bg="#151515", fg="#AAA").grid(row=r, column=0, padx=5)
        e = tk.Entry(p, width=7, justify='center'); e.insert(0, d); e.grid(row=r, column=1)
        lbl = tk.Label(p, text="Act: -", bg="#151515", fg=COLOR_FG, font=("Arial", 8)); lbl.grid(row=r, column=2)
        return e, lbl

    def update_plot(self, frame):
        with data_lock:
            if times:
                self.line_tgt.set_data(times, val_target)
                self.line_ang.set_data(times, val_angle)
                self.ax.set_xlim(times[0], times[-1] + 1)
                self.ax.set_ylim(-10, 110)
                self.lbl_ang.config(text=f"ANG: {current_angle:.1f}°")
                self.lbl_pwm.config(text=f"OUT: {val_output[-1] if val_output else 0:.0f}%")
                
                self.lbl_real_kp.config(text=f"Act: {real_kp:.4f}")
                self.lbl_real_ki.config(text=f"Act: {real_ki:.4f}")
                self.lbl_real_kd.config(text=f"Act: {real_kd:.5f}")
        return self.line_tgt, self.line_ang

    def send_pid_group(self, p, i, d):
        self.send('P', p); self.send('I', i); self.send('D', d)

    def set_mode_manual(self):
        self.send('M', 0)
        self.f_slider.config(text="CONTROL PWM (0-100%)", fg=COLOR_MANUAL)
        self.slider.config(to=100); self.slider.set(0)
        self.line_tgt.set_color(COLOR_MANUAL)

    def set_mode_pid(self):
        self.send('M', 1)
        self.f_slider.config(text="CONTROL ÁNGULO (0-70°)", fg=COLOR_PID)
        self.slider.config(to=70); self.slider.set(0)
        self.line_tgt.set_color(COLOR_PID)

    def set_target_from_entry(self, event=None):
        try:
            val = float(self.ent_target.get())
            limit = 100 if self.var_mode.get() == "MANUAL" else 70
            if 0 <= val <= limit:
                self.slider.set(val); self.on_slide(val)
        except ValueError: pass

    def reset_slider(self):
        self.slider.set(0); self.send('V', 0)
        self.ent_target.delete(0, tk.END); self.ent_target.insert(0, "0")

    def emergency(self):
        self.slider.set(0); self.send('V', -1.0)
        self.ent_target.delete(0, tk.END); self.ent_target.insert(0, "0")

    def send(self, pre, val):
        if serial_conn and serial_conn.is_open:
            try: serial_conn.write(f"{pre}{float(val):.6f}\n".encode())
            except: pass

    def scan_ports(self):
        self.cbox['values'] = [p.device for p in serial.tools.list_ports.comports()]
        if self.cbox['values']: self.cbox.current(0)

    def conn_toggle(self):
        global serial_conn, start_time_offset
        if serial_conn and serial_conn.is_open:
            serial_conn.close(); self.btn_rec.config(state="disabled")
        else:
            try:
                serial_conn = serial.Serial(self.cbox.get(), BAUD_RATE)
                if not times: start_time_offset = None 
                time.sleep(2)
                self.btn_rec.config(state="normal")
                if self.var_mode.get() == "MANUAL": self.set_mode_manual()
                else: self.set_mode_pid()
            except: pass

    def rec_toggle(self):
        global is_recording, csv_writer, csv_file
        if not is_recording:
            csv_file = open(f"data_{int(time.time())}.csv", 'w', newline='')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["Time", "Target", "Angle", "Output", "Mode"])
            is_recording = True; self.btn_rec.config(bg="red", text="STOP CSV")
        else:
            is_recording = False; csv_file.close(); self.btn_rec.config(bg="#444", text="GRABAR")

if __name__ == "__main__": root = tk.Tk(); App(root); root.mainloop()