import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

MAX_POINTS = 200
SERIAL_BAUD = 115200

DARK = "#1e1e2e"
SURFACE = "#2a2a3e"
PRIMARY = "#7c3aed"
PRIMARY_LIGHT = "#a78bfa"
TEXT = "#e0e0e0"
TEXT_DIM = "#888899"
SUCCESS = "#34d399"
ERROR = "#ef4444"
CARD = "#313148"
ACCENT2 = "#f59e0b"

class PidTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Tuner – Gimbal 2D (Cascada Pos+Vel)")
        self.root.geometry("1400x780")
        self.root.configure(bg=DARK)

        self.ser = None
        self.running = False

        self.time_data = deque(maxlen=MAX_POINTS)
        self.angleX_data = deque(maxlen=MAX_POINTS)
        self.angleY_data = deque(maxlen=MAX_POINTS)
        self.spX_data = deque(maxlen=MAX_POINTS)
        self.spY_data = deque(maxlen=MAX_POINTS)
        self.velX_data = deque(maxlen=MAX_POINTS)
        self.velY_data = deque(maxlen=MAX_POINTS)
        self.start_time = time.time()

        # Posicion y velocidad para cada eje
        self.pid = {
            'XP': {'Kp': 10.0, 'Ki': 0.0, 'Kd': 0.2},
            'XV': {'Kp': 8.0,  'Ki': 0.05,'Kd': 0.0},
            'YP': {'Kp': 10.0, 'Ki': 0.0, 'Kd': 0.2},
            'YV': {'Kp': 8.0,  'Ki': 0.05,'Kd': 0.0},
        }

        self._setup_styles()
        self._build_ui()
        self._list_ports()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        for s in [".", "TFrame", "TLabel", "TLabelFrame", "TLabelframe.Label",
                  "TButton", "TCheckbutton", "TScale"]:
            style.configure(s, background=DARK, foreground=TEXT)
        style.configure("TFrame", background=DARK)
        style.configure("TLabelframe", background=DARK, foreground=TEXT,
                        bordercolor=CARD, lightcolor=CARD, darkcolor=CARD)
        style.configure("TLabelframe.Label", background=DARK, foreground=TEXT,
                        font=("Helvetica", 11, "bold"))
        style.configure("TButton", background=PRIMARY, foreground="white",
                        borderwidth=0, focusthickness=0, font=("Helvetica", 10))
        style.map("TButton", background=[("active", PRIMARY_LIGHT)])
        style.configure("Success.TLabel", foreground=SUCCESS)
        style.configure("Error.TLabel", foreground=ERROR)
        style.configure("Large.TEntry", foreground="black", fieldbackground="white",
                        font=("Helvetica", 12), padding=4)

    def _safe_after(self, ms, func):
        try:
            if self.root.winfo_exists():
                self.root.after(ms, func)
        except:
            pass

    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ctrl = ttk.LabelFrame(main, text="Control PID en Cascada", width=340)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        ctrl.pack_propagate(False)

        f_conn = ttk.Frame(ctrl)
        f_conn.pack(fill=tk.X, pady=(8, 4), padx=6)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(f_conn, textvariable=self.port_var, width=16,
                                       font=("Helvetica", 11))
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(f_conn, text="R", width=2, command=self._list_ports).pack(side=tk.LEFT, padx=(0, 4))
        self.btn_connect = ttk.Button(f_conn, text="Conectar", command=self._toggle_connection)
        self.btn_connect.pack(side=tk.LEFT)

        self._pid_section(ctrl, "EJE X (ROLL) - Posicion", 'XP')
        self._pid_section(ctrl, "EJE X (ROLL) - Velocidad", 'XV')
        self._pid_section(ctrl, "EJE Y (PITCH) - Posicion", 'YP')
        self._pid_section(ctrl, "EJE Y (PITCH) - Velocidad", 'YV')

        self.status = ttk.Label(ctrl, text="Desconectado", style="Error.TLabel",
                                font=("Helvetica", 9))
        self.status.pack(pady=(4, 4))

        plot_frame = ttk.Frame(main)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 7), facecolor=DARK)

        def style_ax(ax):
            ax.set_facecolor(SURFACE)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.spines["left"].set_color(TEXT_DIM)
            ax.spines["bottom"].set_color(TEXT_DIM)
            ax.tick_params(colors=TEXT_DIM, labelsize=8)
            ax.title.set_color(TEXT)
            ax.title.set_fontsize(10)
            ax.yaxis.label.set_color(TEXT_DIM)
            ax.xaxis.label.set_color(TEXT_DIM)

        style_ax(self.ax1)
        self.ax1.set_title("ROLL (X) - Angulo")
        self.ax1.set_ylabel("Grados")
        self.ax1.axhline(y=0, color=TEXT_DIM, linestyle="--", linewidth=0.5, alpha=0.5)
        self.line_ax1, = self.ax1.plot([], [], color="#60a5fa", label="Angulo", linewidth=1.5)
        self.line_sp1, = self.ax1.plot([], [], color="#f472b6", linestyle="--", label="Setpoint", linewidth=1)
        self.ax1.legend(loc="upper right", fontsize=7, facecolor=CARD, labelcolor=TEXT, edgecolor="none")
        self.ax1.grid(True, alpha=0.15, color=TEXT_DIM)

        style_ax(self.ax2)
        self.ax2.set_title("PITCH (Y) - Angulo")
        self.ax2.set_ylabel("Grados")
        self.ax2.axhline(y=0, color=TEXT_DIM, linestyle="--", linewidth=0.5, alpha=0.5)
        self.line_ay2, = self.ax2.plot([], [], color="#34d399", label="Angulo", linewidth=1.5)
        self.line_sp2, = self.ax2.plot([], [], color="#f472b6", linestyle="--", label="Setpoint", linewidth=1)
        self.ax2.legend(loc="upper right", fontsize=7, facecolor=CARD, labelcolor=TEXT, edgecolor="none")
        self.ax2.grid(True, alpha=0.15, color=TEXT_DIM)

        style_ax(self.ax3)
        self.ax3.set_title("Velocidad Encoder (rev/s)")
        self.ax3.set_ylabel("rev/s")
        self.ax3.set_xlabel("Tiempo (s)")
        self.ax3.axhline(y=0, color=TEXT_DIM, linestyle="--", linewidth=0.5, alpha=0.5)
        self.line_vx, = self.ax3.plot([], [], color="#60a5fa", label="Roll", linewidth=1.2)
        self.line_vy, = self.ax3.plot([], [], color="#34d399", label="Pitch", linewidth=1.2)
        self.ax3.legend(loc="upper right", fontsize=7, facecolor=CARD, labelcolor=TEXT, edgecolor="none")
        self.ax3.grid(True, alpha=0.15, color=TEXT_DIM)

        plt.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.get_tk_widget().configure(bg=DARK)

    def _pid_section(self, parent, label, pid_key):
        f = ttk.LabelFrame(parent, text=label)
        f.pack(fill=tk.X, pady=2, padx=6)
        color = ACCENT2 if "Velocidad" in label else TEXT

        for pname in ["Kp", "Ki", "Kd"]:
            row = ttk.Frame(f)
            row.pack(fill=tk.X, pady=2)
            lbl = ttk.Label(row, text=pname, width=4, font=("Helvetica", 10, "bold"), foreground=color)
            lbl.pack(side=tk.LEFT)
            var = tk.StringVar(value=f"{self.pid[pid_key][pname]:.3f}")
            entry = ttk.Entry(row, textvariable=var, width=10, justify=tk.RIGHT, style="Large.TEntry")
            entry.pack(side=tk.RIGHT, padx=(4, 0))
            var.trace("w", lambda *args, k=pid_key, p=pname, v=var: self._on_entry(k, p, v))

    def _on_entry(self, pid_key, param, var):
        try:
            val = float(var.get())
            if val < -10000 or val > 10000:
                return
            self.pid[pid_key][param] = val
            self._send_pid(pid_key)
        except ValueError:
            pass

    def _list_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.running = False
            self.ser.close()
            self.btn_connect.config(text="Conectar")
            self.status.config(text="Desconectado", style="Error.TLabel")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get(), SERIAL_BAUD, timeout=0.1)
                time.sleep(2)
                self.ser.reset_input_buffer()
                self.running = True
                self.btn_connect.config(text="Desconectar")
                self.status.config(text="Conectado", style="Success.TLabel")
                threading.Thread(target=self._read_serial, daemon=True).start()
                threading.Thread(target=self._update_plot_loop, daemon=True).start()
            except Exception as e:
                self.status.config(text=f"Error: {e}", foreground=ERROR)

    def _send_pid(self, pid_key):
        if not self.ser or not self.ser.is_open:
            return
        kp = self.pid[pid_key].get("Kp", 0)
        ki = self.pid[pid_key].get("Ki", 0)
        kd = self.pid[pid_key].get("Kd", 0)
        cmd = f"{pid_key} {kp:.3f} {ki:.3f} {kd:.3f}\n"
        try:
            self.ser.write(cmd.encode())
        except:
            pass

    def _read_serial(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                self._parse_line(line)
            except:
                pass

    def _parse_line(self, line):
        try:
            parts = line.split(",")
            d = {}
            for p in parts:
                if ":" in p:
                    k, v = p.split(":", 1)
                    d[k] = float(v)
            now = time.time() - self.start_time
            self.time_data.append(now)
            roll = d.get("X", 0)
            pitch = d.get("Y", 0)
            self.angleX_data.append(roll)
            self.angleY_data.append(pitch)
            self.spX_data.append(d.get("SPX", 0))
            self.spY_data.append(d.get("SPY", 0))
            self.velX_data.append(d.get("VX", 0))
            self.velY_data.append(d.get("VY", 0))
            pwm_x = int(d.get("PWMX", 0))
            pwm_y = int(d.get("PWMY", 0))
            enc_x = int(d.get("ENCX", 0))
            enc_y = int(d.get("ENCY", 0))
            vx = d.get("VX", 0)
            vy = d.get("VY", 0)
            self._safe_after(0, lambda: self.status.config(
                text=f"Roll {roll:+.1f}  Pitch {pitch:+.1f}  |  PWM {pwm_x}/{pwm_y}  |  V {vx:.1f}/{vy:.1f} rev/s"
            ))
        except:
            pass

    def _update_plot(self):
        if not self.time_data:
            return
        self.line_ax1.set_data(list(self.time_data), list(self.angleX_data))
        self.line_sp1.set_data(list(self.time_data), list(self.spX_data))
        self.line_ay2.set_data(list(self.time_data), list(self.angleY_data))
        self.line_sp2.set_data(list(self.time_data), list(self.spY_data))
        self.line_vx.set_data(list(self.time_data), list(self.velX_data))
        self.line_vy.set_data(list(self.time_data), list(self.velY_data))
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.relim()
            ax.autoscale_view()
        try:
            self.canvas.draw_idle()
        except:
            pass

    def _update_plot_loop(self):
        while self.running:
            time.sleep(0.1)
            self._safe_after(0, self._update_plot)


if __name__ == "__main__":
    root = tk.Tk()
    app = PidTuner(root)
    root.mainloop()
