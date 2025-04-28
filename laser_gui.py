import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import json
import os

# Slider de rango con dos 'thumbs'
class RangeSlider(tk.Canvas):
    def __init__(self, master, from_=0, to=3600, start=0, end=0, command=None, **kwargs):
        super().__init__(master, height=30, **kwargs)
        self.from_, self.to = from_, to
        self.start, self.end = start, end
        self.command = command
        self.r = 8  # radio handle
        self.active = None
        self.bind("<Button-1>", self.click)
        self.bind("<B1-Motion>", self.drag)
        self.bind("<Configure>", lambda e: self.draw())
        self.draw()

    def draw(self):
        self.delete("all")
        w = self.winfo_width() or 500
        h = self.winfo_height()
        m = self.r
        # línea base
        self.create_line(m, h/2, w-m, h/2, fill="gray", width=4)
        # posiciones
        def pos(v): return m + (w-2*m)*(v-self.from_)/(self.to-self.from_)
        x1, x2 = pos(self.start), pos(self.end)
        # rango resaltado
        self.create_line(x1, h/2, x2, h/2, fill="blue", width=6)
        # handles
        self.create_oval(x1-m, h/2-m, x1+m, h/2+m, fill="white", outline="black", tags="start")
        self.create_oval(x2-m, h/2-m, x2+m, h/2+m, fill="white", outline="black", tags="end")

    def click(self, e):
        cs = self.coords("start")[0] + self.r
        ce = self.coords("end")[0]   + self.r
        self.active = "start" if abs(e.x-cs) < abs(e.x-ce) else "end"
        self.drag(e)

    def drag(self, e):
        w = self.winfo_width() or 500
        m = self.r
        val = self.from_ + (self.to-self.from_)*min(max(e.x-m,0), w-2*m)/(w-2*m)
        if self.active=="start":
            self.start = int(min(val, self.end))
        else:
            self.end   = int(max(val, self.start))
        self.draw()
        if self.command:
            self.command(self.start, self.end)

class LaserGUI:
    CONFIG_FILE = "laser_config.json"

    def __init__(self, master):
        self.master = master
        master.title("Control de Láser ESP32")
        master.columnconfigure(0, weight=1)

        # Estado de conexión
        self.status = ttk.Label(master, text="No conectado", foreground="red")
        self.status.grid(row=0, column=0, sticky="w", padx=10, pady=5)

        # Contenedores de estado
        self.serial_port = None
        self.vars    = []  # [(IntVar start, IntVar end, BooleanVar enabled), ...]
        self.sliders= []  # [RangeSlider, ...]

        # Puerto y botones
        top = ttk.Frame(master)
        top.grid(row=1, column=0, sticky="ew", padx=10)
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="Puerto Serial:").grid(row=0, column=0, padx=(0,2))
        self.port_cb = ttk.Combobox(top, values=self.scan_ports(), state="readonly")
        self.port_cb.grid(row=0, column=1, sticky="ew", padx=2)
        ttk.Button(top, text="Actualizar", command=self.update_ports).grid(row=0, column=2, padx=2)
        ttk.Button(top, text="Conectar", command=self.connect).grid(row=0, column=3, padx=2)
        ttk.Button(top, text="Guardar Config", command=self.save_config).grid(row=0, column=4, padx=2)

        # Tabla de espejos
        table = ttk.Frame(master)
        table.grid(row=2, column=0, sticky="nsew", padx=10, pady=10)
        for c in range(5): table.columnconfigure(c, weight=1)
        headers = ["M", "On", "Inicio", "Fin", "Rango"]
        for i, h in enumerate(headers):
            ttk.Label(table, text=h).grid(row=0, column=i)

        # Filas dinámicas
        for i in range(12):
            v_s = tk.IntVar(value=0)
            v_e = tk.IntVar(value=0)
            v_en = tk.BooleanVar(value=False)
            self.vars.append((v_s, v_e, v_en))

            ttk.Label(table, text=str(i+1)).grid(row=i+1, column=0)
            chk = ttk.Checkbutton(table, variable=v_en, command=self.on_value_change)
            chk.grid(row=i+1, column=1)

            sb_s = ttk.Spinbox(table, from_=0, to=3600, increment=1,
                               textvariable=v_s, width=5,
                               command=self.on_value_change)
            sb_e = ttk.Spinbox(table, from_=0, to=3600, increment=1,
                               textvariable=v_e, width=5,
                               command=self.on_value_change)
            sb_s.grid(row=i+1, column=2, padx=2)
            sb_e.grid(row=i+1, column=3, padx=2)

            rs = RangeSlider(table, from_=0, to=3600,
                             start=v_s.get(), end=v_e.get(),
                             command=lambda s,e, idx=i: self.on_slider_move(idx, s, e), width=1200)
            rs.grid(row=i+1, column=4, sticky="ew", padx=2)
            self.sliders.append(rs)

        # Carga config existente
        self.load_config()
        master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports or ["(ninguno)"]

    def update_ports(self):
        self.port_cb["values"] = self.scan_ports()
        messagebox.showinfo("Puertos", "Lista actualizada.")

    def connect(self):
        port = self.port_cb.get()
        if port in ("", "(ninguno)"):
            messagebox.showerror("Error", "Selecciona un puerto válido.")
            return
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            self.status.config(text=f"Conectado: {port}", foreground="green")
            self.on_value_change()
        except PermissionError:
            messagebox.showerror("Permiso Denegado",
                                 f"No se pudo abrir {port} (Acceso denegado).\n"
                                 "Cierra apps o ejecuta con permisos elevados.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_slider_move(self, idx, start, end):
        v_s, v_e, _ = self.vars[idx]
        v_s.set(start)
        v_e.set(end)
        self.on_value_change()

    def on_value_change(self):
        if not (self.serial_port and self.serial_port.is_open):
            return
        vals = []
        for (v_s, v_e, v_en), rs in zip(self.vars, self.sliders):
            if v_en.get():
                vals += [v_s.get(), v_e.get()]
            else:
                vals += [0, 0]
            # sincroniza slider con spinboxes
            rs.start = v_s.get()
            rs.end   = v_e.get()
            rs.draw()
        cmd = "UPDATE," + ",".join(map(str, vals)) + "\n"
        try:
            self.serial_port.write(cmd.encode())
            self.serial_port.readline()
        except:
            pass

    def save_config(self):
        data = []
        for v_s, v_e, v_en in self.vars:
            data.append({
                "start":   v_s.get(),
                "end":     v_e.get(),
                "enabled": v_en.get()
            })
        with open(self.CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=2)
        messagebox.showinfo("Guardado", "Configuración guardada en JSON.")

    def load_config(self):
        if os.path.exists(self.CONFIG_FILE):
            try:
                with open(self.CONFIG_FILE) as f:
                    data = json.load(f)
                for i, entry in enumerate(data[:12]):
                    v_s, v_e, v_en = self.vars[i]
                    v_s.set(entry.get("start", 0))
                    v_e.set(entry.get("end",   0))
                    v_en.set(entry.get("enabled", False))
                    rs = self.sliders[i]
                    rs.start = v_s.get()
                    rs.end   = v_e.get()
                    rs.draw()
            except:
                messagebox.showwarning("JSON", "Error al cargar configuración.")

    def on_closing(self):
        if messagebox.askyesno("Salir", "¿Guardar antes de salir?"):
            self.save_config()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = LaserGUI(root)
    root.mainloop()
