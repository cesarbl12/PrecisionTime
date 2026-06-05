import tkinter as tk
import serial
import time

# --- CONFIGURACION ---
PUERTO = 'COM7'
BAUDRATE = 115200
TIEMPO_INICIAL_SEGUNDOS = 600.0  # 10 Minutos

class TemporizadorPTS900:
    def _init_(self, root):
        self.root = root
        self.root.title("PTS900 - Panel de Cronometraje")
        
        self.root.geometry("700x400")
        self.root.minsize(500, 300)
        self.root.configure(bg="#121212")

        self.tiempo_restante = TIEMPO_INICIAL_SEGUNDOS
        self.corriendo = False
        self.ultimo_tiempo = 0
        self.ser = None

        # --- INTERFAZ GRAFICA ---
        self.main_frame = tk.Frame(self.root, bg="#121212")
        self.main_frame.pack(expand=True, fill="both", padx=20, pady=20)

        self.lbl_titulo = tk.Label(self.main_frame, text="SISTEMA DE CRONOMETRAJE DEPORTIVO PTS900", 
                                   font=("Segoe UI", 16, "bold"), bg="#121212", fg="#E0E0E0")
        self.lbl_titulo.pack(pady=(10, 5))

        self.separador = tk.Frame(self.main_frame, bg="#333333", height=2)
        self.separador.pack(fill="x", padx=20, pady=10)

        self.lbl_tiempo = tk.Label(self.main_frame, text="10:00.0", 
                                   font=("Consolas", 90, "bold"), bg="#121212", fg="#00E676")
        self.lbl_tiempo.pack(expand=True)

        # --- BARRA DE ESTADO ---
        self.status_bar = tk.Frame(self.root, bg="#1F1F1F", height=35)
        self.status_bar.pack(side="bottom", fill="x")

        self.lbl_estado = tk.Label(self.status_bar, text="ESTADO: Iniciando sistema...", 
                                   font=("Segoe UI", 10), bg="#1F1F1F", fg="#A0A0A0")
        self.lbl_estado.pack(side="left", padx=15, pady=5)

        self.lbl_puerto = tk.Label(self.status_bar, text=f"LORA TX/RX: {PUERTO}", 
                                   font=("Segoe UI", 10, "bold"), bg="#1F1F1F", fg="#A0A0A0")
        self.lbl_puerto.pack(side="right", padx=15, pady=5)

        self.root.bind("<Configure>", self.escalar_interfaz)

        self.conectar_serial()
        self.actualizar_reloj()

    def escalar_interfaz(self, event):
        if event.widget == self.root:
            tamano_reloj = max(50, int(event.height / 4))
            tamano_titulo = max(12, int(event.width / 45))
            
            self.lbl_tiempo.configure(font=("Consolas", tamano_reloj, "bold"))
            self.lbl_titulo.configure(font=("Segoe UI", tamano_titulo, "bold"))

    def conectar_serial(self):
        try:
            self.ser = serial.Serial(PUERTO, BAUDRATE, timeout=0)
            self.lbl_estado.config(text="ESTADO: Conectado. Esperando comandos LoRa...", fg="#00BFFF")
        except serial.SerialException:
            self.lbl_estado.config(text=f"ERROR: Puerto {PUERTO} bloqueado.", fg="#FF1744")

    def actualizar_reloj(self):
        # 1. Calcular el tiempo exacto transcurrido
        if self.corriendo:
            ahora = time.time()
            delta = ahora - self.ultimo_tiempo
            self.tiempo_restante -= delta
            self.ultimo_tiempo = ahora

            if self.tiempo_restante <= 0:
                self.tiempo_restante = 0
                self.corriendo = False
                self.lbl_estado.config(text="ESTADO: TIEMPO AGOTADO", fg="#FF1744")

        # 2. Formato de visualizacion
        mins = int(self.tiempo_restante // 60)
        secs = int(self.tiempo_restante % 60)
        decimas = int((self.tiempo_restante - int(self.tiempo_restante)) * 10)

        self.lbl_tiempo.config(text=f"{mins:02d}:{secs:02d}.{decimas}")

        # 3. Colores dinamicos
        if self.tiempo_restante == 0:
            self.lbl_tiempo.config(fg="#FF1744")
        elif self.tiempo_restante <= 60 and self.corriendo:
            self.lbl_tiempo.config(fg="#FFEA00")
        elif self.corriendo:
            self.lbl_tiempo.config(fg="#00E676")
        else:
            self.lbl_tiempo.config(fg="#FFFFFF")

        # 4. Lectura del puerto serial
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting > 0:
                    linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not linea:
                        continue

                    # --- LOGICA DE COMANDOS LORA ---
                    if ":start:" in linea and not self.corriendo and self.tiempo_restante > 0:
                        self.corriendo = True
                        self.ultimo_tiempo = time.time()
                        self.lbl_estado.config(text="ESTADO: CRONOMETRO EN MARCHA", fg="#00E676")
                        
                    # Detener el reloj si recibe ':stop:' o ':silbatazo:'
                    elif (":stop:" in linea or ":silbatazo:" in linea) and self.corriendo:
                        self.corriendo = False
                        
                        # Mensaje especifico dependiendo de si fue boton o silbato
                        if ":silbatazo:" in linea:
                            self.lbl_estado.config(text="ESTADO: DETENIDO POR SILBATO ACUSTICO", fg="#FFEA00")
                        else:
                            self.lbl_estado.config(text="ESTADO: PAUSADO POR BOTON", fg="#FFEA00")
                        
            except Exception:
                self.lbl_estado.config(text="ESTADO: Perdida de conexion serial", fg="#FF1744")

        self.root.after(50, self.actualizar_reloj)

    def cerrar(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

if _name_ == "_main_":
    ventana = tk.Tk()
    app = TemporizadorPTS900(ventana)
    ventana.protocol("WM_DELETE_WINDOW", app.cerrar)
    ventana.mainloop()