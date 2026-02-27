#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import sounddevice as sd
import numpy as np
import threading

# Import ROS 2 library
import rclpy
from rclpy.node import Node

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class MicVisualizer:
    def __init__(self, master):
        self.master = master
        master.title("Mic Tester: dBFS + Spectrum + Record/Playback")
        master.geometry("600x720")

        # --- Config ---
        self.sample_rate = 16000
        self.block_size = 2048
        self.device_index = None

        self.is_running = True

        # ---- dBFS meter config ----
        self.db_floor = -60.0  
        self.current_dbfs = self.db_floor
        self.peak_dbfs = self.db_floor

        # Audio buffers
        self.audio_buffer = np.zeros(self.block_size, dtype=np.float32)

        # Recording
        self.is_recording = False
        self.recorded_frames = []
        self.final_recording = None

        # -----------------------------
        # GUI: dBFS Meter
        # -----------------------------
        frame_top = tk.Frame(master)
        frame_top.pack(fill="x", padx=10, pady=8)

        tk.Label(frame_top, text="Loudness (dBFS)", font=("Arial", 12, "bold")).pack()

        self.lbl_dbfs = tk.Label(frame_top, text="RMS: -60.0 dBFS", font=("Courier", 18), fg="blue")
        self.lbl_dbfs.pack()

        self.lbl_peak = tk.Label(frame_top, text="Peak: -60.0 dBFS", font=("Courier", 12), fg="red")
        self.lbl_peak.pack()

        tk.Button(frame_top, text="Reset Peak", command=self.reset_peak, font=("Arial", 9)).pack(pady=3)

        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(frame_top, variable=self.progress_var, maximum=60.0, length=550)
        self.progress_bar.pack(pady=6)

        # -----------------------------
        # GUI: Spectrum
        # -----------------------------
        frame_mid = tk.Frame(master)
        frame_mid.pack(fill="both", expand=True, padx=10, pady=10)

        tk.Label(frame_mid, text="Frequency Spectrum (Real-time)", font=("Arial", 12, "bold")).pack()

        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_ylim(0, 20)
        self.ax.set_xlim(0, self.sample_rate / 2)
        self.ax.set_xlabel("Frequency (Hz)")
        self.ax.set_ylabel("Magnitude")
        self.ax.grid(True)

        self.x_freq = np.fft.rfftfreq(self.block_size, 1 / self.sample_rate)
        self.line, = self.ax.plot(self.x_freq, np.zeros(len(self.x_freq)))

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_mid)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # -----------------------------
        # GUI: Recording Controls
        # -----------------------------
        frame_bot = tk.Frame(master)
        frame_bot.pack(fill="x", padx=10, pady=10)

        ttk.Separator(frame_bot, orient="horizontal").pack(fill="x", pady=5)

        self.lbl_rec_status = tk.Label(frame_bot, text="Ready", font=("Arial", 10, "italic"))
        self.lbl_rec_status.pack()

        self.btn_record = tk.Button(
            frame_bot, text="🔴 Record (5s)", command=self.start_recording,
            bg="#ffcccc", font=("Arial", 11)
        )
        self.btn_record.pack(pady=5)

        self.btn_play = tk.Button(
            frame_bot, text="▶ Playback", command=self.start_playback,
            state="disabled", font=("Arial", 11)
        )
        self.btn_play.pack(pady=5)

        # --- Start Stream ---
        self.stream = sd.InputStream(
            device=self.device_index,
            channels=1,
            samplerate=self.sample_rate,
            blocksize=int(self.block_size / 2),
            callback=self.audio_callback,
        )
        self.stream.start()

        self.update_gui()

    def rms_to_dbfs(self, rms: float) -> float:
        eps = 1e-12
        db = 20.0 * np.log10(max(rms, eps))
        if db < self.db_floor:
            db = self.db_floor
        if db > 0.0:
            db = 0.0
        return float(db)

    def dbfs_to_progress(self, dbfs: float) -> float:
        dbfs = np.clip(dbfs, self.db_floor, 0.0)
        return float(dbfs - self.db_floor)

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            print(status)
        x = indata[:, 0].astype(np.float32)
        rms = float(np.sqrt(np.mean(x * x)))
        dbfs = self.rms_to_dbfs(rms)
        self.current_dbfs = dbfs
        if dbfs > self.peak_dbfs:
            self.peak_dbfs = dbfs
        shift = len(x)
        self.audio_buffer = np.roll(self.audio_buffer, -shift)
        self.audio_buffer[-shift:] = x
        if self.is_recording:
            self.recorded_frames.append(indata.copy())

    def update_gui(self):
        if not self.is_running:
            return
        self.lbl_dbfs.config(text=f"RMS: {self.current_dbfs:5.1f} dBFS")
        self.lbl_peak.config(text=f"Peak: {self.peak_dbfs:5.1f} dBFS")
        self.progress_var.set(self.dbfs_to_progress(self.current_dbfs))

        if self.current_dbfs > -6:
            self.lbl_dbfs.config(fg="red")
        elif self.current_dbfs > -18:
            self.lbl_dbfs.config(fg="orange")
        else:
            self.lbl_dbfs.config(fg="blue")

        fft_data = np.fft.rfft(self.audio_buffer)
        fft_mag = np.abs(fft_data) * 2 / self.block_size * 100
        self.line.set_ydata(fft_mag)
        self.canvas.draw_idle()
        self.master.after(50, self.update_gui)

    def reset_peak(self):
        self.peak_dbfs = self.db_floor

    def start_recording(self):
        self.recorded_frames = []
        self.is_recording = True
        self.btn_record.config(state="disabled", text="Recording...")
        self.btn_play.config(state="disabled")
        self.lbl_rec_status.config(text="Recording... Speak now!", fg="red")
        self.master.after(5000, self.stop_recording)

    def stop_recording(self):
        self.is_recording = False
        self.lbl_rec_status.config(text="Recording Finished.", fg="green")
        self.btn_record.config(state="normal", text="🔴 Record (5s)")
        if len(self.recorded_frames) > 0:
            self.final_recording = np.concatenate(self.recorded_frames, axis=0)
            self.btn_play.config(state="normal")

    def start_playback(self):
        if self.final_recording is None:
            return
        threading.Thread(target=self.playback_thread, daemon=True).start()

    def playback_thread(self):
        self.stream.stop()
        self.btn_play.config(state="disabled", text="Playing...")
        self.btn_record.config(state="disabled")
        self.lbl_rec_status.config(text="Playing sound...", fg="blue")
        try:
            sd.play(self.final_recording, self.sample_rate)
            sd.wait()
        except Exception as e:
            print(f"Playback error: {e}")
        self.stream.start()
        self.lbl_rec_status.config(text="Ready", fg="black")
        self.btn_play.config(state="normal", text="▶ Playback")
        self.btn_record.config(state="normal")

    def close(self):
        self.is_running = False
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass
        self.master.destroy()

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create a simple ROS 2 Node (useful for debugging in ros2 node list)
    node = Node('mic_checker_node')
    
    root = tk.Tk()
    app = MicVisualizer(root)
    
    def on_closing():
        app.close()
        node.destroy_node() # Clean up the node
        rclpy.shutdown()    # Shutdown ROS 2
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()