"""Hold-to-drive controls for the simulated RB."""
import time
import tkinter as tk
from tkinter import ttk


class ManualDrive:
    DIRECTIONS = {
        'Up': (.10, 0.), 'Down': (-.09, 0.),
        'Left': (0., .12), 'Right': (0., -.12),
    }

    def __init__(self, backend):
        self.backend = backend
        self.direction = None
        self.closed = False
        self.driving = False
        self.root = tk.Tk()
        self.root.title('RB · Điều khiển tay')
        self.root.geometry('440x235')
        panel = ttk.Frame(self.root, padding=14)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='Giữ phím mũi tên hoặc nút để di chuyển. Nhả phím để dừng.',
                  wraplength=410).pack()
        pad = ttk.Frame(panel)
        pad.pack(pady=12)
        for key, title, row, col in (
                ('Up', '↑ Tiến', 0, 1), ('Down', '↓ Lùi', 2, 1),
                ('Left', '← Quay trái', 1, 0), ('Right', 'Quay phải →', 1, 2)):
            button = ttk.Button(pad, text=title, takefocus=False)
            button.grid(row=row, column=col, padx=4, pady=2)
            button.bind('<ButtonPress-1>', lambda event, k=key: self.press(k))
            button.bind('<ButtonRelease-1>', lambda event, k=key: self.release(k))
            button.bind('<Leave>', lambda event, k=key: self.release(k))
            self.root.bind(f'<KeyPress-{key}>', lambda event, k=key: self.press(k))
            self.root.bind(f'<KeyRelease-{key}>', lambda event, k=key: self.release(k))
        ttk.Button(pad, text='Dừng · Esc', command=self.stop).grid(row=1, column=1)
        self.status = tk.StringVar(value='Chờ RB cân bằng')
        ttk.Label(panel, textvariable=self.status, wraplength=410).pack()
        self.root.bind('<Escape>', lambda event: self.stop())
        self.root.bind('<FocusOut>', lambda event: self.stop() if event.widget is self.root else None)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after_idle(self.root.lift)

    def press(self, direction):
        if self.ready:
            self.direction = direction
        return 'break'

    def release(self, direction):
        if self.direction == direction:
            self.stop()
        return 'break'

    @property
    def ready(self):
        return (self.backend.running and self.backend.telemetry.get('drive_ready') == '1'
                and time.monotonic()-self.backend.telemetry_received_monotonic_s < .5)

    def update(self):
        self.root.update()
        if self.closed:
            return
        ready = self.ready
        if self.direction and ready:
            self.backend.command_drive(*self.DIRECTIONS[self.direction])
            self.driving = True
        elif self.direction or self.driving:
            self.stop()
        drive_status = 'Sẵn sàng · giữ phím mũi tên để lái' if ready else 'Chờ RB cân bằng'
        tracking = self.backend.live_scan.tracking_status()
        self.status.set(drive_status + '\nSLAM: ' + tracking.get('message', 'Đang khởi động'))

    def stop(self):
        self.direction = None
        self.driving = False
        self.backend.command_stop()

    def close(self):
        self.stop()
        self.closed = True
        self.root.destroy()
