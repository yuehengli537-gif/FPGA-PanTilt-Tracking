"""Chinese desktop servo controls. Run: python -m desktop_control.app"""
from __future__ import annotations
import argparse
import math
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from .controller import ServoController

BG, INK, MUTED, BLUE, NAVY, WHITE = '#eef2f7', '#15263f', '#617086', '#2459d3', '#132642', '#ffffff'
FONT = 'Microsoft YaHei UI'

def number(value):
    return f'{value:.3f}'.rstrip('0').rstrip('.')

class ServoApp:
    def __init__(self, root, controller=None):
        self.root = root
        self.controller = controller or ServoController(directions_path=Path(__file__).resolve().parents[1] / 'reports/runtime/desktop_control/mount_directions.json')
        self.closed = False
        self._updating = False
        self._dirty = {'pan': False, 'tilt': False}
        self._limits_dirty = False
        self._was_connected = False
        self._home_sync = False
        self._ui_error = ''
        self._ports = []
        self._snapshot = {}
        self._log_open = self._limits_open = False
        self._targets = {a: tk.StringVar(value=str(v)) for a, v in (('pan', 90), ('tilt', 0))}
        self._scales = {a: tk.DoubleVar(value=v) for a, v in (('pan', 90), ('tilt', 0))}
        self._readbacks = {a: tk.StringVar(value='当前指令 —    板端目标 —') for a in ('pan', 'tilt')}
        self._ranges = {a: tk.StringVar() for a in ('pan', 'tilt')}
        self._limit_vars = {k: tk.StringVar(value=number(v)) for k, v in
            (('pan_min', 0), ('pan_max', 180), ('tilt_min', -45), ('tilt_max', 45))}
        self._direction_vars = {a: tk.BooleanVar(value=self.controller.snapshot()[a + '_reversed']) for a in ('pan', 'tilt')}
        self._directions_dirty = False
        self.auto_hint_var = tk.StringVar()
        self.profile_var = tk.StringVar(value='连接后读取范围；应用范围不会启用或移动舵机')
        self.demo_var = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value='未连接')
        self.output_var = tk.StringVar(value='PWM 状态未知')
        self.motion_var = tk.StringVar(value='等待连接')
        self.detail_var = tk.StringVar(value='连接后读取固件版本和板端限位')
        self.message_var = tk.StringVar()
        self._setup_styles()
        self._build()
        for axis, var in self._targets.items():
            var.trace_add('write', lambda *_args, a=axis: self._entry_changed(a))
        for var in self._limit_vars.values():
            var.trace_add('write', self._limit_changed)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh_ports()
        self._poll()

    def _setup_styles(self):
        self.root.title('基于FPGA的云台目标追踪系统 · 舵机控制台')
        # Work area includes the taskbar and uses the same coordinate space as Tk.
        left, top = 0, 0
        available_width = self.root.winfo_screenwidth()
        available_height = self.root.winfo_screenheight()
        if sys.platform == 'win32':
            try:
                import ctypes
                from ctypes import wintypes
                work_area = wintypes.RECT()
                if ctypes.windll.user32.SystemParametersInfoW(48, 0, ctypes.byref(work_area), 0):
                    left, top = work_area.left, work_area.top
                    available_width = work_area.right - work_area.left
                    available_height = work_area.bottom - work_area.top
            except (OSError, AttributeError):
                pass
        width = max(640, min(1120, available_width - 40))
        height = max(480, min(800, available_height - 100))
        x = left + max(0, (available_width - width) // 2)
        y = top + max(0, (available_height - height) // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')
        self.root.minsize(min(860, width), min(620, height))
        self.root.configure(bg=BG)
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('TFrame', background=BG)
        style.configure('Card.TFrame', background=WHITE)
        style.configure('Axis.TFrame', background='#f3f6fb')
        style.configure('TLabel', background=WHITE, foreground=INK, font=(FONT, 10))
        style.configure('Muted.TLabel', foreground=MUTED)
        style.configure('Heading.TLabel', font=(FONT, 12, 'bold'))
        style.configure('AxisTitle.TLabel', background='#f3f6fb', font=(FONT, 12, 'bold'))
        style.configure('AxisMuted.TLabel', background='#f3f6fb', foreground=MUTED, font=(FONT, 9))
        style.configure('TButton', font=(FONT, 10), padding=(12, 7), background='#eaf0fa', foreground=INK,
                        borderwidth=0, focusthickness=1, focuscolor=BLUE)
        style.map('TButton', background=[('active', '#dae5f6'), ('disabled', '#eef1f5')],
                  foreground=[('disabled', '#93a0b2')])
        style.configure('Primary.TButton', background=BLUE, foreground=WHITE, padding=(20, 9))
        style.map('Primary.TButton', background=[('active', '#1948b5'), ('disabled', '#d9e2f1')],
                  foreground=[('disabled', '#8192ad')])
        style.configure('Stop.TButton', background='#fff0df', foreground='#875312')
        style.configure('Link.TButton', background=WHITE, foreground=BLUE, padding=(0, 5), anchor='w')
        style.configure('TCheckbutton', background=WHITE, font=(FONT, 10), foreground=INK)
        style.map('TCheckbutton', background=[('active', WHITE)])
        style.configure('TEntry', padding=6, fieldbackground=WHITE)
        style.configure('TCombobox', padding=5, fieldbackground=WHITE)
        style.configure('Horizontal.TScale', background='#f3f6fb', troughcolor='#dae3f0')

    def _build(self):
        header = tk.Frame(self.root, bg=NAVY, padx=24, pady=15)
        header.pack(fill='x')
        self.badge = tk.Label(header, text='未连接设备', bg='#334863', fg='white', font=(FONT, 10, 'bold'), padx=14, pady=8)
        self.badge.pack(side='right')
        tk.Label(header, text='基于FPGA的云台目标追踪系统', bg=NAVY, fg=WHITE,
                 font=(FONT, 20, 'bold')).pack(anchor='w')
        tk.Label(header, text='舵机控制台  /  手动角度与红球跟随 · 画面输出至 HDMI 显示器', bg=NAVY, fg='#b8c9e4', font=(FONT, 10)).pack(anchor='w', pady=(4, 0))
        # Keep validation and device feedback visible while the body scrolls.
        status_bar = tk.Frame(self.root, bg=WHITE, padx=20, pady=10)
        status_bar.pack(side='bottom', fill='x')
        self.message_label = ttk.Label(status_bar, textvariable=self.message_var, wraplength=890, foreground=MUTED)
        self.message_label.pack(anchor='w', fill='x')
        status_bar.bind('<Configure>', lambda e: self.message_label.configure(wraplength=max(300, e.width - 40)))
        viewport = ttk.Frame(self.root)
        viewport.pack(fill='both', expand=True)
        scrollbar = ttk.Scrollbar(viewport, orient='vertical')
        scrollbar.pack(side='right', fill='y')
        self.canvas = tk.Canvas(viewport, background=BG, highlightthickness=0, yscrollcommand=scrollbar.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        scrollbar.configure(command=self.canvas.yview)
        self.body = ttk.Frame(self.canvas, padding=(20, 14, 20, 16))
        self.body_id = self.canvas.create_window((0, 0), window=self.body, anchor='nw')
        self.canvas.bind('<Configure>', lambda e: self.canvas.itemconfigure(self.body_id, width=e.width))
        self.body.bind('<Configure>', lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.root.bind('<MouseWheel>', self._mousewheel)
        connection = self._card()
        ttk.Label(connection, text='设备连接', style='Heading.TLabel').grid(row=0, column=0, sticky='w', padx=(0, 14))
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, state='readonly', width=42, font=(FONT, 10))
        self.port_box.grid(row=0, column=1, sticky='ew')
        self.refresh_button = ttk.Button(connection, text='刷新', command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(8, 12))
        self.demo_check = ttk.Checkbutton(connection, text='演示模式', variable=self.demo_var, command=self._demo_changed)
        self.demo_check.grid(row=0, column=3, padx=(0, 12))
        self.connect_button = ttk.Button(connection, text='连接设备', command=self.toggle_connection)
        self.connect_button.grid(row=0, column=4)
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, textvariable=self.detail_var, style='Muted.TLabel').grid(row=1, column=0, columnspan=5, sticky='w', pady=(9, 0))
        statuses = self._card(padding=(18, 12))
        for col, (title, var) in enumerate((('连接', self.status_var), ('输出', self.output_var), ('动作', self.motion_var))):
            box = ttk.Frame(statuses, style='Card.TFrame')
            box.grid(row=0, column=col, sticky='ew', padx=(0, 16))
            ttk.Label(box, text=title, style='Muted.TLabel').pack(anchor='w')
            ttk.Label(box, textvariable=var, font=(FONT, 11, 'bold')).pack(anchor='w', pady=(3, 0))
            statuses.columnconfigure(col, weight=1)
        manual = self._card()
        title_row = ttk.Frame(manual, style='Card.TFrame')
        title_row.pack(fill='x')
        ttk.Label(title_row, text='手动角度', style='Heading.TLabel').pack(side='left')
        ttk.Label(title_row, text='回正：水平90° / 俯仰0° · 输入后发送', style='Muted.TLabel').pack(side='right')
        axes = ttk.Frame(manual, style='Card.TFrame')
        axes.pack(fill='x', pady=(12, 10))
        self.scale_widgets, self.angle_entries, self.nudge_buttons = {}, {}, []
        for col, (axis, title) in enumerate((('pan', '水平轴'), ('tilt', '俯仰轴'))):
            panel = ttk.Frame(axes, style='Axis.TFrame', padding=14)
            panel.grid(row=0, column=col, sticky='nsew', padx=(0, 6) if col == 0 else (6, 0))
            axes.columnconfigure(col, weight=1, uniform='axis')
            ttk.Label(panel, text=title, style='AxisTitle.TLabel').pack(anchor='w')
            ttk.Label(panel, text='正面看：左0° · 正前90° · 右180°' if axis == 'pan' else '向上 −45° · 正前0° · 向下 ＋45°', style='AxisMuted.TLabel').pack(anchor='w', pady=(3, 0))
            inputs = ttk.Frame(panel, style='Axis.TFrame')
            inputs.pack(fill='x', pady=(10, 5))
            minus = ttk.Button(inputs, text='− 1°', width=6, command=lambda a=axis: self.nudge(a, -1))
            minus.pack(side='left')
            self.nudge_buttons.append(minus)
            entry = ttk.Entry(inputs, textvariable=self._targets[axis], justify='center', width=8, font=(FONT, 24, 'bold'))
            entry.pack(side='left', expand=True, fill='x', padx=10)
            entry.bind('<Return>', lambda _e: self.send_angles())
            plus = ttk.Button(inputs, text='＋ 1°', width=6, command=lambda a=axis: self.nudge(a, 1))
            plus.pack(side='left')
            self.nudge_buttons.append(plus)
            self.angle_entries[axis] = entry
            self.scale_widgets[axis] = ttk.Scale(panel, from_=0 if axis == 'pan' else -45,
                to=180 if axis == 'pan' else 45, variable=self._scales[axis], command=lambda v, a=axis: self._scale_changed(a, v))
            self.scale_widgets[axis].pack(fill='x', pady=(5, 4))
            ttk.Label(panel, textvariable=self._ranges[axis], style='AxisMuted.TLabel').pack(anchor='w')
            ttk.Label(panel, textvariable=self._readbacks[axis], style='AxisMuted.TLabel').pack(anchor='w', pady=(9, 0))
        buttons = ttk.Frame(manual, style='Card.TFrame')
        buttons.pack(fill='x')
        self.send_button = ttk.Button(buttons, text='发送角度', style='Primary.TButton', command=self.send_angles)
        self.send_button.pack(side='left')
        self.home_button = ttk.Button(buttons, text='回正 90° / 0°', command=self.home)
        self.home_button.pack(side='left', padx=(8, 0))
        self.hold_button = ttk.Button(buttons, text='停止移动 / 保持', style='Stop.TButton', command=lambda: self._action(self.controller.hold))
        self.hold_button.pack(side='left', padx=8)
        self.enable_button = ttk.Button(buttons, text='启用并回正', command=self.toggle_output)
        self.enable_button.pack(side='right')
        ttk.Label(manual, text='启用输出会回到照片姿态：水平90°、俯仰0°。显示为指令角度，尚未测量实际角度；关闭 PWM 不等于断开5V。',
                  style='Muted.TLabel', font=(FONT, 9), wraplength=960).pack(anchor='w', pady=(10, 0))
        limits_card = self._card(padding=(18, 7))
        limits_header = ttk.Frame(limits_card, style='Card.TFrame')
        limits_header.pack(fill='x')
        self.limits_toggle = ttk.Button(limits_header, text='▸ 角度限位与方向', style='Link.TButton', command=self.toggle_limits)
        self.limits_toggle.pack(side='left')
        self.range_button = ttk.Button(limits_header, text='应用水平180° / 俯仰±45°范围', command=self.apply_range)
        self.range_button.pack(side='right')
        ttk.Label(limits_card, textvariable=self.profile_var, style='Muted.TLabel', wraplength=920).pack(anchor='w', pady=(3, 5))
        self.limits_panel = ttk.Frame(limits_card, style='Card.TFrame', padding=(0, 4, 0, 8))
        self.limit_entries = []
        for col, (key, title) in enumerate((('pan_min', '水平最小'), ('pan_max', '水平最大'), ('tilt_min', '俯仰最小'), ('tilt_max', '俯仰最大'))):
            ttk.Label(self.limits_panel, text=title).grid(row=0, column=col, sticky='w')
            entry = ttk.Entry(self.limits_panel, textvariable=self._limit_vars[key], width=11, font=(FONT, 11))
            entry.grid(row=1, column=col, sticky='ew', padx=(0, 12), pady=(4, 8))
            self.limit_entries.append(entry)
            self.limits_panel.columnconfigure(col, weight=1)
        self.limits_save = ttk.Button(self.limits_panel, text='保存到板端', command=self.save_limits)
        self.limits_save.grid(row=1, column=4, sticky='e', pady=(0, 4))
        ttk.Label(self.limits_panel, text='仅关闭 PWM 后可修改。水平0～180°须包含90°；俯仰−45～+45°须包含0°。断电后需重新应用范围，机械端点尚未测量。',
                  style='Muted.TLabel', font=(FONT, 9), wraplength=900).grid(row=2, column=0, columnspan=5, sticky='w')
        self.direction_checks = []
        for col, (axis, title) in enumerate((('pan', '水平反向'), ('tilt', '俯仰反向'))):
            widget = ttk.Checkbutton(self.limits_panel, text=title, variable=self._direction_vars[axis], command=self._direction_changed)
            widget.grid(row=3, column=col, sticky='w', pady=(12, 4))
            self.direction_checks.append(widget)
        self.direction_save = ttk.Button(self.limits_panel, text='保存方向（不移动）', command=self.save_directions)
        self.direction_save.grid(row=3, column=4, sticky='e', pady=(12, 4))
        ttk.Label(self.limits_panel, text='本次装配两项均不勾选。方向保存到本机；更改后需重新应用所需范围。', style='Muted.TLabel', wraplength=900).grid(row=4, column=0, columnspan=5, sticky='w')
        tracking = self._card(padding=(18, 12))
        tracking_controls = ttk.Frame(tracking, style='Card.TFrame')
        tracking_controls.pack(side='right', padx=(14, 0))
        self.auto_button = ttk.Button(tracking_controls, text='红球跟随 · 等待连接', command=self.toggle_auto, state='disabled')
        self.auto_button.pack(fill='x')
        self.tracking_hold_button = ttk.Button(tracking_controls, text='停止跟随 / 保持', style='Stop.TButton',
            command=lambda: self._action(self.controller.hold), state='disabled')
        self.tracking_hold_button.pack(fill='x', pady=(6, 0))
        ttk.Label(tracking, text='摄像头与云台联动', style='Heading.TLabel').pack(anchor='w')
        ttk.Label(tracking, textvariable=self.auto_hint_var,
                  style='Muted.TLabel', font=(FONT, 9), wraplength=720).pack(anchor='w', pady=(6, 0))
        foot = self._card(padding=(18, 9), pady=(0, 0))

        self.log_toggle = ttk.Button(foot, text='▸ 操作记录', style='Link.TButton', command=self.toggle_log)
        self.log_toggle.pack(anchor='w', pady=(3, 0))
        self.log_widget = tk.Text(foot, height=5, wrap='word', bg='#f3f6fb', fg=MUTED, relief='flat', font=(FONT, 9), padx=8, pady=6, state='disabled')

    def _card(self, pady=(0, 12), padding=(18, 15)):
        frame = ttk.Frame(self.body, style='Card.TFrame', padding=padding)
        frame.pack(fill='x', pady=pady)
        return frame

    def _mousewheel(self, event):
        if self.canvas.winfo_height() < self.body.winfo_reqheight():
            self.canvas.yview_scroll(int(-event.delta / 120), 'units')

    def _demo_changed(self):
        self._ui_error = ''

    def _selected_port(self):
        index = self.port_box.current()
        return self._ports[index] if 0 <= index < len(self._ports) else None

    def _port_hint(self):
        if self.demo_var.get():
            return '演示模式不需要串口，不控制真实舵机。'
        selected = self._selected_port()
        if selected:
            if selected.board_candidate:
                return f'已选 {selected.device} · USB 调试器候选；点击连接后才会确认开发板固件。'
            if selected.bluetooth:
                return f'{selected.device} 是蓝牙串口，不是本项目的 USB-JTAG；请接好开发板后点击“刷新”。'
            return f'已选 {selected.device} · 其他串口；仅在确认接入兼容设备时连接，连接后核验固件。'
        candidates = [p for p in self._ports if p.board_candidate]
        if len(candidates) > 1:
            return '发现多个 USB 调试器候选，请在下拉框中选择开发板对应端口；连接后才会核验固件。'
        if candidates:
            return '可在下拉框选择 USB 调试器候选；连接后才会核验固件。'
        if self._ports and all(p.bluetooth for p in self._ports):
            return '未检测到开发板 USB 调试器，当前只有蓝牙串口。请接好 USB-JTAG 后点击“刷新”。'
        if self._ports:
            return '未识别到开发板 USB 调试器。请接好 USB-JTAG 后刷新；也可手动选择其他兼容串口。'
        return '未发现串口。请用数据线连接开发板 USB-JTAG，然后点击“刷新”。'

    def refresh_ports(self):
        # Do not replace the selection or enumerate devices during a live session.
        state = self.controller.snapshot()
        if state.get('connected') or state.get('connecting'):
            return
        try:
            selected = self._selected_port()
            previous_device = selected.device if selected else None
            self._ports = self.controller.ports()
            self.port_box.configure(values=[p.label for p in self._ports])
            self.port_var.set('')
            remaining = next((i for i, p in enumerate(self._ports) if p.device == previous_device), None)
            candidates = [i for i, p in enumerate(self._ports) if p.board_candidate]
            if remaining is not None:
                self.port_box.current(remaining)
            elif previous_device is None and len(candidates) == 1:
                self.port_box.current(candidates[0])
            self._ui_error = ''
            # If a previously selected device disappeared, leave selection empty.
            self.detail_var.set(self._port_hint())
        except Exception as exc:
            self._error(f'刷新串口失败：{exc}')

    def toggle_connection(self):
        if self._snapshot.get('connected') or self._snapshot.get('connecting'):
            self._action(self.controller.disconnect)
            return
        index = self.port_box.current()
        if not self.demo_var.get() and not 0 <= index < len(self._ports):
            self._error('尚未选择开发板串口。请接好 USB-JTAG 后刷新并选择端口；蓝牙串口不能代替开发板 USB 连接。')
            return
        port = self._ports[index].device if 0 <= index < len(self._ports) else ''
        self._action(lambda: self.controller.connect(port=port, demo=self.demo_var.get()))

    def _entry_changed(self, axis):
        if self._updating:
            return
        self._dirty[axis] = True
        try:
            value = float(self._targets[axis].get())
            low, high = self._axis_limits(axis)
            if math.isfinite(value) and low <= value <= high:
                self._scales[axis].set(value)
        except ValueError:
            pass

    def _scale_changed(self, axis, value):
        if not self._updating:
            self._targets[axis].set(number(float(value)))

    def _limit_changed(self, *_args):
        if not self._updating:
            self._limits_dirty = True

    def _axis_limits(self, axis):
        default = (0.0, 180.0) if axis == 'pan' else (-45.0, 45.0)
        if not self._snapshot.get('connected'):
            return default
        return float(self._snapshot.get(axis + '_input_min', default[0])), float(self._snapshot.get(axis + '_input_max', default[1]))

    def nudge(self, axis, amount):
        try:
            value = float(self._targets[axis].get())
            if not math.isfinite(value):
                raise ValueError
            low, high = self._axis_limits(axis)
            self._targets[axis].set(number(max(low, min(high, value + amount))))
        except ValueError:
            self._error('请输入有效的角度数值。')

    def send_angles(self):
        if not self._ready_for_motion():
            self._error('请先连接设备并启用 PWM 输出，再发送角度。')
            return
        try:
            values = {}
            for axis, label in (('pan', '水平'), ('tilt', '俯仰')):
                value = float(self._targets[axis].get())
                low, high = self._axis_limits(axis)
                if not math.isfinite(value) or not low <= value <= high:
                    raise ValueError(f'{label}目标需要在 {number(low)}°～{number(high)}° 之间。')
                values[axis] = value
            self._action(lambda: self.controller.set_angles(values['pan'], values['tilt']))
        except ValueError as exc:
            self._error(str(exc) if '目标' in str(exc) else '请输入有效的角度数值。')

    def home(self):
        if self._action(self.controller.home):
            self._home_sync = True

    def toggle_output(self):
        turning_on = not self._snapshot.get('enabled', False)
        if turning_on and self._directions_dirty:
            self._error('方向有未保存的修改，请先保存方向或恢复复选框。')
            return
        if self._action(lambda: self.controller.enable(turning_on)) and turning_on:
            self._home_sync = True

    def toggle_auto(self):
        self._action(lambda: self.controller.set_mode('manual' if self._snapshot.get('mode') == 'auto' else 'auto'))

    def _direction_changed(self):
        self._directions_dirty = any(self._direction_vars[a].get() != self.controller.snapshot()[a + '_reversed'] for a in ('pan', 'tilt'))

    def save_directions(self):
        if self._action(lambda: self.controller.set_directions(self._direction_vars['pan'].get(), self._direction_vars['tilt'].get())):
            self._directions_dirty = False
            self._limits_dirty = False
            self._dirty = {'pan': False, 'tilt': False}
            self._home_sync = True

    def apply_range(self):
        if self._directions_dirty:
            self._error('请先保存方向，再应用范围。')
            return
        if self._action(self.controller.apply_mount_range):
            self._limits_dirty = False

    def save_limits(self):
        if self._directions_dirty:
            self._error('请先保存方向，再保存限位。')
            return
        if not self._snapshot.get('connected') or self._snapshot.get('enabled'):
            self._error('请连接设备并关闭 PWM 输出后再修改限位。')
            return
        try:
            values = [float(self._limit_vars[k].get()) for k in ('pan_min', 'pan_max', 'tilt_min', 'tilt_max')]
            bounds = ((0, 180), (0, 180), (-45, 45), (-45, 45))
            if any(not math.isfinite(v) or not low <= v <= high for v, (low, high) in zip(values, bounds)):
                raise ValueError('水平限位角度需在0～180°，俯仰限位角度需在−45～+45°。')
            if values[0] >= values[1] or values[2] >= values[3]:
                raise ValueError('每个轴的最小角度必须小于最大角度。')
            if self._action(lambda: self.controller.set_limits(*values)):
                self._limits_dirty = False
        except ValueError as exc:
            self._error(str(exc) if '角度' in str(exc) else '请填写四个有效的角度数值。')

    def _action(self, operation):
        try:
            operation()
            self._ui_error = ''
            return True
        except Exception as exc:
            self._error(str(exc))
            return False

    def _error(self, message):
        self._ui_error = message
        self.message_var.set(message)
        self.message_label.configure(foreground='#b42332')
        self._log(message)

    def _log(self, message):
        self.log_widget.configure(state='normal')
        self.log_widget.insert('end', message + '\n')
        if int(self.log_widget.index('end-1c').split('.')[0]) > 250:
            self.log_widget.delete('1.0', '51.0')
        self.log_widget.see('end')
        self.log_widget.configure(state='disabled')

    def toggle_log(self):
        self._log_open = not self._log_open
        self.log_toggle.configure(text=('▾' if self._log_open else '▸') + ' 操作记录')
        self.log_widget.pack(fill='x', pady=(4, 2)) if self._log_open else self.log_widget.pack_forget()

    def toggle_limits(self):
        self._limits_open = not self._limits_open
        self.limits_toggle.configure(text=('▾' if self._limits_open else '▸') + ' 角度限位与方向')
        self.limits_panel.pack(fill='x') if self._limits_open else self.limits_panel.pack_forget()

    def _ready_for_motion(self):
        s = self._snapshot
        return s.get('connected', False) and s.get('enabled', False) and not s.get('pending', False) and s.get('mode', 'manual') == 'manual'

    @staticmethod
    def _state(widget, enabled):
        widget.configure(state='normal' if enabled else 'disabled')

    def _poll(self):
        if self.closed:
            return
        try:
            s = self.controller.snapshot()
            mode_changed = s.get('mode') != self._snapshot.get('mode', 'manual')
            self._snapshot = s
            for event in self.controller.poll_events():
                self._log(str(event))
            connected, connecting, enabled, demo, pending = (bool(s.get(k)) for k in ('connected', 'connecting', 'enabled', 'demo', 'pending'))
            active_auto = connected and s.get('mode') == 'auto'
            just_connected = connected and not self._was_connected
            home_done = self._home_sync and not pending
            self._updating = True
            try:
                for axis in ('pan', 'tilt'):
                    low, high = self._axis_limits(axis)
                    self.scale_widgets[axis].configure(from_=low, to=high)
                    raw_low, raw_high = s.get(axis + '_min', low), s.get(axis + '_max', high)
                    self._ranges[axis].set(f'可输入 {number(low)}° ～ {number(high)}°' + (f' · 板端 {number(raw_low)}° ～ {number(raw_high)}°' if connected else ' · 等待读取板端限位'))
                    if connected:
                        actual, target = float(s.get(axis + '_angle', 90 if axis == 'pan' else 0)), float(s.get('target_' + axis, 90 if axis == 'pan' else 0))
                        self._readbacks[axis].set(f'当前指令 {number(actual)}°    板端目标 {number(target)}°')
                        if just_connected or home_done or mode_changed or s.get('mode') == 'auto' or not self._dirty[axis]:
                            self._targets[axis].set(number(target))
                            self._scales[axis].set(max(low, min(high, target)))
                            self._dirty[axis] = False
                    else:
                        self._readbacks[axis].set('当前指令 —    板端目标 —')
                if connected and (just_connected or not self._limits_dirty):
                    for key, var in self._limit_vars.items():
                        axis, bound = key.split('_')
                        var.set(number(float(s[axis + '_input_' + bound])))
                    self._limits_dirty = False
                if not self._directions_dirty:
                    for axis, var in self._direction_vars.items():
                        var.set(bool(s.get(axis + '_reversed', False)))
            finally:
                self._updating = False
            if home_done:
                self._home_sync = False
            self._was_connected = connected
            self.connect_button.configure(text='取消连接' if connecting else '断开连接' if connected else '连接设备')
            self.port_box.configure(state='disabled' if connected or connecting or self.demo_var.get() else 'readonly')
            self._state(self.demo_check, not connected and not connecting)
            self._state(self.refresh_button, not connected and not connecting)
            ready = self._ready_for_motion()
            self._state(self.send_button, ready)
            self._state(self.home_button, ready)
            for widget in list(self.angle_entries.values()) + list(self.scale_widgets.values()) + self.nudge_buttons:
                self._state(widget, s.get('mode') != 'auto' or not connected)
            self.hold_button.configure(text='停止跟随 / 保持' if active_auto else '停止移动 / 保持')
            self._state(self.hold_button, connected and enabled)
            self._state(self.tracking_hold_button, connected and enabled)
            self._state(self.enable_button, connected and (enabled or not pending))
            self.enable_button.configure(text='关闭 PWM 输出' if enabled else '启用并回正')
            for widget in self.limit_entries + [self.limits_save, self.range_button]:
                self._state(widget, connected and not enabled and not pending)
            for widget in self.direction_checks + [self.direction_save]:
                self._state(widget, not connecting and not enabled and not pending)
            self.profile_var.set('当前板端已应用：水平0～180°、俯仰−45～+45°；机械端点尚未测量。' if s.get('profile_ready') else
                '板端当前采用自定义或保守范围。关闭PWM后，可点击右上按钮应用180° / ±45°范围。' if connected else
                '连接后读取范围；应用范围不会启用或移动舵机。')
            supports_auto = bool(s.get('supports_auto'))
            self._state(self.auto_button, connected and enabled and supports_auto and not pending)
            self.auto_button.configure(text=('切回手动并保持' if s.get('mode') == 'auto' else '开始红球跟随') if connected and supports_auto else
                '当前固件仅支持手动' if connected else '红球跟随 · 等待连接')
            self.auto_hint_var.set('通信超时，自动跟随已退出；恢复通信不会自动重启，请重新确认后手动启动。' if connected and s.get('watchdog') else
                '等待开发板确认；确认前请勿视为跟随已启动。右侧“停止跟随 / 保持”仍可停止。' if connected and enabled and pending and not active_auto else
                '演示设备只模拟模式切换，不模拟摄像头识别。' if connected and demo and supports_auto else
                '自动跟随已开启。观察 HDMI 的绿框与 CENTER / HOLD / LOST；点击右侧“停止跟随 / 保持”可退出跟随。' if active_auto else
                '画面在 HDMI 显示器。启用输出后，点击右侧按钮开始跟随；自动控制沿用当前板端限位。' if connected and supports_auto else
                '当前为独立舵机固件；下载摄像头联动固件后可切换自动跟随。' if connected else
                '未连接，自动跟随状态无法确认；重新连接后检查并重新启动。' if s.get('error') else
                '未连接，自动跟随状态未知；连接后读取固件能力，连接本身不会启用或开始跟随。')
            if connected:
                self.badge.configure(text='演示模式 · 不控制硬件' if demo else '真实串口 · 设备已连接', bg='#90601a' if demo else '#146a63')
                self.status_var.set('模拟设备已连接' if demo else f'{s.get("port", "")} 已连接')
                self.output_var.set('PWM 输出已启用' if enabled else 'PWM 输出关闭')
                self.motion_var.set('通信超时 · 跟随已退出' if s.get('watchdog') else '等待设备确认' if pending else '指令缓动中' if s.get('moving') else '自动模式 · 观察 HDMI 锁定状态' if active_auto else '保持当前指令' if enabled else '待启用')
                age = s.get('last_reply_age')
                age_text = f' · 最近反馈 {age:.1f} 秒前' if isinstance(age, (int, float)) else ''
                self.detail_var.set(f'固件 {s.get("firmware") or "—"} · {"自动跟随" if s.get("mode") == "auto" else "手动模式"}{age_text}')
            elif connecting:
                self.badge.configure(text='正在连接…', bg='#334863')
                self.status_var.set('正在握手…')
                self.output_var.set('PWM 状态待确认')
                self.motion_var.set('等待连接')
                self.detail_var.set('正在确认设备与固件能力…')
            else:
                self.badge.configure(text='未连接设备', bg='#334863')
                self.status_var.set('未连接')
                self.output_var.set('PWM 状态未知')
                self.motion_var.set('无设备反馈')
                self.detail_var.set(self._port_hint())
            error = self._ui_error or str(s.get('error') or '')
            if error:
                self.message_var.set(error)
                self.message_label.configure(foreground='#b42332')
            elif connected and s.get('watchdog'):
                self.message_var.set('通信超时：移动已冻结并退出自动跟随，PWM 继续保持；恢复后可重新选择控制模式。保持不等于断电。')
                self.message_label.configure(foreground='#875312')
            else:
                self.message_var.set('演示模式：所有角度与运动均为模拟，未控制真实舵机。' if connected and demo else
                    '正在自动跟随红球；切回手动会保持当前位置，再按需要发送角度。' if connected and s.get('mode') == 'auto' else
                    '编辑角度不会立即运动；确认目标后点击“发送角度”。' if connected and enabled else
                    '输出已关闭。点击“启用并回正”后，将回到水平90°、俯仰0°。' if connected else
                    '未连接时无法确认输出状态；断线可能继续保持 PWM，不等于断电。摄像头画面在外接 HDMI 显示器。')
                self.message_label.configure(foreground=MUTED)
        except Exception as exc:
            self._error(f'界面状态更新失败：{exc}')
        self.root.after(100, self._poll)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.controller.close()
        finally:
            self.root.destroy()

def main():
    parser = argparse.ArgumentParser(description='基于FPGA的云台目标追踪系统 · 桌面舵机控制台')
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument('--demo', action='store_true', help='连接模拟设备；不连接真实串口，不自动启用输出')
    connection.add_argument('--port', help='启动后只读连接指定串口（如COM12），不自动启用或应用范围')
    parser.add_argument('--quit-after', type=float, default=0, metavar='SECONDS', help='测试用：指定秒数后关闭窗口')
    parser.add_argument('--screenshot', type=Path, help='测试用：窗口稳定后保存截图（需要 Pillow）')
    args = parser.parse_args()
    root = tk.Tk()
    app = ServoApp(root)
    if args.demo:
        app.demo_var.set(True)
        root.after(200, app.toggle_connection)
    elif args.port:
        root.after(200, lambda: app._action(lambda: app.controller.connect(port=args.port)))
    if args.screenshot:
        def screenshot():
            try:
                from PIL import ImageGrab
                root.update_idletasks()
                args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                if sys.platform == 'win32':
                    # Capture the native top-level window, without DPI-sensitive
                    # screen coordinates or unrelated desktop content.
                    import ctypes
                    user32 = ctypes.windll.user32
                    user32.GetParent.argtypes = [ctypes.c_void_p]
                    user32.GetParent.restype = ctypes.c_void_p
                    hwnd = user32.GetParent(root.winfo_id()) or root.winfo_id()
                    capture = ImageGrab.grab(window=hwnd)
                else:
                    x, y = root.winfo_rootx(), root.winfo_rooty()
                    capture = ImageGrab.grab(bbox=(x, y, x + root.winfo_width(), y + root.winfo_height()))
                capture.save(args.screenshot)
                print(f'Screenshot saved: {args.screenshot}', flush=True)
            except Exception as exc:
                app._error(f'截图失败：{exc}')
        root.after(1800, screenshot)
    if args.quit_after > 0:
        root.after(max(100, int(args.quit_after * 1000)), app.close)
    root.mainloop()

if __name__ == '__main__':
    main()
