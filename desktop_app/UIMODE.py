import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import os
import zlib
from pathlib import Path
import json
import subprocess
import sys
import shutil
import glob

class BMSLoggerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Data Logger")
        self.root.geometry("1500x900")
        self.root.configure(bg='#0a0a0a')
        
        # Set style
        self.setup_styles()
        
        # Serial connection
        self.serial_conn = None
        self.listener_running = False
        self.heartbeat_timer = None
        
        # File handling
        self.file_list = []
        self.diag_list = []
        self.session_history_list = []
        self.in_file_list = False
        self.in_diag_list = False
        self.in_session_history = False
        self.receiving_file = False
        self.receiving_binary = False
        self.receiving_compressed = False
        self.receiving_rle = False
        self.binary_size = 0
        self.binary_received = 0
        self.binary_data = bytearray()
        self.compressed_size = 0
        self.original_size = 0
        self.file_content = []
        self.current_filename = ""
        self.in_card_info = False
        self.card_info = []
        self.download_start_time = 0
        
        # Data markers
        self.in_live_data = False
        self.in_stats = False
        self.in_log_status = False
        self.in_session_summary = False
        
        # Session tracking
        self.current_session = 0
        self.current_file = 0
        self.session_records = 0
        self.file_records = 0
        self.ecu_state = 0
        self.ecu_states = ['Unknown', 'Connected', 'Disconnected', 'Degraded', 'Silent']
        self.rotate_reasons = ['None', 'Hourly', 'Size', 'ECU Disconnect', 'User Command', 'System', 'Recovery']
        
        # Date filtering for files tab
        self.available_dates = []
        self.current_filter_date = None
        self.current_year = None
        self.current_month = None
        self.current_day = None
        
        # Session History filtering
        self.session_year = None
        self.session_month = None
        self.all_sessions = []
        
        # Password visibility flags
        self.show_wifi_password = False
        self.show_ap_password = False
        self.show_mqtt_password = False
        
        # Configuration storage
        self.config = self.load_config()
        
        # Setup UI
        self.setup_ui()
        self.scan_ports()
    
    def load_config(self):
        """Load saved configuration from file"""
        config_file = Path("logger_config.json")
        default_config = {
            "logging": {
                "interval_ms": 100,
                "max_file_size_mb": 100,
                "rotate_hourly": False,
                "auto_delete_days": 30,
                "include_date_in_filename": True,
                "print_every_message": False,
                "print_logged_data": False
            },
            "can": {
                "filter_mode": 1,
                "baud_rate": 500,
                "rx_queue_len": 100
            },
            "gps": {
                "baud_rate": 38400,
                "update_interval_ms": 100
            },
            "wifi": {
                "ssid": "Moonrider_02",
                "password": "Moonrider@567",
                "ap_ssid": "BMS_Logger",
                "ap_password": "12345678"
            },
            "mqtt": {
                "broker": "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud",
                "port": 8883,
                "topic": "tractor/data",
                "client_id": "ESP32_Tractor_Logger",
                "username": "MR_TRACTOR",
                "password": "#Lokesh000"
            },
            "i2c": {
                "update_interval_ms": 1000
            },
            "system": {
                "serial_debug": False,
                "buffer_size": 16384,
                "flush_interval": 2000,
                "ecu_disconnect_timeout": 30000
            }
        }
        
        if config_file.exists():
            try:
                with open(config_file, 'r') as f:
                    saved_config = json.load(f)
                    for category in default_config:
                        if category not in saved_config:
                            saved_config[category] = default_config[category]
                        else:
                            for key in default_config[category]:
                                if key not in saved_config[category]:
                                    saved_config[category][key] = default_config[category][key]
                    return saved_config
            except Exception as e:
                print(f"Error loading config: {e}")
                return default_config
        return default_config
    
    def save_config(self):
        """Save current configuration to file"""
        config_file = Path("logger_config.json")
        try:
            with open(config_file, 'w') as f:
                json.dump(self.config, f, indent=4)
            self.log("✅ Configuration saved", "success")
            self.config_status.config(text="Configuration saved", foreground="#10b981")
            self.root.after(3000, lambda: self.config_status.config(text=""))
            return True
        except Exception as e:
            self.log(f"❌ Failed to save config: {e}", "error")
            return False
    
    def send_config_to_device(self):
        """Send configuration commands to the device"""
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Warning", "Not connected to device")
            return
        
        commands = []
        
        commands.append(f"config logging interval {self.config['logging']['interval_ms']}")
        commands.append(f"config logging maxsize {self.config['logging']['max_file_size_mb']}")
        commands.append(f"config logging rotatehourly {int(self.config['logging']['rotate_hourly'])}")
        commands.append(f"config logging autodelete {self.config['logging']['auto_delete_days']}")
        
        commands.append(f"config can filtermode {self.config['can']['filter_mode']}")
        commands.append(f"config can baud {self.config['can']['baud_rate']}")
        commands.append(f"config can rxqueue {self.config['can']['rx_queue_len']}")
        
        commands.append(f"config gps baud {self.config['gps']['baud_rate']}")
        commands.append(f"config gps interval {self.config['gps']['update_interval_ms']}")
        
        commands.append(f"config wifi ssid {self.config['wifi']['ssid']}")
        commands.append(f"config wifi password {self.config['wifi']['password']}")
        commands.append(f"config wifi apssid {self.config['wifi']['ap_ssid']}")
        commands.append(f"config wifi appassword {self.config['wifi']['ap_password']}")
        
        commands.append(f"config mqtt broker {self.config['mqtt']['broker']}")
        commands.append(f"config mqtt port {self.config['mqtt']['port']}")
        commands.append(f"config mqtt topic {self.config['mqtt']['topic']}")
        commands.append(f"config mqtt clientid {self.config['mqtt']['client_id']}")
        commands.append(f"config mqtt username {self.config['mqtt']['username']}")
        commands.append(f"config mqtt password {self.config['mqtt']['password']}")
        
        commands.append(f"config i2c interval {self.config['i2c']['update_interval_ms']}")
        
        commands.append(f"config system debug {int(self.config['system']['serial_debug'])}")
        commands.append(f"config system buffersize {self.config['system']['buffer_size']}")
        commands.append(f"config system flushinterval {self.config['system']['flush_interval']}")
        commands.append(f"config system ecutimeout {self.config['system']['ecu_disconnect_timeout']}")
        
        sent_count = 0
        for cmd in commands:
            try:
                self.serial_conn.write((cmd + '\n').encode())
                sent_count += 1
                time.sleep(0.05)
            except Exception as e:
                self.log(f"Error sending command: {cmd} - {e}", "error")
        
        self.log(f"📡 Sent {sent_count} configuration commands to device", "info")
        self.config_status.config(text=f"Sent {sent_count} commands", foreground="#10b981")
        self.root.after(3000, lambda: self.config_status.config(text=""))
        messagebox.showinfo("Success", f"Sent {sent_count} configuration commands to device.\nPlease check the console for responses.")
    
    def setup_styles(self):
        """Configure custom styles for professional modern look with denim blue theme"""
        style = ttk.Style()
        style.theme_use('clam')
        
        bg_color = '#0a0a0a'
        fg_color = '#e0e0e0'
        
        denim_primary = '#2c5f8a'
        denim_light = '#3a7ca5'
        denim_dark = '#1f4662'
        denim_deep = '#143240'
        denim_accent = '#4681a9'
        denim_hover = '#5a9ac2'
        
        success_color = '#10b981'
        warning_color = '#f59e0b'
        error_color = '#ef4444'
        surface_color = '#1a1a1a'
        border_color = '#2a2a2a'
        
        light_download = denim_primary
        light_download_hover = denim_light
        light_delete = denim_dark
        light_delete_hover = denim_deep
        
        style.configure('TFrame', background=bg_color)
        style.configure('TLabel', background=bg_color, foreground=fg_color, font=('Segoe UI', 9))
        style.configure('TLabelframe', background=bg_color, foreground=fg_color, 
                        font=('Segoe UI', 9, 'bold'), bordercolor=border_color)
        style.configure('TLabelframe.Label', background=bg_color, foreground=denim_accent,
                        font=('Segoe UI', 9, 'bold'))
        
        style.configure('TButton', background=surface_color, foreground=fg_color,
                        borderwidth=1, focusthickness=3, focuscolor='none',
                        font=('Segoe UI', 8), padding=(6, 3))
        style.map('TButton', 
                  background=[('active', denim_hover), ('pressed', denim_primary)],
                  foreground=[('active', '#ffffff'), ('pressed', '#ffffff')])
        
        style.configure('Success.TButton', background=success_color, foreground='#ffffff')
        style.map('Success.TButton', 
                  background=[('active', '#059669'), ('pressed', '#047857')])
        
        style.configure('Danger.TButton', background=error_color, foreground='#ffffff')
        style.map('Danger.TButton', 
                  background=[('active', '#dc2626'), ('pressed', '#b91c1c')])
        
        style.configure('Download.TButton', background=light_download, foreground='#ffffff')
        style.map('Download.TButton', 
                  background=[('active', light_download_hover), ('pressed', denim_dark)])
        
        style.configure('Delete.TButton', background=light_delete, foreground='#ffffff')
        style.map('Delete.TButton', 
                  background=[('active', light_delete_hover), ('pressed', denim_deep)])
        
        style.configure('Primary.TButton', background=denim_accent, foreground='#ffffff')
        style.map('Primary.TButton', 
                  background=[('active', denim_hover), ('pressed', denim_primary)])
        
        style.configure('TCombobox', fieldbackground=surface_color, background=surface_color,
                        foreground=fg_color, arrowcolor=fg_color, borderwidth=1)
        style.map('TCombobox', fieldbackground=[('readonly', surface_color)],
                  selectbackground=[('readonly', denim_accent)])
        
        style.configure('TNotebook', background=bg_color, borderwidth=0)
        style.configure('TNotebook.Tab', background=surface_color, foreground=fg_color,
                        padding=[12, 6], font=('Segoe UI', 9, 'bold'))
        style.map('TNotebook.Tab', 
                  background=[('selected', denim_accent)],
                  foreground=[('selected', '#ffffff')])
        
        style.configure('TProgressbar', background=denim_accent, troughcolor=surface_color,
                        borderwidth=0, lightcolor=denim_accent, darkcolor=denim_accent)
        
        style.configure('TScrollbar', background=surface_color, troughcolor=bg_color,
                        arrowcolor=fg_color, borderwidth=0)
        style.map('TScrollbar', background=[('active', denim_accent)])
        
        style.configure('TSeparator', background=border_color)
        
        # Configure Treeview for dark theme
        style.configure('Treeview', background=surface_color, foreground=fg_color,
                        fieldbackground=surface_color, borderwidth=0, font=('Segoe UI', 8))
        style.map('Treeview', background=[('selected', denim_accent)],
                  foreground=[('selected', '#ffffff')])
        
        # Configure Heading
        style.configure('Treeview.Heading', background=denim_dark, foreground=fg_color,
                        font=('Segoe UI', 9, 'bold'), borderwidth=1)
        style.map('Treeview.Heading', background=[('active', denim_accent)])
    
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill="both", expand=True)
        
        toolbar = ttk.Frame(main_frame)
        toolbar.pack(fill="x", pady=(0, 8))
        
        title_label = ttk.Label(toolbar, text="DATA Logger",
                                font=('Segoe UI', 12, 'bold'), foreground='#4681a9')
        title_label.pack(side="left", padx=(0, 12))
        
        ttk.Separator(toolbar, orient='vertical').pack(side="left", fill='y', padx=5)
        
        ttk.Label(toolbar, text="Port:", font=('Segoe UI', 8)).pack(side="left")
        self.port_combo = ttk.Combobox(toolbar, width=8, font=('Segoe UI', 8))
        self.port_combo.pack(side="left", padx=2)
        
        ttk.Label(toolbar, text="Baud:", font=('Segoe UI', 8)).pack(side="left", padx=(5, 0))
        self.baud_combo = ttk.Combobox(toolbar, width=7, 
                                       values=['115200', '230400', '460800', '921600'],
                                       font=('Segoe UI', 8))
        self.baud_combo.set('921600')
        self.baud_combo.pack(side="left", padx=2)
        
        self.connect_btn = ttk.Button(toolbar, text="Connect", 
                                     command=self.toggle_connection, width=7)
        self.connect_btn.pack(side="left", padx=2)
        
        ttk.Button(toolbar, text="Scan", command=self.scan_ports,
                  width=5).pack(side="left", padx=2)
        
        ttk.Separator(toolbar, orient='vertical').pack(side="left", fill='y', padx=8)
        
        self.conn_status = ttk.Label(toolbar, text="🔴 Disconnected", foreground="#ef4444",
                                     font=('Segoe UI', 8))
        self.conn_status.pack(side="left", padx=4)
        
        self.sd_status = ttk.Label(toolbar, text="💾 Unknown", foreground="#e0e0e0",
                                   font=('Segoe UI', 8))
        self.sd_status.pack(side="left", padx=4)
        
        self.ecu_status = ttk.Label(toolbar, text="🚗 Unknown", foreground="#e0e0e0",
                                    font=('Segoe UI', 8))
        self.ecu_status.pack(side="left", padx=4)
        
        self.session_status = ttk.Label(toolbar, text="📊 S0/F0/R0", foreground="#10b981",
                                        font=('Segoe UI', 8))
        self.session_status.pack(side="left", padx=4)
        
        self.status_indicator = tk.Canvas(toolbar, width=10, height=10, 
                                          bg='#0a0a0a', highlightthickness=0)
        self.status_indicator.pack(side="right", padx=5)
        self.status_indicator.create_oval(1, 1, 9, 9, fill='#ef4444', outline='')
        
        # Toolbar buttons
        file_toolbar = ttk.Frame(main_frame)
        file_toolbar.pack(fill="x", pady=(0, 8))
        
        ttk.Button(file_toolbar, text="🔄 Refresh All", command=self.refresh_all,
                  style='Primary.TButton', width=14).pack(side="left", padx=2)
        ttk.Button(file_toolbar, text="ℹ️ Card Info", command=self.get_card_info,
                  style='Primary.TButton', width=12).pack(side="left", padx=2)
        
        # Date filter for files tab
        date_filter_frame = ttk.LabelFrame(main_frame, text="📅 Date Filter (Files)", padding="5")
        date_filter_frame.pack(fill="x", pady=(0, 8))
        
        filter_row = ttk.Frame(date_filter_frame)
        filter_row.pack()
        
        ttk.Label(filter_row, text="Browse by date:", font=('Segoe UI', 9, 'bold')).pack(side="left", padx=5)
        
        ttk.Label(filter_row, text="Year:").pack(side="left", padx=(10, 2))
        self.year_combo = ttk.Combobox(filter_row, width=8, values=[], state="readonly")
        self.year_combo.pack(side="left", padx=2)
        self.year_combo.bind("<<ComboboxSelected>>", self.on_year_selected)
        
        ttk.Label(filter_row, text="Month:").pack(side="left", padx=(10, 2))
        self.month_combo = ttk.Combobox(filter_row, width=10, values=[], state="readonly")
        self.month_combo.pack(side="left", padx=2)
        self.month_combo.bind("<<ComboboxSelected>>", self.on_month_selected)
        
        ttk.Label(filter_row, text="Day:").pack(side="left", padx=(10, 2))
        self.day_combo = ttk.Combobox(filter_row, width=8, values=[], state="readonly")
        self.day_combo.pack(side="left", padx=2)
        self.day_combo.bind("<<ComboboxSelected>>", self.on_day_selected)
        
        ttk.Button(filter_row, text="Apply Filter", command=self.apply_date_filter,
                  width=12).pack(side="left", padx=(15, 5))
        ttk.Button(filter_row, text="Clear Filter", command=self.clear_date_filter,
                  width=10).pack(side="left", padx=2)
        ttk.Button(filter_row, text="📂 Show All", command=self.show_all_files,
                  width=10).pack(side="left", padx=2)
        
        self.filter_info = ttk.Label(filter_row, text="", foreground="#10b981")
        self.filter_info.pack(side="left", padx=10)
        
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill="both", expand=True, pady=5)
        
        files_tab = ttk.Frame(self.notebook)
        self.notebook.add(files_tab, text="📁 SD Card Files")
        self.setup_files_tab(files_tab)
        
        session_tab = ttk.Frame(self.notebook)
        self.notebook.add(session_tab, text="📊 Session History")
        self.setup_session_tab(session_tab)
        
        config_tab = ttk.Frame(self.notebook)
        self.notebook.add(config_tab, text="⚙️ Configuration")
        self.setup_config_tab(config_tab)
        
        firmware_tab = ttk.Frame(self.notebook)
        self.notebook.add(firmware_tab, text="🔥 Firmware Update")
        self.setup_firmware_tab(firmware_tab)
        
        console_tab = ttk.Frame(self.notebook)
        self.notebook.add(console_tab, text="💻 Console")
        self.setup_console_tab(console_tab)
        
        footer_frame = ttk.Frame(main_frame)
        footer_frame.pack(fill="x", pady=(5, 0))
        
        self.progress_frame = ttk.Frame(footer_frame)
        self.progress_frame.pack(fill="x", pady=(0, 3))
        
        self.progress_label = ttk.Label(self.progress_frame, text="", font=('Segoe UI', 7))
        self.progress_label.pack(side="left", padx=(0, 8))
        
        self.progress_bar = ttk.Progressbar(self.progress_frame, length=300, mode='determinate')
        self.progress_bar.pack(side="left", fill="x", expand=True)
        self.progress_bar.pack_forget()
        
        self.speed_label = ttk.Label(self.progress_frame, text="", font=('Segoe UI', 7))
        self.speed_label.pack(side="right")
        
        self.status_bar = ttk.Label(footer_frame, text="Ready", relief="sunken",
                                    font=('Segoe UI', 8))
        self.status_bar.pack(fill="x", pady=(3, 0))
    
    def setup_session_tab(self, parent):
        """Setup the Session History tab with detailed view"""
        main_container = ttk.Frame(parent)
        main_container.pack(fill="both", expand=True, padx=8, pady=8)
        
        # Toolbar
        toolbar = ttk.Frame(main_container)
        toolbar.pack(fill="x", pady=(0, 10))
        
        ttk.Button(toolbar, text="🔄 Refresh History", command=self.refresh_session_history,
                style='Primary.TButton', width=18).pack(side="left", padx=2)
        ttk.Button(toolbar, text="📊 Summary Stats", command=self.show_session_summary,
                style='Primary.TButton', width=15).pack(side="left", padx=2)
        ttk.Button(toolbar, text="📤 Export CSV", command=self.export_session_history,
                style='Download.TButton', width=15).pack(side="left", padx=2)
        
        # Treeview for sessions - with dark theme
        tree_frame = ttk.LabelFrame(main_container, text="📋 Session Records", padding="5")
        tree_frame.pack(fill="both", expand=True)
        
        # Create scrollbars
        tree_scroll_y = ttk.Scrollbar(tree_frame, orient="vertical")
        tree_scroll_x = ttk.Scrollbar(tree_frame, orient="horizontal")
        
        # Create Treeview with columns
        columns = ('SessionID', 'FileSeq', 'StartTime', 'EndTime', 'Duration', 'FileName', 
                'Records', 'FileRecords', 'ECUState', 'RotateReason', 'FileSize', 'Clean')
        
        self.session_tree = ttk.Treeview(tree_frame, columns=columns, show='headings',
                                        yscrollcommand=tree_scroll_y.set,
                                        xscrollcommand=tree_scroll_x.set,
                                        height=15,
                                        selectmode='browse')
        
        tree_scroll_y.config(command=self.session_tree.yview)
        tree_scroll_x.config(command=self.session_tree.xview)
        
        # Define column headings
        self.session_tree.heading('SessionID', text='Session ID')
        self.session_tree.heading('FileSeq', text='File Seq')
        self.session_tree.heading('StartTime', text='Start Time')
        self.session_tree.heading('EndTime', text='End Time')
        self.session_tree.heading('Duration', text='Duration')
        self.session_tree.heading('FileName', text='File Name')
        self.session_tree.heading('Records', text='Session Recs')
        self.session_tree.heading('FileRecords', text='File Recs')
        self.session_tree.heading('ECUState', text='ECU State')
        self.session_tree.heading('RotateReason', text='Rotate Reason')
        self.session_tree.heading('FileSize', text='File Size')
        self.session_tree.heading('Clean', text='Clean')
        
        # Configure column widths
        self.session_tree.column('SessionID', width=80, anchor='center')
        self.session_tree.column('FileSeq', width=80, anchor='center')
        self.session_tree.column('StartTime', width=150)
        self.session_tree.column('EndTime', width=150)
        self.session_tree.column('Duration', width=100, anchor='center')
        self.session_tree.column('FileName', width=300)
        self.session_tree.column('Records', width=100, anchor='center')
        self.session_tree.column('FileRecords', width=80, anchor='center')
        self.session_tree.column('ECUState', width=100, anchor='center')
        self.session_tree.column('RotateReason', width=120, anchor='center')
        self.session_tree.column('FileSize', width=100, anchor='center')
        self.session_tree.column('Clean', width=60, anchor='center')
        
        # Pack treeview
        self.session_tree.pack(side="left", fill="both", expand=True)
        tree_scroll_y.pack(side="right", fill="y")
        tree_scroll_x.pack(side="bottom", fill="x")
        
        # Double-click to open file
        self.session_tree.bind("<Double-1>", self.on_session_double_click)
        
        # Status label at bottom
        self.session_status_label = ttk.Label(main_container, text="Ready", foreground="#808080")
        self.session_status_label.pack(pady=5)
    
    # Session History Methods
    def refresh_session_history(self):
        """Request session history from device"""
        self.send_command("sessions")
    
    def parse_session_history(self, lines):
        """Parse session history lines from device"""
        sessions = []
        for line in lines:
            if line and not line.startswith("SessionID") and not line.startswith("NO_SESSIONS"):
                parts = line.split(',')
                if len(parts) >= 11:
                    try:
                        # Parse the date from DD-MM-YYYY to YYYY-MM-DD
                        start_parts = parts[2].split(' ')
                        start_date = start_parts[0]
                        start_time = start_parts[1] if len(start_parts) > 1 else "00:00:00"
                        
                        # Convert date from DD-MM-YYYY to YYYY-MM-DD
                        if '-' in start_date:
                            day, month, year = start_date.split('-')
                            start_date_formatted = f"{year}-{month}-{day}"
                        else:
                            start_date_formatted = start_date
                        
                        end_parts = parts[3].split(' ')
                        end_date = end_parts[0]
                        end_time = end_parts[1] if len(end_parts) > 1 else "00:00:00"
                        
                        if '-' in end_date:
                            day, month, year = end_date.split('-')
                            end_date_formatted = f"{year}-{month}-{day}"
                        else:
                            end_date_formatted = end_date
                        
                        start_time_str = f"{start_date_formatted} {start_time}"
                        end_time_str = f"{end_date_formatted} {end_time}"
                        
                        session = {
                            'SessionID': int(parts[0]),
                            'FileSeq': int(parts[1]),
                            'StartTime': start_time_str,
                            'EndTime': end_time_str,
                            'FileName': parts[4],
                            'Records': int(parts[5]),
                            'FileRecords': int(parts[6]),
                            'ECUState': self.ecu_states[int(parts[7])] if len(parts) > 7 and parts[7].isdigit() else 'Unknown',
                            'RotateReason': self.rotate_reasons[int(parts[8])] if len(parts) > 8 and parts[8].isdigit() else 'Unknown',
                            'FileSize': int(parts[9]) if len(parts) > 9 else 0,
                            'Clean': 'Yes' if len(parts) > 10 and parts[10] == '1' else 'No'
                        }
                        
                        # Calculate duration
                        try:
                            start = datetime.strptime(start_time_str, '%Y-%m-%d %H:%M')
                            end = datetime.strptime(end_time_str, '%Y-%m-%d %H:%M')
                            duration = end - start
                            total_seconds = int(duration.total_seconds())
                            hours = total_seconds // 3600
                            minutes = (total_seconds % 3600) // 60
                            seconds = total_seconds % 60
                            session['Duration'] = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
                        except Exception as e:
                            print(f"Duration calculation error: {e}")
                            session['Duration'] = 'N/A'
                        
                        # Format file size
                        size_bytes = session['FileSize']
                        if size_bytes < 1024:
                            session['FileSizeFormatted'] = f"{size_bytes} B"
                        elif size_bytes < 1024 * 1024:
                            session['FileSizeFormatted'] = f"{size_bytes / 1024:.1f} KB"
                        else:
                            session['FileSizeFormatted'] = f"{size_bytes / (1024 * 1024):.2f} MB"
                        
                        sessions.append(session)
                        print(f"Parsed session: ID={session['SessionID']}, File={session['FileName']}")  # Debug
                    except Exception as e:
                        print(f"Error parsing session line: {e}")
                        print(f"Line was: {line}")
                        continue
        return sessions
    
    def update_session_tree(self, sessions):
        """Update the session treeview with parsed data"""
        print(f"update_session_tree called with {len(sessions)} sessions")  # Debug
        
        # Clear existing items
        for item in self.session_tree.get_children():
            self.session_tree.delete(item)
        
        # Store all sessions for filtering
        self.all_sessions = sessions
        
        # Update filter combos (if you have them, otherwise remove)
        if sessions:
            years = sorted(set([s['StartTime'][:4] for s in sessions]), reverse=True)
            if hasattr(self, 'session_year_combo'):
                self.session_year_combo['values'] = years
        
        # Display all sessions
        self.display_sessions(sessions)
    
    def display_sessions(self, sessions):
        """Display sessions in treeview"""
        print(f"display_sessions called with {len(sessions)} sessions")  # Debug
        
        for item in self.session_tree.get_children():
            self.session_tree.delete(item)
        
        if not sessions:
            self.session_status_label.config(text="No sessions found", foreground="#ef4444")
            return
        
        for session in sessions:
            values = (
                session['SessionID'],
                session['FileSeq'],
                session['StartTime'],
                session['EndTime'],
                session['Duration'],
                session['FileName'],
                f"{session['Records']:,}",
                f"{session['FileRecords']:,}",
                session['ECUState'],
                session['RotateReason'],
                session['FileSizeFormatted'],
                session['Clean']
            )
            self.session_tree.insert('', 'end', values=values)
            print(f"Added session: {session['SessionID']} - {session['FileName']}")  # Debug
        
        total_records = sum(s['Records'] for s in sessions)
        total_size = sum(s['FileSize'] for s in sessions)
        total_size_formatted = f"{total_size / (1024 * 1024):.2f} MB" if total_size > 0 else "0 B"
        
        self.session_status_label.config(
            text=f"Showing {len(sessions)} sessions | Total Records: {total_records:,} | Total Size: {total_size_formatted}",
            foreground="#10b981"
        )
        
        print(f"Displayed {len(sessions)} sessions in treeview")  # Debug
    
    def on_session_year_selected(self, event):
        """Handle year selection in session filter"""
        self.session_year = self.session_year_combo.get()
        # Update month combo based on selected year
        months = set()
        for session in self.all_sessions:
            year = session['StartTime'][:4]
            if year == self.session_year:
                month = session['StartTime'][5:7]
                month_name = f"{month} ({self.get_month_name(int(month))})"
                months.add(month_name)
        self.session_month_combo['values'] = sorted(list(months))
        self.session_month_combo.set('')
    
    def on_session_month_selected(self, event):
        """Handle month selection in session filter"""
        month_str = self.session_month_combo.get()
        if month_str:
            self.session_month = month_str.split(' ')[0]
        else:
            self.session_month = None
    
    def apply_session_filter(self):
        """Apply date filter to session list"""
        if not hasattr(self, 'all_sessions') or not self.all_sessions:
            return
        
        year = self.session_year_combo.get() if hasattr(self, 'session_year_combo') else None
        month = self.session_month if hasattr(self, 'session_month') else None
        
        filtered = self.all_sessions
        if year:
            filtered = [s for s in filtered if s['StartTime'].startswith(year)]
        if month:
            filtered = [s for s in filtered if s['StartTime'][5:7] == month]
        
        self.display_sessions(filtered)
        
        if year or month:
            filter_text = f"Filter: {year}"
            if month:
                filter_text += f"-{month}"
            self.session_filter_info.config(text=filter_text)
        else:
            self.session_filter_info.config(text="")
    
    def clear_session_filter(self):
        """Clear session filter and show all"""
        self.session_year_combo.set('')
        self.session_month_combo.set('')
        if hasattr(self, 'session_month'):
            self.session_month = None
        self.display_sessions(self.all_sessions)
        self.session_filter_info.config(text="")
    
    def show_session_summary(self):
        """Show summary statistics of sessions"""
        if not hasattr(self, 'all_sessions') or not self.all_sessions:
            messagebox.showinfo("Info", "No session data available")
            return
        
        total_sessions = len(self.all_sessions)
        total_records = sum(s['Records'] for s in self.all_sessions)
        total_size = sum(s['FileSize'] for s in self.all_sessions)
        
        # Find first and last session
        first_session = min(self.all_sessions, key=lambda x: x['StartTime'])
        last_session = max(self.all_sessions, key=lambda x: x['StartTime'])
        
        # Count by rotate reason
        rotate_reasons = {}
        for session in self.all_sessions:
            reason = session['RotateReason']
            rotate_reasons[reason] = rotate_reasons.get(reason, 0) + 1
        
        # Count by ECU state
        ecu_states = {}
        for session in self.all_sessions:
            state = session['ECUState']
            ecu_states[state] = ecu_states.get(state, 0) + 1
        
        summary = f"""
═══════════════════════════════════════════════════════════
                    SESSION SUMMARY REPORT
═══════════════════════════════════════════════════════════

Total Sessions:      {total_sessions}
Total Records:       {total_records:,}
Total Data Size:     {total_size / (1024 * 1024):.2f} MB

Date Range:
  First Session:     {first_session['StartTime']}
  Last Session:      {last_session['StartTime']}
  Duration:          {first_session['StartTime'][:10]} to {last_session['StartTime'][:10]}

Rotation Reasons:
"""
        for reason, count in sorted(rotate_reasons.items(), key=lambda x: x[1], reverse=True):
            summary += f"  {reason:<20} {count:>6} sessions\n"
        
        summary += "\nECU States:\n"
        for state, count in sorted(ecu_states.items(), key=lambda x: x[1], reverse=True):
            summary += f"  {state:<20} {count:>6} sessions\n"
        
        summary += "\nClean Closure:\n"
        clean_count = sum(1 for s in self.all_sessions if s['Clean'] == 'Yes')
        unclean_count = total_sessions - clean_count
        summary += f"  Clean Close:        {clean_count:>6} sessions\n"
        summary += f"  Unclean Close:      {unclean_count:>6} sessions\n"
        
        summary += "\n═══════════════════════════════════════════════════════════"
        
        messagebox.showinfo("Session Summary", summary)
    
    def export_session_history(self):
        """Export session history to CSV file"""
        if not hasattr(self, 'all_sessions') or not self.all_sessions:
            messagebox.showinfo("Info", "No session data to export")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=f"sessions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    # Write header
                    f.write("SessionID,FileSeq,StartTime,EndTime,FileName,SessionRecords,FileRecords,ECUState,RotateReason,FileSize_Bytes,CleanClosure,Duration\n")
                    
                    # Write data
                    for session in self.all_sessions:
                        f.write(f"{session['SessionID']},{session['FileSeq']},{session['StartTime']},{session['EndTime']},"
                               f"{session['FileName']},{session['Records']},{session['FileRecords']},"
                               f"{session['ECUState']},{session['RotateReason']},{session['FileSize']},"
                               f"{1 if session['Clean'] == 'Yes' else 0},{session['Duration']}\n")
                
                self.log(f"✅ Session history exported to {filename}", "success")
                messagebox.showinfo("Success", f"Exported {len(self.all_sessions)} sessions to {filename}")
            except Exception as e:
                self.log(f"❌ Export failed: {e}", "error")
                messagebox.showerror("Error", f"Export failed: {e}")
    
    def on_session_double_click(self, event):
        """Handle double-click on session to download the file"""
        selection = self.session_tree.selection()
        if selection:
            item = self.session_tree.item(selection[0])
            values = item['values']
            if len(values) >= 6:
                filename = values[5]  # FileName column
                self.log(f"📥 Downloading file from session: {filename}", "info")
                self.send_command(f"send {filename}")
    
    def setup_config_tab(self, parent):
        canvas = tk.Canvas(parent, bg='#0a0a0a', highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        main_container = ttk.Frame(scrollable_frame, padding="10")
        main_container.pack(fill="both", expand=True)
        
        header_frame = ttk.Frame(main_container)
        header_frame.pack(fill="x", pady=(0, 15))
        
        ttk.Label(header_frame, text="⚙️ Data Logger Configuration", 
                font=('Segoe UI', 14, 'bold'), foreground='#4681a9').pack(side="left")
        
        ttk.Separator(main_container, orient='horizontal').pack(fill="x", pady=5)
        
        columns_frame = ttk.Frame(main_container)
        columns_frame.pack(fill="both", expand=True, pady=10)
        
        columns_frame.columnconfigure(0, weight=1)
        columns_frame.columnconfigure(1, weight=1)
        columns_frame.columnconfigure(2, weight=1)
        
        col1 = ttk.Frame(columns_frame)
        col1.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        
        col2 = ttk.Frame(columns_frame)
        col2.grid(row=0, column=1, sticky="nsew", padx=5)
        
        col3 = ttk.Frame(columns_frame)
        col3.grid(row=0, column=2, sticky="nsew", padx=(5, 0))
        
        logging_frame = ttk.LabelFrame(col1, text="📝 Logging Settings", padding="8")
        logging_frame.pack(fill="x", pady=(0, 10))
        self.setup_logging_config_compact(logging_frame)
        
        system_frame = ttk.LabelFrame(col1, text="⚙️ System Settings", padding="8")
        system_frame.pack(fill="x", pady=(0, 10))
        self.setup_system_config_compact(system_frame)
        
        can_frame = ttk.LabelFrame(col2, text="🚗 CAN Bus Settings", padding="8")
        can_frame.pack(fill="x", pady=(0, 10))
        self.setup_can_config_compact(can_frame)
        
        gps_frame = ttk.LabelFrame(col2, text="📡 GPS Settings", padding="8")
        gps_frame.pack(fill="x", pady=(0, 10))
        self.setup_gps_config_compact(gps_frame)
        
        i2c_frame = ttk.LabelFrame(col2, text="🔌 I2C Sensor Settings", padding="8")
        i2c_frame.pack(fill="x", pady=(0, 10))
        self.setup_i2c_config_compact(i2c_frame)
        
        wifi_frame = ttk.LabelFrame(col3, text="🌐 WiFi Settings", padding="8")
        wifi_frame.pack(fill="x", pady=(0, 10))
        self.setup_wifi_config_compact(wifi_frame)
        
        mqtt_frame = ttk.LabelFrame(col3, text="📨 MQTT Settings", padding="8")
        mqtt_frame.pack(fill="x", pady=(0, 10))
        self.setup_mqtt_config_compact(mqtt_frame)
        
        action_frame = ttk.Frame(main_container)
        action_frame.pack(fill="x", pady=15)
        
        ttk.Separator(action_frame, orient='horizontal').pack(fill="x", pady=10)
        
        btn_frame = ttk.Frame(action_frame)
        btn_frame.pack()
        
        ttk.Button(btn_frame, text="💾 Save to File", command=self.save_config,
                style='Primary.TButton', width=15).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="📡 Send to Device", command=self.send_config_to_device,
                style='Success.TButton', width=15).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="🔄 Reload from File", command=self.reload_config,
                style='Primary.TButton', width=15).pack(side="left", padx=5)
        
        self.config_status = ttk.Label(action_frame, text="", foreground="#10b981")
        self.config_status.pack(pady=10)

    def setup_logging_config_compact(self, parent):
        interval_frame = ttk.Frame(parent)
        interval_frame.pack(fill="x", pady=2)
        ttk.Label(interval_frame, text="Interval:", width=12, anchor="w").pack(side="left")
        self.log_interval = ttk.Entry(interval_frame, width=10)
        self.log_interval.insert(0, str(self.config['logging']['interval_ms']))
        self.log_interval.pack(side="right")
        self.log_interval.bind('<KeyRelease>', self.update_logging_config)
        
        size_frame = ttk.Frame(parent)
        size_frame.pack(fill="x", pady=2)
        ttk.Label(size_frame, text="Max File Size:", width=12, anchor="w").pack(side="left")
        self.max_file_size = ttk.Entry(size_frame, width=10)
        self.max_file_size.insert(0, str(self.config['logging']['max_file_size_mb']))
        self.max_file_size.pack(side="right")
        self.max_file_size.bind('<KeyRelease>', self.update_logging_config)
        
        delete_frame = ttk.Frame(parent)
        delete_frame.pack(fill="x", pady=2)
        ttk.Label(delete_frame, text="Auto Delete:", width=12, anchor="w").pack(side="left")
        self.auto_delete_days = ttk.Entry(delete_frame, width=10)
        self.auto_delete_days.insert(0, str(self.config['logging']['auto_delete_days']))
        self.auto_delete_days.pack(side="right")
        self.auto_delete_days.bind('<KeyRelease>', self.update_logging_config)
        
        self.rotate_hourly = tk.BooleanVar(value=self.config['logging']['rotate_hourly'])
        ttk.Checkbutton(parent, text="Rotate hourly", variable=self.rotate_hourly).pack(anchor="w", pady=1)
        self.rotate_hourly.trace_add('write', self.update_logging_config)
        
        self.include_date = tk.BooleanVar(value=self.config['logging']['include_date_in_filename'])
        ttk.Checkbutton(parent, text="Include date", variable=self.include_date).pack(anchor="w", pady=1)
        self.include_date.trace_add('write', self.update_logging_config)
        
        self.print_every = tk.BooleanVar(value=self.config['logging']['print_every_message'])
        ttk.Checkbutton(parent, text="Print CAN msgs", variable=self.print_every).pack(anchor="w", pady=1)
        self.print_every.trace_add('write', self.update_logging_config)
        
        self.print_logged = tk.BooleanVar(value=self.config['logging']['print_logged_data'])
        ttk.Checkbutton(parent, text="Print logged", variable=self.print_logged).pack(anchor="w", pady=1)
        self.print_logged.trace_add('write', self.update_logging_config)

    def setup_can_config_compact(self, parent):
        filter_frame = ttk.Frame(parent)
        filter_frame.pack(fill="x", pady=2)
        ttk.Label(filter_frame, text="Filter Mode:", width=12, anchor="w").pack(side="left")
        self.filter_mode = ttk.Combobox(filter_frame, values=[0, 1], width=8, state="readonly")
        self.filter_mode.set(self.config['can']['filter_mode'])
        self.filter_mode.pack(side="right")
        self.filter_mode.bind('<<ComboboxSelected>>', self.update_can_config)
        
        baud_frame = ttk.Frame(parent)
        baud_frame.pack(fill="x", pady=2)
        ttk.Label(baud_frame, text="CAN Baud:", width=12, anchor="w").pack(side="left")
        self.can_baud = ttk.Combobox(baud_frame, values=[125, 250, 500, 1000], width=8, state="readonly")
        self.can_baud.set(self.config['can']['baud_rate'])
        self.can_baud.pack(side="right")
        self.can_baud.bind('<<ComboboxSelected>>', self.update_can_config)
        
        queue_frame = ttk.Frame(parent)
        queue_frame.pack(fill="x", pady=2)
        ttk.Label(queue_frame, text="RX Queue:", width=12, anchor="w").pack(side="left")
        self.rx_queue_len = ttk.Entry(queue_frame, width=10)
        self.rx_queue_len.insert(0, str(self.config['can']['rx_queue_len']))
        self.rx_queue_len.pack(side="right")
        self.rx_queue_len.bind('<KeyRelease>', self.update_can_config)

    def setup_gps_config_compact(self, parent):
        baud_frame = ttk.Frame(parent)
        baud_frame.pack(fill="x", pady=2)
        ttk.Label(baud_frame, text="GPS Baud:", width=12, anchor="w").pack(side="left")
        self.gps_baud = ttk.Combobox(baud_frame, values=[9600, 19200, 38400, 57600, 115200], width=8, state="readonly")
        self.gps_baud.set(self.config['gps']['baud_rate'])
        self.gps_baud.pack(side="right")
        self.gps_baud.bind('<<ComboboxSelected>>', self.update_gps_config)
        
        interval_frame = ttk.Frame(parent)
        interval_frame.pack(fill="x", pady=2)
        ttk.Label(interval_frame, text="Update Int:", width=12, anchor="w").pack(side="left")
        self.gps_interval = ttk.Entry(interval_frame, width=10)
        self.gps_interval.insert(0, str(self.config['gps']['update_interval_ms']))
        self.gps_interval.pack(side="right")
        self.gps_interval.bind('<KeyRelease>', self.update_gps_config)

    def setup_i2c_config_compact(self, parent):
        interval_frame = ttk.Frame(parent)
        interval_frame.pack(fill="x", pady=2)
        ttk.Label(interval_frame, text="Update Int:", width=12, anchor="w").pack(side="left")
        self.i2c_interval = ttk.Entry(interval_frame, width=10)
        self.i2c_interval.insert(0, str(self.config['i2c']['update_interval_ms']))
        self.i2c_interval.pack(side="right")
        self.i2c_interval.bind('<KeyRelease>', self.update_i2c_config)
        
        ttk.Label(parent, text="Web: /i2c_config.html", foreground="#808080", font=('Segoe UI', 7)).pack(anchor="w", pady=4)

    def setup_wifi_config_compact(self, parent):
        ssid_frame = ttk.Frame(parent)
        ssid_frame.pack(fill="x", pady=2)
        ttk.Label(ssid_frame, text="Station SSID:", width=12, anchor="w").pack(side="left")
        self.wifi_ssid = ttk.Entry(ssid_frame, width=18)
        self.wifi_ssid.insert(0, self.config['wifi']['ssid'])
        self.wifi_ssid.pack(side="right")
        self.wifi_ssid.bind('<KeyRelease>', self.update_wifi_config)
        
        pass_frame = ttk.Frame(parent)
        pass_frame.pack(fill="x", pady=2)
        ttk.Label(pass_frame, text="Station Pwd:", width=12, anchor="w").pack(side="left")
        pwd_container = ttk.Frame(pass_frame)
        pwd_container.pack(side="right")
        self.wifi_password_entry = ttk.Entry(pwd_container, width=15, show="*")
        self.wifi_password_entry.insert(0, self.config['wifi']['password'])
        self.wifi_password_entry.pack(side="left")
        wifi_toggle_btn = ttk.Button(pwd_container, text="👁️", width=2, 
                                    command=self.toggle_wifi_password)
        wifi_toggle_btn.pack(side="left", padx=(2, 0))
        
        ap_ssid_frame = ttk.Frame(parent)
        ap_ssid_frame.pack(fill="x", pady=2)
        ttk.Label(ap_ssid_frame, text="AP SSID:", width=12, anchor="w").pack(side="left")
        self.ap_ssid = ttk.Entry(ap_ssid_frame, width=18)
        self.ap_ssid.insert(0, self.config['wifi']['ap_ssid'])
        self.ap_ssid.pack(side="right")
        self.ap_ssid.bind('<KeyRelease>', self.update_wifi_config)
        
        ap_pass_frame = ttk.Frame(parent)
        ap_pass_frame.pack(fill="x", pady=2)
        ttk.Label(ap_pass_frame, text="AP Password:", width=12, anchor="w").pack(side="left")
        ap_pwd_container = ttk.Frame(ap_pass_frame)
        ap_pwd_container.pack(side="right")
        self.ap_password_entry = ttk.Entry(ap_pwd_container, width=15, show="*")
        self.ap_password_entry.insert(0, self.config['wifi']['ap_password'])
        self.ap_password_entry.pack(side="left")
        ap_toggle_btn = ttk.Button(ap_pwd_container, text="👁️", width=2, 
                                command=self.toggle_ap_password)
        ap_toggle_btn.pack(side="left", padx=(2, 0))

    def setup_mqtt_config_compact(self, parent):
        broker_frame = ttk.Frame(parent)
        broker_frame.pack(fill="x", pady=2)
        ttk.Label(broker_frame, text="Broker:", width=12, anchor="w").pack(side="left")
        self.mqtt_broker = ttk.Entry(broker_frame, width=25)
        self.mqtt_broker.insert(0, self.config['mqtt']['broker'])
        self.mqtt_broker.pack(side="right")
        self.mqtt_broker.bind('<KeyRelease>', self.update_mqtt_config)
        
        port_frame = ttk.Frame(parent)
        port_frame.pack(fill="x", pady=2)
        ttk.Label(port_frame, text="Port:", width=12, anchor="w").pack(side="left")
        self.mqtt_port = ttk.Entry(port_frame, width=10)
        self.mqtt_port.insert(0, str(self.config['mqtt']['port']))
        self.mqtt_port.pack(side="right")
        self.mqtt_port.bind('<KeyRelease>', self.update_mqtt_config)
        
        topic_frame = ttk.Frame(parent)
        topic_frame.pack(fill="x", pady=2)
        ttk.Label(topic_frame, text="Topic:", width=12, anchor="w").pack(side="left")
        self.mqtt_topic = ttk.Entry(topic_frame, width=25)
        self.mqtt_topic.insert(0, self.config['mqtt']['topic'])
        self.mqtt_topic.pack(side="right")
        self.mqtt_topic.bind('<KeyRelease>', self.update_mqtt_config)
        
        client_frame = ttk.Frame(parent)
        client_frame.pack(fill="x", pady=2)
        ttk.Label(client_frame, text="Client ID:", width=12, anchor="w").pack(side="left")
        self.mqtt_client_id = ttk.Entry(client_frame, width=20)
        self.mqtt_client_id.insert(0, self.config['mqtt']['client_id'])
        self.mqtt_client_id.pack(side="right")
        self.mqtt_client_id.bind('<KeyRelease>', self.update_mqtt_config)
        
        user_frame = ttk.Frame(parent)
        user_frame.pack(fill="x", pady=2)
        ttk.Label(user_frame, text="Username:", width=12, anchor="w").pack(side="left")
        self.mqtt_username = ttk.Entry(user_frame, width=20)
        self.mqtt_username.insert(0, self.config['mqtt']['username'])
        self.mqtt_username.pack(side="right")
        self.mqtt_username.bind('<KeyRelease>', self.update_mqtt_config)
        
        pass_frame = ttk.Frame(parent)
        pass_frame.pack(fill="x", pady=2)
        ttk.Label(pass_frame, text="Password:", width=12, anchor="w").pack(side="left")
        pwd_container = ttk.Frame(pass_frame)
        pwd_container.pack(side="right")
        self.mqtt_password_entry = ttk.Entry(pwd_container, width=17, show="*")
        self.mqtt_password_entry.insert(0, self.config['mqtt']['password'])
        self.mqtt_password_entry.pack(side="left")
        mqtt_toggle_btn = ttk.Button(pwd_container, text="👁️", width=2, 
                                    command=self.toggle_mqtt_password)
        mqtt_toggle_btn.pack(side="left", padx=(2, 0))
    
    def toggle_wifi_password(self):
        self.show_wifi_password = not self.show_wifi_password
        self.wifi_password_entry.config(show="" if self.show_wifi_password else "*")
        self.config['wifi']['password'] = self.wifi_password_entry.get()

    def toggle_ap_password(self):
        self.show_ap_password = not self.show_ap_password
        self.ap_password_entry.config(show="" if self.show_ap_password else "*")
        self.config['wifi']['ap_password'] = self.ap_password_entry.get()

    def toggle_mqtt_password(self):
        self.show_mqtt_password = not self.show_mqtt_password
        self.mqtt_password_entry.config(show="" if self.show_mqtt_password else "*")
        self.config['mqtt']['password'] = self.mqtt_password_entry.get()

    def setup_system_config_compact(self, parent):
        self.serial_debug = tk.BooleanVar(value=self.config['system']['serial_debug'])
        ttk.Checkbutton(parent, text="Serial Debug Output", variable=self.serial_debug).pack(anchor="w", pady=2)
        self.serial_debug.trace_add('write', self.update_system_config)
        
        buffer_frame = ttk.Frame(parent)
        buffer_frame.pack(fill="x", pady=2)
        ttk.Label(buffer_frame, text="Buffer Size:", width=12, anchor="w").pack(side="left")
        self.buffer_size = ttk.Entry(buffer_frame, width=10)
        self.buffer_size.insert(0, str(self.config['system']['buffer_size']))
        self.buffer_size.pack(side="right")
        self.buffer_size.bind('<KeyRelease>', self.update_system_config)
        
        flush_frame = ttk.Frame(parent)
        flush_frame.pack(fill="x", pady=2)
        ttk.Label(flush_frame, text="Flush Int:", width=12, anchor="w").pack(side="left")
        self.flush_interval = ttk.Entry(flush_frame, width=10)
        self.flush_interval.insert(0, str(self.config['system']['flush_interval']))
        self.flush_interval.pack(side="right")
        self.flush_interval.bind('<KeyRelease>', self.update_system_config)
        
        timeout_frame = ttk.Frame(parent)
        timeout_frame.pack(fill="x", pady=2)
        ttk.Label(timeout_frame, text="ECU Timeout:", width=12, anchor="w").pack(side="left")
        self.ecu_timeout = ttk.Entry(timeout_frame, width=10)
        self.ecu_timeout.insert(0, str(self.config['system']['ecu_disconnect_timeout']))
        self.ecu_timeout.pack(side="right")
        self.ecu_timeout.bind('<KeyRelease>', self.update_system_config)
    
    def setup_firmware_tab(self, parent):
        main_container = ttk.Frame(parent)
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        header_frame = ttk.Frame(main_container)
        header_frame.pack(fill="x", pady=(0, 15))
        
        ttk.Label(header_frame, text="🔧 Firmware Update", 
                font=('Segoe UI', 14, 'bold'), foreground='#4681a9').pack(side="left")
        
        ttk.Separator(main_container, orient='horizontal').pack(fill="x", pady=5)
        
        chip_notebook = ttk.Notebook(main_container)
        chip_notebook.pack(fill="both", expand=True, pady=10)
        
        esp32_frame = ttk.Frame(chip_notebook)
        chip_notebook.add(esp32_frame, text="ESP32")
        self.setup_esp32_flash_tab(esp32_frame)
        
        esp8266_frame = ttk.Frame(chip_notebook)
        chip_notebook.add(esp8266_frame, text="ESP8266")
        self.setup_esp8266_flash_tab(esp8266_frame)
        
        generic_frame = ttk.Frame(chip_notebook)
        chip_notebook.add(generic_frame, text="Generic")
        self.setup_generic_flash_tab(generic_frame)

    def setup_esp32_flash_tab(self, parent):
        main_paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_paned.pack(fill="both", expand=True)
        
        left_panel = ttk.Frame(main_paned)
        main_paned.add(left_panel, weight=1)
        
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=1)
        
        canvas = tk.Canvas(left_panel, bg='#0a0a0a', highlightthickness=0)
        scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        content = ttk.Frame(scrollable_frame, padding="10")
        content.pack(fill="both", expand=True)
        
        file_frame = ttk.LabelFrame(content, text="📁 Firmware File", padding="10")
        file_frame.pack(fill="x", pady=(0, 10))
        
        file_select_frame = ttk.Frame(file_frame)
        file_select_frame.pack(fill="x")
        
        self.esp32_firmware_path = tk.StringVar()
        self.esp32_firmware_entry = ttk.Entry(file_select_frame, textvariable=self.esp32_firmware_path, width=40)
        self.esp32_firmware_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        ttk.Button(file_select_frame, text="Browse", command=self.select_esp32_firmware,
                style='Primary.TButton', width=10).pack(side="right")
        
        settings_frame = ttk.LabelFrame(content, text="⚙️ Flash Settings", padding="10")
        settings_frame.pack(fill="x", pady=(0, 10))
        
        addr_frame = ttk.Frame(settings_frame)
        addr_frame.pack(fill="x", pady=5)
        ttk.Label(addr_frame, text="Flash Address:", width=15).pack(side="left")
        self.esp32_flash_addr = ttk.Combobox(addr_frame, values=["0x1000", "0x0000", "0x10000"], width=15)
        self.esp32_flash_addr.set("0x1000")
        self.esp32_flash_addr.pack(side="left", padx=5)
        ttk.Label(addr_frame, text="(Default: 0x1000)").pack(side="left", padx=5)
        
        baud_frame = ttk.Frame(settings_frame)
        baud_frame.pack(fill="x", pady=5)
        ttk.Label(baud_frame, text="Baud Rate:", width=15).pack(side="left")
        self.esp32_baud = ttk.Combobox(baud_frame, values=["115200", "230400", "460800", "921600"], width=15)
        self.esp32_baud.set("921600")
        self.esp32_baud.pack(side="left", padx=5)
        
        self.esp32_erase_flash = tk.BooleanVar(value=False)
        ttk.Checkbutton(settings_frame, text="Erase flash before flashing", 
                        variable=self.esp32_erase_flash).pack(anchor="w", pady=5)
        
        self.esp32_verify = tk.BooleanVar(value=True)
        ttk.Checkbutton(settings_frame, text="Verify after flashing", 
                        variable=self.esp32_verify).pack(anchor="w", pady=5)
        
        flash_frame = ttk.Frame(content)
        flash_frame.pack(fill="x", pady=10)
        
        self.esp32_flash_btn = ttk.Button(flash_frame, text="🔥 Flash ESP32", 
                                        command=self.flash_esp32, style='Success.TButton')
        self.esp32_flash_btn.pack()
        
        progress_frame = ttk.LabelFrame(content, text="📊 Progress", padding="10")
        progress_frame.pack(fill="x", pady=(0, 10))
        
        self.esp32_progress_bar = ttk.Progressbar(progress_frame, length=400, mode='determinate')
        self.esp32_progress_bar.pack(fill="x")
        
        self.esp32_progress_label = ttk.Label(progress_frame, text="Ready", foreground="#808080")
        self.esp32_progress_label.pack(pady=5)
        
        console_frame = ttk.LabelFrame(right_panel, text="📝 Flash Console", padding="10")
        console_frame.pack(fill="both", expand=True)
        
        self.esp32_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD,
                                                        bg='#0a0a0a', fg='#e0e0e0',
                                                        insertbackground='#e0e0e0',
                                                        font=('Consolas', 8),
                                                        bd=0, highlightthickness=0)
        self.esp32_console.pack(fill="both", expand=True)
        
        self.esp32_console.tag_config("info", foreground="#3a7ca5")
        self.esp32_console.tag_config("success", foreground="#10b981")
        self.esp32_console.tag_config("error", foreground="#ef4444")
        self.esp32_console.tag_config("progress", foreground="#4681a9")

    def setup_esp8266_flash_tab(self, parent):
        main_paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_paned.pack(fill="both", expand=True)
        
        left_panel = ttk.Frame(main_paned)
        main_paned.add(left_panel, weight=1)
        
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=1)
        
        canvas = tk.Canvas(left_panel, bg='#0a0a0a', highlightthickness=0)
        scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        content = ttk.Frame(scrollable_frame, padding="10")
        content.pack(fill="both", expand=True)
        
        file_frame = ttk.LabelFrame(content, text="📁 Firmware File", padding="10")
        file_frame.pack(fill="x", pady=(0, 10))
        
        file_select_frame = ttk.Frame(file_frame)
        file_select_frame.pack(fill="x")
        
        self.esp8266_firmware_path = tk.StringVar()
        self.esp8266_firmware_entry = ttk.Entry(file_select_frame, textvariable=self.esp8266_firmware_path, width=40)
        self.esp8266_firmware_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        ttk.Button(file_select_frame, text="Browse", command=self.select_esp8266_firmware,
                style='Primary.TButton', width=10).pack(side="right")
        
        settings_frame = ttk.LabelFrame(content, text="⚙️ Flash Settings", padding="10")
        settings_frame.pack(fill="x", pady=(0, 10))
        
        addr_frame = ttk.Frame(settings_frame)
        addr_frame.pack(fill="x", pady=5)
        ttk.Label(addr_frame, text="Flash Address:", width=15).pack(side="left")
        self.esp8266_flash_addr = ttk.Combobox(addr_frame, values=["0x00000", "0x10000"], width=15)
        self.esp8266_flash_addr.set("0x00000")
        self.esp8266_flash_addr.pack(side="left", padx=5)
        
        baud_frame = ttk.Frame(settings_frame)
        baud_frame.pack(fill="x", pady=5)
        ttk.Label(baud_frame, text="Baud Rate:", width=15).pack(side="left")
        self.esp8266_baud = ttk.Combobox(baud_frame, values=["115200", "230400", "460800", "921600"], width=15)
        self.esp8266_baud.set("460800")
        self.esp8266_baud.pack(side="left", padx=5)
        
        mode_frame = ttk.Frame(settings_frame)
        mode_frame.pack(fill="x", pady=5)
        ttk.Label(mode_frame, text="Flash Mode:", width=15).pack(side="left")
        self.esp8266_flash_mode = ttk.Combobox(mode_frame, values=["qio", "dio", "qout", "dout"], width=10)
        self.esp8266_flash_mode.set("dio")
        self.esp8266_flash_mode.pack(side="left", padx=5)
        
        size_frame = ttk.Frame(settings_frame)
        size_frame.pack(fill="x", pady=5)
        ttk.Label(size_frame, text="Flash Size:", width=15).pack(side="left")
        self.esp8266_flash_size = ttk.Combobox(size_frame, values=["1MB", "2MB", "4MB"], width=10)
        self.esp8266_flash_size.set("4MB")
        self.esp8266_flash_size.pack(side="left", padx=5)
        
        self.esp8266_erase_flash = tk.BooleanVar(value=False)
        ttk.Checkbutton(settings_frame, text="Erase flash before flashing", 
                        variable=self.esp8266_erase_flash).pack(anchor="w", pady=5)
        
        flash_frame = ttk.Frame(content)
        flash_frame.pack(fill="x", pady=10)
        
        self.esp8266_flash_btn = ttk.Button(flash_frame, text="🔥 Flash ESP8266", 
                                            command=self.flash_esp8266, style='Success.TButton')
        self.esp8266_flash_btn.pack()
        
        progress_frame = ttk.LabelFrame(content, text="📊 Progress", padding="10")
        progress_frame.pack(fill="x", pady=(0, 10))
        
        self.esp8266_progress_bar = ttk.Progressbar(progress_frame, length=400, mode='determinate')
        self.esp8266_progress_bar.pack(fill="x")
        
        self.esp8266_progress_label = ttk.Label(progress_frame, text="Ready", foreground="#808080")
        self.esp8266_progress_label.pack(pady=5)
        
        console_frame = ttk.LabelFrame(right_panel, text="📝 Flash Console", padding="10")
        console_frame.pack(fill="both", expand=True)
        
        self.esp8266_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD,
                                                        bg='#0a0a0a', fg='#e0e0e0',
                                                        insertbackground='#e0e0e0',
                                                        font=('Consolas', 8),
                                                        bd=0, highlightthickness=0)
        self.esp8266_console.pack(fill="both", expand=True)
        
        self.esp8266_console.tag_config("info", foreground="#3a7ca5")
        self.esp8266_console.tag_config("success", foreground="#10b981")
        self.esp8266_console.tag_config("error", foreground="#ef4444")
        self.esp8266_console.tag_config("progress", foreground="#4681a9")

    def setup_generic_flash_tab(self, parent):
        main_paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_paned.pack(fill="both", expand=True)
        
        left_panel = ttk.Frame(main_paned)
        main_paned.add(left_panel, weight=1)
        
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=1)
        
        canvas = tk.Canvas(left_panel, bg='#0a0a0a', highlightthickness=0)
        scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        content = ttk.Frame(scrollable_frame, padding="10")
        content.pack(fill="both", expand=True)
        
        file_frame = ttk.LabelFrame(content, text="📁 Firmware File", padding="10")
        file_frame.pack(fill="x", pady=(0, 10))
        
        file_select_frame = ttk.Frame(file_frame)
        file_select_frame.pack(fill="x")
        
        self.generic_firmware_path = tk.StringVar()
        self.generic_firmware_entry = ttk.Entry(file_select_frame, textvariable=self.generic_firmware_path, width=40)
        self.generic_firmware_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        ttk.Button(file_select_frame, text="Browse", command=self.select_generic_firmware,
                style='Primary.TButton', width=10).pack(side="right")
        
        settings_frame = ttk.LabelFrame(content, text="⚙️ Flash Settings", padding="10")
        settings_frame.pack(fill="x", pady=(0, 10))
        
        chip_frame = ttk.Frame(settings_frame)
        chip_frame.pack(fill="x", pady=5)
        ttk.Label(chip_frame, text="Chip Type:", width=15).pack(side="left")
        self.generic_chip = ttk.Combobox(chip_frame, values=["esp32", "esp8266", "esp32s2", "esp32c3"], width=15)
        self.generic_chip.set("esp32")
        self.generic_chip.pack(side="left", padx=5)
        
        addr_frame = ttk.Frame(settings_frame)
        addr_frame.pack(fill="x", pady=5)
        ttk.Label(addr_frame, text="Flash Address:", width=15).pack(side="left")
        self.generic_flash_addr = ttk.Entry(addr_frame, width=15)
        self.generic_flash_addr.insert(0, "0x1000")
        self.generic_flash_addr.pack(side="left", padx=5)
        
        baud_frame = ttk.Frame(settings_frame)
        baud_frame.pack(fill="x", pady=5)
        ttk.Label(baud_frame, text="Baud Rate:", width=15).pack(side="left")
        self.generic_baud = ttk.Combobox(baud_frame, values=["115200", "230400", "460800", "921600"], width=15)
        self.generic_baud.set("460800")
        self.generic_baud.pack(side="left", padx=5)
        
        flash_frame = ttk.Frame(content)
        flash_frame.pack(fill="x", pady=10)
        
        self.generic_flash_btn = ttk.Button(flash_frame, text="🔥 Flash Device", 
                                            command=self.flash_generic, style='Success.TButton')
        self.generic_flash_btn.pack()
        
        progress_frame = ttk.LabelFrame(content, text="📊 Progress", padding="10")
        progress_frame.pack(fill="x", pady=(0, 10))
        
        self.generic_progress_bar = ttk.Progressbar(progress_frame, length=400, mode='determinate')
        self.generic_progress_bar.pack(fill="x")
        
        self.generic_progress_label = ttk.Label(progress_frame, text="Ready", foreground="#808080")
        self.generic_progress_label.pack(pady=5)
        
        console_frame = ttk.LabelFrame(right_panel, text="📝 Flash Console", padding="10")
        console_frame.pack(fill="both", expand=True)
        
        self.generic_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD,
                                                        bg='#0a0a0a', fg='#e0e0e0',
                                                        insertbackground='#e0e0e0',
                                                        font=('Consolas', 8),
                                                        bd=0, highlightthickness=0)
        self.generic_console.pack(fill="both", expand=True)
        
        self.generic_console.tag_config("info", foreground="#3a7ca5")
        self.generic_console.tag_config("success", foreground="#10b981")
        self.generic_console.tag_config("error", foreground="#ef4444")
        self.generic_console.tag_config("progress", foreground="#4681a9")

    def select_esp32_firmware(self):
        filename = filedialog.askopenfilename(
            title="Select ESP32 Firmware (.bin)",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if filename:
            self.esp32_firmware_path.set(filename)
            self.log_to_console(self.esp32_console, f"Selected: {filename}", "info")

    def select_esp8266_firmware(self):
        filename = filedialog.askopenfilename(
            title="Select ESP8266 Firmware (.bin)",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if filename:
            self.esp8266_firmware_path.set(filename)
            self.log_to_console(self.esp8266_console, f"Selected: {filename}", "info")

    def select_generic_firmware(self):
        filename = filedialog.askopenfilename(
            title="Select Firmware (.bin)",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if filename:
            self.generic_firmware_path.set(filename)
            self.log_to_console(self.generic_console, f"Selected: {filename}", "info")

    def log_to_console(self, console, message, tag="info"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        console.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        console.see(tk.END)
        self.root.update_idletasks()

    def _find_esptool(self):
        """Find esptool executable or module"""
        # Method 1: Try to find esptool.py in PATH
        esptool_py = shutil.which("esptool.py")
        if esptool_py:
            self.log_to_console(self.esp32_console, f"Found esptool: {esptool_py}", "info")
            return esptool_py
        
        # Method 2: Try to find esptool in PATH
        esptool_cmd = shutil.which("esptool")
        if esptool_cmd:
            self.log_to_console(self.esp32_console, f"Found esptool: {esptool_cmd}", "info")
            return esptool_cmd
        
        # Method 3: Check user's Python scripts directory
        user_scripts = os.path.expanduser("~\\AppData\\Roaming\\Python\\Python3*\\Scripts\\esptool.exe")
        matches = glob.glob(user_scripts)
        if matches:
            self.log_to_console(self.esp32_console, f"Found esptool in Python scripts: {matches[0]}", "info")
            return matches[0]
        
        # Method 4: Try to use Python module
        try:
            result = subprocess.run([sys.executable, "-c", "import esptool; print('ok')"], 
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.log_to_console(self.esp32_console, "Using Python esptool module", "info")
                return "python_module"
        except:
            pass
        
        return None

    def flash_esp32(self):
        firmware_path = self.esp32_firmware_path.get()
        if not firmware_path:
            messagebox.showwarning("Warning", "Please select a firmware file")
            return
        
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Warning", "Please connect to the device first")
            return
        
        self.listener_running = False
        threading.Thread(target=self._flash_esp32_thread, daemon=True).start()

    def _flash_esp32_thread(self):
        try:
            self.log_to_console(self.esp32_console, "Starting ESP32 flash...", "info")
            self.esp32_flash_btn.config(state="disabled")
            self.esp32_progress_bar['value'] = 0
            self.esp32_progress_label.config(text="Starting...")
            
            port = self.port_combo.get()
            baud = self.esp32_baud.get()
            addr = self.esp32_flash_addr.get()
            firmware = self.esp32_firmware_path.get()
            
            # First, close the serial connection if it's open
            if self.serial_conn and self.serial_conn.is_open:
                self.log_to_console(self.esp32_console, "Closing serial connection...", "info")
                self.serial_conn.close()
                time.sleep(1)
            
            # Find esptool
            esptool_path = self._find_esptool()
            if not esptool_path:
                self.log_to_console(self.esp32_console, "❌ esptool not found! Please install it with: pip install esptool", "error")
                return
            
            # Build esptool command
            if esptool_path == "python_module":
                esptool_cmd = [sys.executable, "-m", "esptool"]
            else:
                esptool_cmd = [esptool_path]
            
            # Write firmware - this is the main flashing step
            self.log_to_console(self.esp32_console, f"Writing firmware to {addr}...", "progress")
            write_cmd = esptool_cmd + ["--port", port, "--baud", str(baud), "write_flash", addr, firmware]
            
            self._run_command(write_cmd, self.esp32_console, self.esp32_progress_bar, self.esp32_progress_label)
            
            self.log_to_console(self.esp32_console, "✅ Flash completed successfully!", "success")
            self.esp32_progress_label.config(text="Flash completed!")
            
            # Reset the ESP32 after flashing - FIXED: use a proper command
            self.log_to_console(self.esp32_console, "Resetting ESP32...", "info")
            try:
                # Just send a simple command to reset via DTR/RTS
                import serial
                reset_serial = serial.Serial(port, baudrate=115200, timeout=1)
                reset_serial.dtr = False
                reset_serial.rts = True
                time.sleep(0.1)
                reset_serial.dtr = True
                reset_serial.rts = False
                time.sleep(0.1)
                reset_serial.close()
                self.log_to_console(self.esp32_console, "ESP32 reset complete", "info")
            except Exception as e:
                self.log_to_console(self.esp32_console, f"Could not reset ESP32: {e}", "warning")
                self.log_to_console(self.esp32_console, "Please press the RESET button manually", "info")
            
        except Exception as e:
            self.log_to_console(self.esp32_console, f"❌ Flash failed: {e}", "error")
            self.esp32_progress_label.config(text="Flash failed!")
        finally:
            self.esp32_flash_btn.config(state="normal")
            self.listener_running = True
            # Try to reconnect
            try:
                if self.serial_conn and not self.serial_conn.is_open:
                    self.serial_conn.open()
                    self.connect()
            except:
                pass

    def _run_command(self, cmd, console, progress_bar=None, progress_label=None, timeout=30):
        """Run a command and capture output with timeout"""
        self.log_to_console(console, f"Running: {' '.join(cmd)}", "info")
        
        try:
            # Use PIPE with binary mode to avoid encoding issues
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=False
            )
            
            output_lines = []
            start_time = time.time()
            
            # Read output in binary mode and decode with error handling
            while True:
                # Check timeout
                if time.time() - start_time > timeout:
                    process.terminate()
                    raise Exception(f"Command timed out after {timeout} seconds")
                
                # Read line with timeout
                try:
                    line = process.stdout.readline()
                    if not line and process.poll() is not None:
                        break
                    if not line:
                        continue
                except:
                    break
                
                # Try multiple encodings
                line_str = None
                for encoding in ['utf-8', 'latin-1', 'cp1252', 'ascii']:
                    try:
                        line_str = line.decode(encoding, errors='replace').strip()
                        break
                    except:
                        continue
                
                if not line_str:
                    line_str = str(line).strip()
                
                if line_str:
                    # Filter out binary garbage
                    if not all(ord(c) < 32 and c not in '\n\r\t' for c in line_str):
                        output_lines.append(line_str)
                        self.log_to_console(console, line_str, "info")
                        
                        # Update progress if we see percentage
                        if "%" in line_str and progress_bar:
                            try:
                                import re
                                match = re.search(r'\((\d+)\s*%\)', line_str)
                                if not match:
                                    match = re.search(r'\((\d+)%\)', line_str)
                                if match:
                                    percent = float(match.group(1))
                                    progress_bar['value'] = percent
                                    if progress_label:
                                        progress_label.config(text=f"Progress: {percent:.1f}%")
                            except:
                                pass
            
            process.wait()
            
            # Check for common errors in output
            error_strings = ['error', 'failed', 'timeout', 'no such', 'not found', 'permission denied', 'fatal']
            for line in output_lines:
                line_lower = line.lower()
                if any(err in line_lower for err in error_strings):
                    if 'error' in line_lower and 'no error' not in line_lower:
                        self.log_to_console(console, f"⚠️ Warning: {line}", "error")
            
            if process.returncode != 0:
                # Provide helpful error messages
                if process.returncode == 2:
                    error_msg = "Connection failed. Make sure ESP32 is in download mode.\n"
                    error_msg += "Try holding BOOT button while clicking Flash."
                    raise Exception(error_msg)
                elif process.returncode == 1:
                    raise Exception("General error. Check COM port and connections.")
                else:
                    raise Exception(f"Command failed with exit code {process.returncode}")
                    
        except subprocess.TimeoutExpired:
            process.kill()
            raise Exception(f"Command timed out after {timeout} seconds")
        except Exception as e:
            self.log_to_console(console, f"Error: {e}", "error")
            raise

    def flash_esp8266(self):
        firmware_path = self.esp8266_firmware_path.get()
        if not firmware_path:
            messagebox.showwarning("Warning", "Please select a firmware file")
            return
        
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Warning", "Please connect to the device first")
            return
        
        self.listener_running = False
        threading.Thread(target=self._flash_esp8266_thread, daemon=True).start()

    def _flash_esp8266_thread(self):
        try:
            self.log_to_console(self.esp8266_console, "Starting ESP8266 flash...", "info")
            self.esp8266_flash_btn.config(state="disabled")
            self.esp8266_progress_bar['value'] = 0
            self.esp8266_progress_label.config(text="Starting...")
            
            port = self.port_combo.get()
            baud = self.esp8266_baud.get()
            addr = self.esp8266_flash_addr.get()
            firmware = self.esp8266_firmware_path.get()
            flash_mode = self.esp8266_flash_mode.get()
            flash_size = self.esp8266_flash_size.get()
            
            esptool_path = self._find_esptool()
            if not esptool_path:
                self.log_to_console(self.esp8266_console, "❌ esptool not found!", "error")
                return
            
            if esptool_path == "python_module":
                esptool_cmd = [sys.executable, "-m", "esptool"]
            else:
                esptool_cmd = [esptool_path]
            
            esptool_cmd.extend(["--port", port, "--baud", baud])
            
            if self.esp8266_erase_flash.get():
                self.log_to_console(self.esp8266_console, "Erasing flash...", "progress")
                erase_cmd = esptool_cmd + ["erase_flash"]
                self._run_command(erase_cmd, self.esp8266_console)
            
            self.log_to_console(self.esp8266_console, f"Writing firmware to {addr}...", "progress")
            write_cmd = esptool_cmd + ["write_flash", "--flash_mode", flash_mode, 
                                        "--flash_size", flash_size, addr, firmware]
            
            self._run_command(write_cmd, self.esp8266_console, self.esp8266_progress_bar, self.esp8266_progress_label)
            
            self.log_to_console(self.esp8266_console, "✅ Flash completed successfully!", "success")
            self.esp8266_progress_label.config(text="Flash completed!")
            
        except Exception as e:
            self.log_to_console(self.esp8266_console, f"❌ Flash failed: {e}", "error")
            self.esp8266_progress_label.config(text="Flash failed!")
        finally:
            self.esp8266_flash_btn.config(state="normal")
            self.listener_running = True
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.connect()

    def flash_generic(self):
        firmware_path = self.generic_firmware_path.get()
        if not firmware_path:
            messagebox.showwarning("Warning", "Please select a firmware file")
            return
        
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Warning", "Please connect to the device first")
            return
        
        self.listener_running = False
        threading.Thread(target=self._flash_generic_thread, daemon=True).start()

    def _flash_generic_thread(self):
        try:
            self.log_to_console(self.generic_console, f"Starting flash for {self.generic_chip.get()}...", "info")
            self.generic_flash_btn.config(state="disabled")
            self.generic_progress_bar['value'] = 0
            self.generic_progress_label.config(text="Starting...")
            
            port = self.port_combo.get()
            baud = self.generic_baud.get()
            addr = self.generic_flash_addr.get()
            firmware = self.generic_firmware_path.get()
            chip = self.generic_chip.get()
            
            esptool_path = self._find_esptool()
            if not esptool_path:
                self.log_to_console(self.generic_console, "❌ esptool not found!", "error")
                return
            
            if esptool_path == "python_module":
                esptool_cmd = [sys.executable, "-m", "esptool", "--chip", chip]
            else:
                esptool_cmd = [esptool_path, "--chip", chip]
            
            esptool_cmd.extend(["--port", port, "--baud", baud])
            
            self.log_to_console(self.generic_console, f"Writing firmware to {addr}...", "progress")
            write_cmd = esptool_cmd + ["write_flash", addr, firmware]
            
            self._run_command(write_cmd, self.generic_console, self.generic_progress_bar, self.generic_progress_label)
            
            self.log_to_console(self.generic_console, "✅ Flash completed successfully!", "success")
            self.generic_progress_label.config(text="Flash completed!")
            
        except Exception as e:
            self.log_to_console(self.generic_console, f"❌ Flash failed: {e}", "error")
            self.generic_progress_label.config(text="Flash failed!")
        finally:
            self.generic_flash_btn.config(state="normal")
            self.listener_running = True
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.connect()
    
    def update_logging_config(self, *args):
        try:
            self.config['logging']['interval_ms'] = int(self.log_interval.get())
        except:
            pass
        try:
            self.config['logging']['max_file_size_mb'] = int(self.max_file_size.get())
        except:
            pass
        try:
            self.config['logging']['auto_delete_days'] = int(self.auto_delete_days.get())
        except:
            pass
        self.config['logging']['rotate_hourly'] = self.rotate_hourly.get()
        self.config['logging']['include_date_in_filename'] = self.include_date.get()
        self.config['logging']['print_every_message'] = self.print_every.get()
        self.config['logging']['print_logged_data'] = self.print_logged.get()
    
    def update_can_config(self, *args):
        try:
            self.config['can']['filter_mode'] = int(self.filter_mode.get())
        except:
            pass
        try:
            self.config['can']['baud_rate'] = int(self.can_baud.get())
        except:
            pass
        try:
            self.config['can']['rx_queue_len'] = int(self.rx_queue_len.get())
        except:
            pass
    
    def update_gps_config(self, *args):
        try:
            self.config['gps']['baud_rate'] = int(self.gps_baud.get())
        except:
            pass
        try:
            self.config['gps']['update_interval_ms'] = int(self.gps_interval.get())
        except:
            pass
    
    def update_wifi_config(self, *args):
        self.config['wifi']['ssid'] = self.wifi_ssid.get()
        self.config['wifi']['password'] = self.wifi_password_entry.get()
        self.config['wifi']['ap_ssid'] = self.ap_ssid.get()
        self.config['wifi']['ap_password'] = self.ap_password_entry.get()
    
    def update_mqtt_config(self, *args):
        self.config['mqtt']['broker'] = self.mqtt_broker.get()
        try:
            self.config['mqtt']['port'] = int(self.mqtt_port.get())
        except:
            pass
        self.config['mqtt']['topic'] = self.mqtt_topic.get()
        self.config['mqtt']['client_id'] = self.mqtt_client_id.get()
        self.config['mqtt']['username'] = self.mqtt_username.get()
        self.config['mqtt']['password'] = self.mqtt_password_entry.get()
    
    def update_i2c_config(self, *args):
        try:
            self.config['i2c']['update_interval_ms'] = int(self.i2c_interval.get())
        except:
            pass
    
    def update_system_config(self, *args):
        self.config['system']['serial_debug'] = self.serial_debug.get()
        try:
            self.config['system']['buffer_size'] = int(self.buffer_size.get())
        except:
            pass
        try:
            self.config['system']['flush_interval'] = int(self.flush_interval.get())
        except:
            pass
        try:
            self.config['system']['ecu_disconnect_timeout'] = int(self.ecu_timeout.get())
        except:
            pass
    
    def reload_config(self):
        self.config = self.load_config()
        self.refresh_config_ui()
        self.log("🔄 Configuration reloaded from file", "success")
        self.config_status.config(text="Configuration reloaded", foreground="#10b981")
        self.root.after(3000, lambda: self.config_status.config(text=""))
    
    def refresh_config_ui(self):
        self.log_interval.delete(0, tk.END)
        self.log_interval.insert(0, str(self.config['logging']['interval_ms']))
        self.max_file_size.delete(0, tk.END)
        self.max_file_size.insert(0, str(self.config['logging']['max_file_size_mb']))
        self.auto_delete_days.delete(0, tk.END)
        self.auto_delete_days.insert(0, str(self.config['logging']['auto_delete_days']))
        self.rotate_hourly.set(self.config['logging']['rotate_hourly'])
        self.include_date.set(self.config['logging']['include_date_in_filename'])
        self.print_every.set(self.config['logging']['print_every_message'])
        self.print_logged.set(self.config['logging']['print_logged_data'])
        
        self.filter_mode.set(self.config['can']['filter_mode'])
        self.can_baud.set(self.config['can']['baud_rate'])
        self.rx_queue_len.delete(0, tk.END)
        self.rx_queue_len.insert(0, str(self.config['can']['rx_queue_len']))
        
        self.gps_baud.set(self.config['gps']['baud_rate'])
        self.gps_interval.delete(0, tk.END)
        self.gps_interval.insert(0, str(self.config['gps']['update_interval_ms']))
        
        self.wifi_ssid.delete(0, tk.END)
        self.wifi_ssid.insert(0, self.config['wifi']['ssid'])
        self.wifi_password_entry.delete(0, tk.END)
        self.wifi_password_entry.insert(0, self.config['wifi']['password'])
        self.ap_ssid.delete(0, tk.END)
        self.ap_ssid.insert(0, self.config['wifi']['ap_ssid'])
        self.ap_password_entry.delete(0, tk.END)
        self.ap_password_entry.insert(0, self.config['wifi']['ap_password'])
        
        self.show_wifi_password = False
        self.wifi_password_entry.config(show="*")
        self.show_ap_password = False
        self.ap_password_entry.config(show="*")
        
        self.mqtt_broker.delete(0, tk.END)
        self.mqtt_broker.insert(0, self.config['mqtt']['broker'])
        self.mqtt_port.delete(0, tk.END)
        self.mqtt_port.insert(0, str(self.config['mqtt']['port']))
        self.mqtt_topic.delete(0, tk.END)
        self.mqtt_topic.insert(0, self.config['mqtt']['topic'])
        self.mqtt_client_id.delete(0, tk.END)
        self.mqtt_client_id.insert(0, self.config['mqtt']['client_id'])
        self.mqtt_username.delete(0, tk.END)
        self.mqtt_username.insert(0, self.config['mqtt']['username'])
        self.mqtt_password_entry.delete(0, tk.END)
        self.mqtt_password_entry.insert(0, self.config['mqtt']['password'])
        
        self.show_mqtt_password = False
        self.mqtt_password_entry.config(show="*")
        
        self.i2c_interval.delete(0, tk.END)
        self.i2c_interval.insert(0, str(self.config['i2c']['update_interval_ms']))
        
        self.serial_debug.set(self.config['system']['serial_debug'])
        self.buffer_size.delete(0, tk.END)
        self.buffer_size.insert(0, str(self.config['system']['buffer_size']))
        self.flush_interval.delete(0, tk.END)
        self.flush_interval.insert(0, str(self.config['system']['flush_interval']))
        self.ecu_timeout.delete(0, tk.END)
        self.ecu_timeout.insert(0, str(self.config['system']['ecu_disconnect_timeout']))
    
    def setup_files_tab(self, parent):
        main_container = ttk.Frame(parent)
        main_container.pack(fill="both", expand=True, padx=8, pady=8)
        
        paned = ttk.PanedWindow(main_container, orient=tk.HORIZONTAL)
        paned.pack(fill="both", expand=True)
        
        left_frame = ttk.LabelFrame(paned, text="📊 Data Files", padding="5")
        paned.add(left_frame, weight=1)
        
        list_container = ttk.Frame(left_frame)
        list_container.pack(fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(list_container)
        scrollbar.pack(side="right", fill="y")
        
        self.file_listbox = tk.Listbox(list_container, height=18, 
                                       bg='#1a1a1a', fg='#e0e0e0',
                                       selectbackground='#4681a9',
                                       selectforeground='#ffffff',
                                       font=('Consolas', 8),
                                       bd=0, highlightthickness=0)
        self.file_listbox.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.file_listbox.yview)
        self.file_listbox.config(yscrollcommand=scrollbar.set)
        self.file_listbox.bind("<Double-1>", lambda e: self.download_file())
        
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill="x", pady=4)
        
        ttk.Button(btn_frame, text="📥 Download", command=self.download_file,
                  style='Download.TButton').pack(side="left", padx=2, expand=True, fill="x")
        ttk.Button(btn_frame, text="🗑️ Delete", command=self.delete_file,
                  style='Delete.TButton').pack(side="left", padx=2, expand=True, fill="x")
        
        right_frame = ttk.LabelFrame(paned, text="🔧 Diagnostic Files", padding="5")
        paned.add(right_frame, weight=1)
        
        diag_container = ttk.Frame(right_frame)
        diag_container.pack(fill="both", expand=True)
        
        diag_scrollbar = ttk.Scrollbar(diag_container)
        diag_scrollbar.pack(side="right", fill="y")
        
        self.diag_listbox = tk.Listbox(diag_container, height=18,
                                       bg='#1a1a1a', fg='#e0e0e0',
                                       selectbackground='#4681a9',
                                       selectforeground='#ffffff',
                                       font=('Consolas', 8),
                                       bd=0, highlightthickness=0)
        self.diag_listbox.pack(side="left", fill="both", expand=True)
        diag_scrollbar.config(command=self.diag_listbox.yview)
        self.diag_listbox.config(yscrollcommand=diag_scrollbar.set)
        self.diag_listbox.bind("<Double-1>", lambda e: self.download_diag_file())
        
        diag_btn_frame = ttk.Frame(right_frame)
        diag_btn_frame.pack(fill="x", pady=4)
        
        ttk.Button(diag_btn_frame, text="📥 Download", command=self.download_diag_file,
                  style='Download.TButton').pack(side="left", padx=2, expand=True, fill="x")
        ttk.Button(diag_btn_frame, text="🗑️ Delete", command=self.delete_diag_file,
                  style='Delete.TButton').pack(side="left", padx=2, expand=True, fill="x")
    
    def setup_console_tab(self, parent):
        main_container = ttk.Frame(parent)
        main_container.pack(fill="both", expand=True, padx=8, pady=8)
        
        console_frame = ttk.LabelFrame(main_container, text="Communication Log", padding="5")
        console_frame.pack(fill="both", expand=True, pady=(0, 8))
        
        self.console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD,
                                                 bg='#0a0a0a', fg='#e0e0e0',
                                                 insertbackground='#e0e0e0',
                                                 font=('Consolas', 9),
                                                 bd=0, highlightthickness=0)
        self.console.pack(fill="both", expand=True)
        
        self.console.tag_config("sent", foreground="#4681a9")
        self.console.tag_config("received", foreground="#10b981")
        self.console.tag_config("error", foreground="#ef4444")
        self.console.tag_config("info", foreground="#3a7ca5")
        self.console.tag_config("success", foreground="#10b981")
        
        cmd_frame = ttk.LabelFrame(main_container, text="Command Console", padding="8")
        cmd_frame.pack(fill="x")
        
        entry_frame = ttk.Frame(cmd_frame)
        entry_frame.pack(fill="x")
        
        ttk.Label(entry_frame, text=">", font=('Consolas', 10, 'bold'), foreground="#4681a9").pack(side="left", padx=(0, 8))
        
        self.cmd_entry = ttk.Entry(entry_frame, font=('Consolas', 9))
        self.cmd_entry.pack(side="left", fill="x", expand=True)
        self.cmd_entry.bind("<Return>", self.send_command)
        
        btn_frame = ttk.Frame(cmd_frame)
        btn_frame.pack(fill="x", pady=(8, 0))
        
        ttk.Button(btn_frame, text="Send", command=self.send_command,
                  style='Primary.TButton').pack(side="left", padx=2, expand=True, fill="x")
        ttk.Button(btn_frame, text="Clear Console", command=self.clear_console,
                  style='Delete.TButton').pack(side="left", padx=2, expand=True, fill="x")
        
        quick_frame = ttk.LabelFrame(main_container, text="Quick Commands", padding="8")
        quick_frame.pack(fill="x", pady=(8, 0))
        
        commands = [
            ("📋 List Files", "list"),
            ("ℹ️ Card Info", "info"),
            ("📊 Statistics", "stats"),
            ("📝 Log Status", "logstatus"),
            ("▶️ Start Logging", "logstart"),
            ("⏸️ Stop Logging", "logstop"),
            ("🔄 Reset Stats", "reset"),
            ("⚙️ Config Status", "configstatus"),
        ]
        
        for i, (text, cmd) in enumerate(commands):
            row = i // 4
            col = i % 4
            btn = ttk.Button(quick_frame, text=text, width=18,
                           command=lambda c=cmd: self.send_command(c))
            btn.grid(row=row, column=col, padx=2, pady=2, sticky="ew")
        
        quick_frame.grid_columnconfigure(list(range(4)), weight=1)
    
    def parse_file_path(self, file_path):
        try:
            filename = os.path.basename(file_path)
            import re
            match = re.search(r'(\d{4})(\d{2})(\d{2})', filename)
            if match:
                year = match.group(1)
                month = match.group(2)
                day = match.group(3)
                return f"{year}-{month}-{day}"
        except:
            pass
        return None
    
    def extract_available_dates(self, file_list):
        dates = set()
        for entry in file_list:
            if "|" in entry:
                name = entry.split("|")[0]
                if '/' in name:
                    name = os.path.basename(name)
                date_str = self.parse_file_path(name)
                if date_str:
                    dates.add(date_str)
        return sorted(list(dates), reverse=True)
    
    def update_date_filter_combos(self):
        if not self.available_dates:
            self.year_combo['values'] = []
            self.month_combo['values'] = []
            self.day_combo['values'] = []
            return
        
        years = sorted(set([d.split('-')[0] for d in self.available_dates]), reverse=True)
        self.year_combo['values'] = years
        
        if self.current_year:
            months = sorted(set([d.split('-')[1] for d in self.available_dates 
                                if d.startswith(f"{self.current_year}-")]))
            month_names = [f"{m} ({self.get_month_name(int(m))})" for m in months]
            self.month_combo['values'] = month_names
            self.month_combo.set('')
            
            if self.current_month:
                days = sorted(set([d.split('-')[2] for d in self.available_dates 
                                  if d.startswith(f"{self.current_year}-{self.current_month}")]))
                self.day_combo['values'] = days
                self.day_combo.set('')
    
    def get_month_name(self, month_num):
        months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 
                  'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
        if 1 <= month_num <= 12:
            return months[month_num - 1]
        return ""
    
    def on_year_selected(self, event):
        year = self.year_combo.get()
        if year:
            self.current_year = year
            self.current_month = None
            self.current_day = None
            self.update_date_filter_combos()
            self.filter_info.config(text=f"Selected year: {year}")
    
    def on_month_selected(self, event):
        month_str = self.month_combo.get()
        if month_str:
            self.current_month = month_str.split(' ')[0]
            self.current_day = None
            self.update_date_filter_combos()
            self.filter_info.config(text=f"Selected: {self.current_year}-{self.current_month}")
    
    def on_day_selected(self, event):
        self.current_day = self.day_combo.get()
        if self.current_day:
            self.filter_info.config(text=f"Selected date: {self.current_year}-{self.current_month}-{self.current_day}")
    
    def apply_date_filter(self):
        """Apply date filter to files"""
        if not self.current_year or not self.current_month or not self.current_day:
            messagebox.showinfo("Info", "Please select year, month, and day")
            return
        
        selected_date = f"{self.current_year}-{self.current_month}-{self.current_day}"
        
        filtered_files = []
        filtered_diag = []
        
        for entry in self.file_list:
            if "|" in entry:
                name = entry.split("|")[0]
                if '/' in name:
                    name = os.path.basename(name)
                date_str = self.parse_file_path(name)
                if date_str == selected_date:
                    filtered_files.append(entry)
        
        for entry in self.diag_list:
            if "|" in entry:
                name = entry.split("|")[0]
                if '/' in name:
                    name = os.path.basename(name)
                date_str = self.parse_file_path(name)
                if date_str == selected_date:
                    filtered_diag.append(entry)
        
        self.display_filtered_files(filtered_files, filtered_diag, selected_date)
        
        # Also filter sessions if we have them loaded
        if hasattr(self, 'all_sessions') and self.all_sessions:
            year = self.current_year
            month = self.current_month
            day = self.current_day
            
            filtered_sessions = [s for s in self.all_sessions 
                                if s['StartTime'].startswith(f"{year}-{month}-{day}")]
            self.display_sessions(filtered_sessions)
            
            # Update status label
            self.session_status_label.config(
                text=f"Showing {len(filtered_sessions)} sessions filtered by date: {selected_date}", 
                foreground="#10b981"
            )
        else:
            self.session_status_label.config(text="No sessions loaded yet. Click Refresh History.", foreground="#f59e0b")
    
    def display_filtered_files(self, files, diag_files, date_str):
        self.file_listbox.delete(0, tk.END)
        self.diag_listbox.delete(0, tk.END)
        
        if not files:
            self.file_listbox.insert(tk.END, f"📭 No data files for {date_str}")
            self.file_listbox.itemconfig(tk.END, {'fg': '#808080'})
        else:
            for entry in files:
                if "|" in entry:
                    name, size = entry.split("|", 1)
                    try:
                        size = int(size)
                        if size < 1024:
                            size_str = f"{size} B"
                        elif size < 1024*1024:
                            size_str = f"{size/1024:.1f} KB"
                        else:
                            size_str = f"{size/(1024*1024):.2f} MB"
                        
                        if name.startswith("DATA_"):
                            self.file_listbox.insert(tk.END, f"📊 {name} ({size_str})")
                        else:
                            self.file_listbox.insert(tk.END, f"📄 {name} ({size_str})")
                    except:
                        self.file_listbox.insert(tk.END, f"📄 {entry}")
        
        if not diag_files:
            self.diag_listbox.insert(tk.END, f"📭 No diagnostic files for {date_str}")
            self.diag_listbox.itemconfig(tk.END, {'fg': '#808080'})
        else:
            for entry in diag_files:
                if "|" in entry:
                    name, size = entry.split("|", 1)
                    try:
                        size = int(size)
                        if size < 1024:
                            size_str = f"{size} B"
                        elif size < 1024*1024:
                            size_str = f"{size/1024:.1f} KB"
                        else:
                            size_str = f"{size/(1024*1024):.2f} MB"
                        self.diag_listbox.insert(tk.END, f"🔧 {name} ({size_str})")
                    except:
                        self.diag_listbox.insert(tk.END, f"🔧 {entry}")
        
        self.log(f"📅 Filtered files for {date_str}: {len(files)} data, {len(diag_files)} diagnostic", "info")
    
    def clear_date_filter(self):
        """Clear date filter and show all files and sessions"""
        self.current_year = None
        self.current_month = None
        self.current_day = None
        self.year_combo.set('')
        self.month_combo.set('')
        self.day_combo.set('')
        self.filter_info.config(text="")
        self.update_file_list()
        self.update_diag_list()
        
        # Also clear session filter and show all
        if hasattr(self, 'all_sessions') and self.all_sessions:
            self.display_sessions(self.all_sessions)
            self.session_status_label.config(text=f"Showing all {len(self.all_sessions)} sessions", foreground="#808080")
        
        self.log("🗑️ Date filter cleared", "info")
    
    def show_all_files(self):
        self.clear_date_filter()
        self.list_files()
        self.list_diag_files()
    
    def update_file_list(self):
        self.file_listbox.delete(0, tk.END)
        
        if not self.file_list:
            self.file_listbox.insert(tk.END, "📭 No files found")
            self.file_listbox.itemconfig(tk.END, {'fg': '#808080'})
            return
        
        self.available_dates = self.extract_available_dates(self.file_list)
        self.update_date_filter_combos()
        
        for entry in self.file_list:
            if "|" in entry:
                name, size = entry.split("|", 1)
                try:
                    size = int(size)
                    if size < 1024:
                        size_str = f"{size} B"
                    elif size < 1024*1024:
                        size_str = f"{size/1024:.1f} KB"
                    else:
                        size_str = f"{size/(1024*1024):.2f} MB"
                    
                    date_str = self.parse_file_path(name)
                    date_info = f" [{date_str}]" if date_str else ""
                    
                    if name.startswith("DATA_"):
                        self.file_listbox.insert(tk.END, f"📊 {name} ({size_str}){date_info}")
                    else:
                        self.file_listbox.insert(tk.END, f"📄 {name} ({size_str}){date_info}")
                except:
                    self.file_listbox.insert(tk.END, f"📄 {entry}")
            else:
                self.file_listbox.insert(tk.END, f"📄 {entry}")
        
        if self.available_dates:
            date_list = ', '.join(self.available_dates[:5])
            if len(self.available_dates) > 5:
                date_list += f" and {len(self.available_dates)-5} more"
            self.log(f"📅 Available dates: {date_list}", "info")
        else:
            self.log(f"📋 Found {len(self.file_list)} files (no date info)", "info")
    
    def update_diag_list(self):
        self.diag_listbox.delete(0, tk.END)
        
        if not self.diag_list:
            self.diag_listbox.insert(tk.END, "📭 No diagnostic files")
            self.diag_listbox.itemconfig(tk.END, {'fg': '#808080'})
            return
            
        for entry in self.diag_list:
            if "|" in entry:
                name, size = entry.split("|", 1)
                try:
                    size = int(size)
                    if size < 1024:
                        size_str = f"{size} B"
                    elif size < 1024*1024:
                        size_str = f"{size/1024:.1f} KB"
                    else:
                        size_str = f"{size/(1024*1024):.2f} MB"
                    
                    date_str = self.parse_file_path(name)
                    date_info = f" [{date_str}]" if date_str else ""
                    self.diag_listbox.insert(tk.END, f"🔧 {name} ({size_str}){date_info}")
                except:
                    self.diag_listbox.insert(tk.END, f"🔧 {entry}")
            else:
                self.diag_listbox.insert(tk.END, f"🔧 {entry}")
        
        diag_dates = self.extract_available_dates(self.diag_list)
        if diag_dates:
            self.log(f"🔧 Diagnostic files available for {len(diag_dates)} dates", "info")
    
    def scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
        self.log(f"Found {len(ports)} ports", "info")
    
    def toggle_connection(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.disconnect()
        else:
            self.connect()
    def start_heartbeat(self):
        """Start the heartbeat timer to keep UI mode active."""
        self.stop_heartbeat()  # ensure only one timer
        self._heartbeat()

    def _heartbeat(self):
        """Send a 'ping' command to keep the logger's UI mode alive."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(b"ping\n")
            # Optionally, you can log this:
            # self.log("💓 Heartbeat sent", "info")
        except Exception as e:
            print(f"Heartbeat error: {e}")
        # Schedule next heartbeat if still connected
        if self.listener_running:
            self.heartbeat_timer = self.root.after(2000, self._heartbeat)

    def stop_heartbeat(self):
        """Cancel the heartbeat timer."""
        if hasattr(self, 'heartbeat_timer') and self.heartbeat_timer:
            try:
                self.root.after_cancel(self.heartbeat_timer)
            except:
                pass
            self.heartbeat_timer = None
    
    def connect(self):
        port = self.port_combo.get()
        baudrate = int(self.baud_combo.get())
        if not port:
            messagebox.showerror("Error", "Select a port")
            return
            
        try:
            self.serial_conn = serial.Serial()
            self.serial_conn.port = port
            self.serial_conn.baudrate = baudrate
            self.serial_conn.timeout = 0.5
            self.serial_conn.write_timeout = 0.5
            self.serial_conn.bytesize = serial.EIGHTBITS
            self.serial_conn.parity = serial.PARITY_NONE
            self.serial_conn.stopbits = serial.STOPBITS_ONE
            self.serial_conn.set_buffer_size(rx_size=65536, tx_size=65536)
            
            self.serial_conn.open()
            time.sleep(1)
            self.serial_conn.reset_input_buffer()
            
            self.conn_status.config(text="🟢 Connected", foreground="#10b981")
            self.connect_btn.config(text="Disconnect", style='Danger.TButton')
            self.status_bar.config(text=f"Connected to {port} at {baudrate} baud")
            
            self.status_indicator.delete("all")
            self.status_indicator.create_oval(1, 1, 9, 9, fill='#10b981', outline='')
            
            self.listener_running = True
            threading.Thread(target=self.listener_thread, daemon=True).start()
            
            # --- NEW: Activate UI mode and start heartbeat ---
            self.send_command("ui on")
            self.start_heartbeat() 
            # -----------------------------------------------
            
            self.log(f"✅ Connected to {port} at {baudrate} baud", "success")
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def disconnect(self):
        self.listener_running = False
        # --- NEW: Stop heartbeat and release UI mode ---
        self.stop_heartbeat()
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(b"ui off\n")
            except:
                pass
        # -----------------------------------------------
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
            
        self.conn_status.config(text="🔴 Disconnected", foreground="#ef4444")
        self.sd_status.config(text="💾 Unknown", foreground="#e0e0e0")
        self.ecu_status.config(text="🚗 Unknown", foreground="#e0e0e0")
        self.connect_btn.config(text="Connect", style='Primary.TButton')
        self.status_bar.config(text="Disconnected")
        
        self.status_indicator.delete("all")
        self.status_indicator.create_oval(1, 1, 9, 9, fill='#ef4444', outline='')
        
        self.progress_bar.pack_forget()
        self.speed_label.config(text="")
        self.log("Disconnected", "info")
    
    def listener_thread(self):
        buffer = ""
        while self.listener_running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting:
                    if self.receiving_compressed or self.receiving_rle:
                        bytes_available = self.serial_conn.in_waiting
                        chunk = self.serial_conn.read(bytes_available)
                        self.binary_data.extend(chunk)
                        
                        if self.compressed_size > 0:
                            progress = (len(self.binary_data) / self.compressed_size) * 100
                            elapsed = time.time() - self.download_start_time
                            speed = (len(self.binary_data) / 1024) / elapsed if elapsed > 0 else 0
                            remaining = (self.compressed_size - len(self.binary_data)) / 1024 / speed if speed > 0 else 0
                            
                            self.root.after(0, self.update_progress, progress, speed, remaining)
                            
                            if len(self.binary_data) >= self.compressed_size:
                                elapsed = time.time() - self.download_start_time
                                speed = (self.compressed_size / 1024) / elapsed
                                self.root.after(0, self.log_message, 
                                    f"✅ Downloaded compressed {self.compressed_size/1024:.1f} KB in {elapsed:.1f}s ({speed:.1f} KB/s)", "success")
                                
                                if self.receiving_compressed:
                                    self.root.after(0, self.save_compressed_file)
                                elif self.receiving_rle:
                                    self.root.after(0, self.save_rle_file)
                                    
                                self.root.after(0, self.progress_bar.pack_forget)
                                self.root.after(0, lambda: self.speed_label.config(text=""))
                                self.root.after(0, lambda: self.progress_label.config(text=""))
                                self.receiving_compressed = False
                                self.receiving_rle = False
                            
                    elif self.receiving_binary:
                        bytes_available = self.serial_conn.in_waiting
                        bytes_to_read = min(bytes_available, self.binary_size - self.binary_received)
                        chunk = self.serial_conn.read(bytes_to_read)
                        self.binary_data.extend(chunk)
                        self.binary_received += len(chunk)
                        
                        if self.binary_received % (200 * 1024) < 4096:
                            progress = (self.binary_received / self.binary_size) * 100
                            elapsed = time.time() - self.download_start_time
                            speed = (self.binary_received / 1024) / elapsed if elapsed > 0 else 0
                            remaining = (self.binary_size - self.binary_received) / 1024 / speed if speed > 0 else 0
                            
                            self.root.after(0, self.update_progress, progress, speed, remaining)
                        
                        if self.binary_received >= self.binary_size:
                            elapsed = time.time() - self.download_start_time
                            speed = (self.binary_size / 1024) / elapsed
                            self.root.after(0, self.log_message, 
                                f"✅ Downloaded {self.binary_size/1024:.1f} KB in {elapsed:.1f}s ({speed:.1f} KB/s)", "success")
                            self.root.after(0, self.save_binary_file)
                            self.root.after(0, self.progress_bar.pack_forget)
                            self.root.after(0, lambda: self.speed_label.config(text=""))
                            self.root.after(0, lambda: self.progress_label.config(text=""))
                            self.receiving_binary = False
                            
                    else:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        text = data.decode('utf-8', errors='ignore')
                        buffer += text
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.root.after(0, self.process_line, line)
            except serial.SerialException as e:
                self.root.after(0, self.log_message, f"Serial Error: {e}", "error")
                break
            except Exception as e:
                self.root.after(0, self.log_message, f"Error: {e}", "error")
                time.sleep(0.1)
    
    def process_line(self, line):
        if line == "SD_OK":
            self.sd_status.config(text="💾 OK", foreground="#10b981")
            return
            
        if line == "SD_ERROR":
            self.sd_status.config(text="💾 Error", foreground="#ef4444")
            return
            
        if line == "CAN_OK":
            self.log("✅ CAN Bus initialized", "success")
            return
            
        if line == "CAN_ERROR":
            self.log("❌ CAN Bus error", "error")
            return
            
        if line.startswith("WIFI_"):
            self.log(f"📶 WiFi: {line[5:]}", "info")
            return
            
        if line.startswith("IP_"):
            self.log(f"🌐 IP: {line[3:]}", "info")
            return
            
        if line == "LOGGER_READY":
            self.log("✅ Logger Ready", "success")
            return
            
        if line == "LOGGING_STARTED":
            self.log("📝 Logging started", "success")
            return
            
        if line == "LOGGING_STOPPED":
            self.log("⏸️ Logging stopped", "info")
            return
            
        if line.startswith("ECU State:"):
            self.ecu_status.config(text=f"🚗 {line.split(':')[1].strip()}", foreground="#10b981")
            self.log(line, "info")
            return
            
        if line.startswith("SESSION_INFO:"):
            parts = line.split(":")[1].split(",")
            if len(parts) >= 4:
                self.current_session = int(parts[0])
                self.current_file = int(parts[1])
                self.session_records = int(parts[2])
                self.file_records = int(parts[3])
                self.update_session_display()
            return
            
        if line.startswith("NEW_DATA_FILE:"):
            filename = line.split(":", 1)[1]
            self.log(f"📁 New data file: {filename}", "info")
            self.root.after(1000, self.list_files)
            return
            
        if line.startswith("NEW_DIAG_FILE:"):
            filename = line.split(":", 1)[1]
            self.log(f"🔧 New diagnostic file: {filename}", "info")
            self.root.after(1000, self.list_diag_files)
            return
        
        # ========== ADD THESE SESSION HISTORY HANDLERS ==========
        if line == "SESSION_HISTORY_BEGIN":
            self.in_session_history = True
            self.session_history_list = []
            self.log("📊 Loading session history...", "info")
            return
            
        if line == "SESSION_HISTORY_END":
            self.in_session_history = False
            self.log(f"📊 Received {len(self.session_history_list)} session records", "info")
            sessions = self.parse_session_history(self.session_history_list)
            self.update_session_tree(sessions)
            return
        # =======================================================
            
        if line.startswith("FILE_RLE:"):
            parts = line.split(":", 3)
            self.original_size = int(parts[1])
            self.compressed_size = int(parts[2])
            self.current_filename = parts[3].replace("/", "")
            
            self.receiving_rle = True
            self.binary_data = bytearray()
            self.download_start_time = time.time()
            
            self.progress_bar['maximum'] = 100
            self.progress_bar['value'] = 0
            self.progress_label.config(text="Downloading...")
            self.progress_bar.pack(side="left", fill="x", expand=True)
            
            ratio = (self.compressed_size / self.original_size) * 100
            self.log(f"📥 Downloading RLE compressed {self.current_filename} "
                f"({self.original_size/1024:.1f} KB → {self.compressed_size/1024:.1f} KB, {ratio:.1f}%)", "info")
            return
            
        if line.startswith("FILE_COMPRESSED:"):
            parts = line.split(":", 3)
            self.original_size = int(parts[1])
            self.compressed_size = int(parts[2])
            self.current_filename = parts[3].replace("/", "")
            
            self.receiving_compressed = True
            self.binary_data = bytearray()
            self.download_start_time = time.time()
            
            self.progress_bar['maximum'] = 100
            self.progress_bar['value'] = 0
            self.progress_label.config(text="Downloading...")
            self.progress_bar.pack(side="left", fill="x", expand=True)
            
            ratio = (self.compressed_size / self.original_size) * 100
            self.log(f"📥 Downloading compressed {self.current_filename} "
                f"({self.original_size/1024:.1f} KB → {self.compressed_size/1024:.1f} KB, {ratio:.1f}%)", "info")
            return
            
        if line.startswith("FILE_BEGIN:"):
            parts = line.split(":", 2)
            self.binary_size = int(parts[1])
            self.current_filename = parts[2].replace("/", "")
            self.receiving_binary = True
            self.binary_received = 0
            self.binary_data = bytearray()
            self.download_start_time = time.time()
            
            self.progress_bar['maximum'] = 100
            self.progress_bar['value'] = 0
            self.progress_label.config(text="Downloading...")
            self.progress_bar.pack(side="left", fill="x", expand=True)
            
            self.log(f"📥 Downloading {self.current_filename} ({self.binary_size/1024:.1f} KB)...", "info")
            return
            
        if line == "FILE_END":
            return
            
        if line == "FILE_LIST_BEGIN":
            self.in_file_list = True
            self.file_list = []
            return
            
        if line == "FILE_LIST_END":
            self.in_file_list = False
            self.update_file_list()
            return
            
        if line == "DIAG_LIST_BEGIN":
            self.in_diag_list = True
            self.diag_list = []
            return
            
        if line == "DIAG_LIST_END":
            self.in_diag_list = False
            self.update_diag_list()
            return
            
        if line == "SD_CARD_INFO_BEGIN":
            self.in_card_info = True
            self.card_info = []
            return
            
        if line == "SD_CARD_INFO_END":
            self.in_card_info = False
            self.show_card_info()
            return
            
        if line == "LOG_STATUS_BEGIN":
            self.in_log_status = True
            self.log("📊 Log Status:", "info")
            return
            
        if line == "LOG_STATUS_END":
            self.in_log_status = False
            return
            
        if line == "SESSION_SUMMARY_BEGIN":
            self.in_session_summary = True
            self.log("📋 Session Summary:", "info")
            return
            
        if line == "SESSION_SUMMARY_END":
            self.in_session_summary = False
            return
            
        if line == "LIVE_DATA_BEGIN":
            self.in_live_data = True
            return
            
        if line == "LIVE_DATA_END":
            self.in_live_data = False
            return
            
        if line == "STATS_BEGIN":
            self.in_stats = True
            self.log("📊 Statistics:", "info")
            return
            
        if line == "STATS_END":
            self.in_stats = False
            return
            
        # ========== MODIFY THIS SECTION - ADD session history collection ==========
        if self.in_session_history:
            self.session_history_list.append(line)
            # Optional debug: self.log(f"  Session line: {line[:80]}...", "info")
        elif self.in_file_list and "|" in line:
            self.file_list.append(line)
        elif self.in_diag_list and "|" in line:
            self.diag_list.append(line)
        elif self.in_card_info:
            self.card_info.append(line)
        elif self.in_live_data:
            self.log(f"  {line}", "received")
        elif self.in_stats:
            if ":" in line:
                key, value = line.split(":", 1)
                self.log(f"  {key}: {value}", "info")
            else:
                self.log(f"  {line}", "info")
        elif self.in_log_status:
            if ":" in line:
                parts = line.split(":", 1)
                self.log(f"  {parts[0]}: {parts[1]}", "info")
                if "Session:" in line:
                    self.current_session = int(line.split(":")[1].strip())
                elif "File sequence:" in line:
                    self.current_file = int(line.split(":")[1].strip())
                elif "Session records:" in line:
                    self.session_records = int(line.split(":")[1].strip())
                elif "File records:" in line:
                    self.file_records = int(line.split(":")[1].strip())
                self.update_session_display()
            else:
                self.log(f"  {line}", "info")
        elif self.in_session_summary:
            self.log(f"  {line}", "info")
        else:
            self.log(line, "received")
    
    def update_progress(self, progress, speed, remaining):
        self.progress_bar['value'] = progress
        self.speed_label.config(text=f"{speed:.1f} KB/s | {remaining:.0f}s")
        self.root.update_idletasks()
    
    def update_session_display(self):
        self.session_status.config(text=f"📊 S:{self.current_session}/F:{self.current_file}/R:{self.session_records}",
                                   foreground="#10b981")
    
    def refresh_all(self):
        """Refresh all data from device"""
        self.list_files()
        self.list_diag_files()
        self.refresh_session_history()
        self.session_status_label.config(text="Ready", foreground="#808080")
    
    def delete_diag_file(self):
        sel = self.diag_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info", "Select a diagnostic file")
            return
            
        entry = self.diag_listbox.get(sel[0])
        if "📭" in entry:
            return
            
        if "🔧" in entry:
            filename = entry.split(" ", 1)[1].split(" (")[0]
        else:
            filename = entry.split(" (")[0]
            
        if messagebox.askyesno("Confirm", f"Delete {filename}?"):
            self.send_command(f"delete {filename}")
            self.log(f"🗑️ Deleting diagnostic: {filename}", "info")
    
    def save_rle_file(self):
        if not self.binary_data:
            return
            
        try:
            decompressed = bytearray()
            i = 0
            data = self.binary_data
            
            while i < len(data):
                if data[i] == 0xFC and i + 2 < len(data):
                    run_length = data[i + 1]
                    value = data[i + 2]
                    decompressed.extend([value] * run_length)
                    i += 3
                else:
                    decompressed.append(data[i])
                    i += 1
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            suggested = f"{self.current_filename}_{timestamp}.csv"
            
            filename = filedialog.asksaveasfilename(
                initialfile=suggested,
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )
            
            if filename:
                with open(filename, 'wb') as f:
                    f.write(decompressed)
                
                ratio = (len(self.binary_data) / self.original_size) * 100
                self.log(f"✅ Saved: {os.path.basename(filename)} "
                       f"({self.original_size/1024:.1f} KB, compressed {ratio:.1f}%)", "success")
                
        except Exception as e:
            self.log(f"❌ RLE decompression failed: {e}", "error")
            filename = filedialog.asksaveasfilename(
                initialfile=self.current_filename + ".rle",
                defaultextension=".bin"
            )
            if filename:
                with open(filename, 'wb') as f:
                    f.write(self.binary_data)
                self.log(f"✅ Saved RLE data: {os.path.basename(filename)}", "success")
    
    def save_compressed_file(self):
        if not self.binary_data:
            return
            
        try:
            decompressed_data = zlib.decompress(self.binary_data)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            suggested = f"{self.current_filename}_{timestamp}.csv"
            
            filename = filedialog.asksaveasfilename(
                initialfile=suggested,
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )
            
            if filename:
                with open(filename, 'wb') as f:
                    f.write(decompressed_data)
                
                ratio = (len(self.binary_data) / self.original_size) * 100
                self.log(f"✅ Saved: {os.path.basename(filename)} "
                       f"({self.original_size/1024:.1f} KB, compressed {ratio:.1f}%)", "success")
                
        except zlib.error as e:
            self.log(f"❌ Decompression failed: {e}", "error")
            filename = filedialog.asksaveasfilename(
                initialfile=self.current_filename + ".compressed",
                defaultextension=".bin"
            )
            if filename:
                with open(filename, 'wb') as f:
                    f.write(self.binary_data)
                self.log(f"✅ Saved compressed data: {os.path.basename(filename)}", "success")
    
    def save_binary_file(self):
        if not self.binary_data:
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suggested = f"{self.current_filename}_{timestamp}.csv"
            
        filename = filedialog.asksaveasfilename(
            initialfile=suggested,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filename:
            with open(filename, 'wb') as f:
                f.write(self.binary_data)
            self.log(f"✅ Saved: {os.path.basename(filename)} ({len(self.binary_data)/1024:.1f} KB)", "success")
    
    def show_card_info(self):
        if self.card_info:
            self.log("📊 SD Card Info:", "info")
            for line in self.card_info:
                self.log(f"  {line}", "info")
    
    def send_command(self, cmd=None):
        if cmd is None:
            cmd = self.cmd_entry.get().strip()
            if cmd:
                self.cmd_entry.delete(0, tk.END)
        else:
            pass
            
        if cmd and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write((cmd + '\n').encode())
                self.console.insert(tk.END, f"→ {cmd}\n", "sent")
                self.console.see(tk.END)
                self.status_bar.config(text=f"Sent: {cmd}")
                
                if cmd in ["delete", "logrotate"]:
                    self.root.after(1000, self.list_files)
                    self.root.after(1000, self.list_diag_files)
                    
            except Exception as e:
                self.log_message(f"Error sending command: {e}", "error")
        elif not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Warning", "Not connected")
    
    def list_files(self):
        self.send_command("list")
    
    def list_diag_files(self):
        self.send_command("listdiag")
    
    def download_file(self):
        sel = self.file_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info", "Select a file")
            return
            
        entry = self.file_listbox.get(sel[0])
        if "📭" in entry:
            return
            
        display_text = entry.split(" [")[0]
        if "📊" in display_text or "📄" in display_text:
            filename = display_text.split(" ", 1)[1].split(" (")[0]
        else:
            filename = display_text.split(" (")[0]
            
        self.send_command(f"send {filename}")
        self.log(f"📥 Downloading: {filename}", "info")
    
    def download_diag_file(self):
        sel = self.diag_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info", "Select a diagnostic file")
            return
            
        entry = self.diag_listbox.get(sel[0])
        if "📭" in entry:
            return
            
        display_text = entry.split(" [")[0]
        if "🔧" in display_text:
            filename = display_text.split(" ", 1)[1].split(" (")[0]
        else:
            filename = display_text.split(" (")[0]
            
        self.send_command(f"send {filename}")
        self.log(f"🔧 Downloading diagnostic: {filename}", "info")
    
    def delete_file(self):
        sel = self.file_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info", "Select a file")
            return
            
        entry = self.file_listbox.get(sel[0])
        if "📭" in entry:
            return
            
        display_text = entry.split(" [")[0]
        if "📊" in display_text or "📄" in display_text:
            filename = display_text.split(" ", 1)[1].split(" (")[0]
        else:
            filename = display_text.split(" (")[0]
            
        if messagebox.askyesno("Confirm", f"Delete {filename}?"):
            self.send_command(f"delete {filename}")
            self.log(f"🗑️ Deleting: {filename}", "info")
    
    def get_card_info(self):
        self.send_command("info")
    
    def log(self, msg, tag="received"):
        self.console.insert(tk.END, msg + "\n", tag)
        self.console.see(tk.END)
    
    def log_message(self, msg, tag="info"):
        self.console.insert(tk.END, msg + "\n", tag)
        self.console.see(tk.END)
    
    def clear_console(self):
        self.console.delete(1.0, tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = BMSLoggerUI(root)
    root.mainloop()