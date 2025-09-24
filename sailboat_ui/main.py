#!/usr/bin/env python3
"""
Sailboat Control UI
A simple GUI application to control the integrated sailboat autopilot system
via serial communication.
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import queue
from dataclasses import dataclass
from typing import Optional

@dataclass
class SystemStatus:
    """Data class to hold system status information"""
    rudder_percentage: float = 0.0
    rudder_pwm: int = 0
    motor_percentage: float = 0.0
    motor_voltage: float = 0.0
    current_angle: float = 0.0
    target_angle: float = 0.0
    pid_enabled: bool = False
    pid_kp: float = 0.0
    pid_ki: float = 0.0
    pid_kd: float = 0.0

class SerialManager:
    """Manages serial communication with the Arduino"""
    
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.read_thread: Optional[threading.Thread] = None
        self.stop_reading = False
        self.message_queue = queue.Queue()
        
    def get_available_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port: str, baudrate: int = 9600) -> bool:
        """Connect to serial port"""
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.is_connected = True
            
            # Start reading thread
            self.stop_reading = False
            self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
            self.read_thread.start()
            
            return True
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.stop_reading = True
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
    
    def send_command(self, command: str) -> bool:
        """Send command to Arduino"""
        if not self.is_connected or not self.serial_port:
            return False
        
        try:
            command_bytes = (command + '\n').encode('utf-8')
            self.serial_port.write(command_bytes)
            self.serial_port.flush()
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def _read_serial(self):
        """Read from serial port in background thread"""
        while not self.stop_reading and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.message_queue.put(line)
            except Exception as e:
                print(f"Serial read error: {e}")
                break
            time.sleep(0.01)
    
    def get_messages(self):
        """Get all queued messages"""
        messages = []
        while not self.message_queue.empty():
            try:
                messages.append(self.message_queue.get_nowait())
            except queue.Empty:
                break
        return messages

class SailboatUI:
    """Main UI class for sailboat control"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Sailboat Control Interface")
        self.root.geometry("800x700")
        
        self.serial_manager = SerialManager()
        self.status = SystemStatus()
        
        self.create_widgets()
        self.update_display()
        
        # Handle window closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        """Create and layout all GUI widgets"""
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=10)
        self.port_combo.pack(side="left", padx=(5, 10))
        
        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=(0, 10))
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left")
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side="right")
        
        # Control Notebook
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Manual Control Tab
        manual_frame = ttk.Frame(notebook)
        notebook.add(manual_frame, text="Manual Control")
        self.create_manual_controls(manual_frame)
        
        # PID Control Tab
        pid_frame = ttk.Frame(notebook)
        notebook.add(pid_frame, text="PID Control")
        self.create_pid_controls(pid_frame)
        
        # Debug Tab
        debug_frame = ttk.Frame(notebook)
        notebook.add(debug_frame, text="Debug")
        self.create_debug_controls(debug_frame)
        
        # Initialize ports
        self.refresh_ports()
    
    def create_manual_controls(self, parent):
        """Create manual control widgets"""
        
        # Rudder Control
        rudder_frame = ttk.LabelFrame(parent, text="Rudder Control", padding="10")
        rudder_frame.pack(fill="x", padx=10, pady=5)
        
        # Control mode selection
        mode_frame = ttk.Frame(rudder_frame)
        mode_frame.pack(fill="x", pady=(0, 10))
        
        self.rudder_mode_var = tk.StringVar(value="manual")
        ttk.Radiobutton(mode_frame, text="Manual Speed Control", 
                       variable=self.rudder_mode_var, value="manual",
                       command=self.update_rudder_mode).pack(side="left")
        ttk.Radiobutton(mode_frame, text="Angle Control (PID)", 
                       variable=self.rudder_mode_var, value="pid",
                       command=self.update_rudder_mode).pack(side="left", padx=(20, 0))
        
        # Immediate effect option
        self.immediate_effect_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(mode_frame, text="Immediate Effect", 
                       variable=self.immediate_effect_var).pack(side="right")
        
        # Manual speed control
        self.manual_frame = ttk.Frame(rudder_frame)
        self.manual_frame.pack(fill="x", pady=5)
        
        ttk.Label(self.manual_frame, text="Rudder Speed (-100% to +100%):").pack()
        
        self.rudder_var = tk.DoubleVar(value=0)
        self.rudder_scale = ttk.Scale(self.manual_frame, from_=-100, to=100, 
                                     variable=self.rudder_var, orient="horizontal",
                                     command=self.on_rudder_change)
        self.rudder_scale.pack(fill="x", pady=5)
        
        rudder_btn_frame = ttk.Frame(self.manual_frame)
        rudder_btn_frame.pack(fill="x", pady=5)
        
        self.rudder_value_label = ttk.Label(rudder_btn_frame, text="0%")
        self.rudder_value_label.pack(side="left")
        
        ttk.Button(rudder_btn_frame, text="Set Rudder", 
                  command=self.set_rudder).pack(side="right", padx=(0, 5))
        ttk.Button(rudder_btn_frame, text="Center", 
                  command=self.center_rudder).pack(side="right", padx=(0, 5))
        
        # Angle control (PID)
        self.pid_frame = ttk.Frame(rudder_frame)
        
        ttk.Label(self.pid_frame, text="Target Angle (-45° to +45°):").pack()
        
        self.angle_var = tk.DoubleVar(value=0)
        self.angle_scale = ttk.Scale(self.pid_frame, from_=-45, to=45, 
                                    variable=self.angle_var, orient="horizontal",
                                    command=self.on_angle_change)
        self.angle_scale.pack(fill="x", pady=5)
        
        angle_btn_frame = ttk.Frame(self.pid_frame)
        angle_btn_frame.pack(fill="x", pady=5)
        
        self.angle_value_label = ttk.Label(angle_btn_frame, text="0°")
        self.angle_value_label.pack(side="left")
        
        self.pid_enabled_var = tk.BooleanVar(value=False)
        pid_check = ttk.Checkbutton(angle_btn_frame, text="Enable PID", 
                                   variable=self.pid_enabled_var, 
                                   command=self.toggle_pid)
        pid_check.pack(side="left", padx=(20, 0))
        
        ttk.Button(angle_btn_frame, text="Set Angle", 
                  command=self.set_angle).pack(side="right", padx=(0, 5))
        ttk.Button(angle_btn_frame, text="Center", 
                  command=self.center_angle).pack(side="right", padx=(0, 5))
        
        # PID Settings Expander
        self.create_pid_expander(self.pid_frame)
        
        # Initialize mode
        self.update_rudder_mode()
        
        # Motor Control
        motor_frame = ttk.LabelFrame(parent, text="Motor Control", padding="10")
        motor_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(motor_frame, text="Motor Speed (0% to 100%):").pack()
        
        self.motor_var = tk.DoubleVar(value=0)
        self.motor_scale = ttk.Scale(motor_frame, from_=0, to=100, 
                                    variable=self.motor_var, orient="horizontal",
                                    command=self.on_motor_change)
        self.motor_scale.pack(fill="x", pady=5)
        
        motor_btn_frame = ttk.Frame(motor_frame)
        motor_btn_frame.pack(fill="x", pady=5)
        
        self.motor_value_label = ttk.Label(motor_btn_frame, text="0%")
        self.motor_value_label.pack(side="left")
        
        ttk.Button(motor_btn_frame, text="Set Motor", 
                  command=self.set_motor).pack(side="right", padx=(0, 5))
        ttk.Button(motor_btn_frame, text="Stop", 
                  command=self.stop_motor).pack(side="right", padx=(0, 5))
        
        # Emergency Stop
        emergency_frame = ttk.Frame(parent)
        emergency_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Button(emergency_frame, text="EMERGENCY STOP", 
                  command=self.emergency_stop, 
                  style="Emergency.TButton").pack(expand=True)
    
    def create_pid_expander(self, parent):
        """Create expandable PID settings section"""
        
        # Expander button
        expander_frame = ttk.Frame(parent)
        expander_frame.pack(fill="x", pady=(10, 0))
        
        self.pid_expanded = tk.BooleanVar(value=False)
        self.pid_expander_btn = ttk.Button(expander_frame, text="▶ PID Settings", 
                                          command=self.toggle_pid_expander)
        self.pid_expander_btn.pack(side="left")
        
        ttk.Button(expander_frame, text="Get Status", 
                  command=self.get_status).pack(side="right")
        
        # Expandable PID settings frame
        self.pid_settings_frame = ttk.Frame(parent)
        
        # PID Parameters
        params_frame = ttk.LabelFrame(self.pid_settings_frame, text="PID Tuning", padding="10")
        params_frame.pack(fill="x", pady=(5, 0))
        
        param_entry_frame = ttk.Frame(params_frame)
        param_entry_frame.pack(fill="x")
        
        ttk.Label(param_entry_frame, text="Kp:").grid(row=0, column=0, sticky="w", padx=(0, 5))
        self.kp_var = tk.DoubleVar(value=0.2)
        kp_entry = ttk.Entry(param_entry_frame, textvariable=self.kp_var, width=8)
        kp_entry.grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(param_entry_frame, text="Ki:").grid(row=0, column=2, sticky="w", padx=(0, 5))
        self.ki_var = tk.DoubleVar(value=0.01)
        ki_entry = ttk.Entry(param_entry_frame, textvariable=self.ki_var, width=8)
        ki_entry.grid(row=0, column=3, padx=(0, 10))
        
        ttk.Label(param_entry_frame, text="Kd:").grid(row=0, column=4, sticky="w", padx=(0, 5))
        self.kd_var = tk.DoubleVar(value=0.05)
        kd_entry = ttk.Entry(param_entry_frame, textvariable=self.kd_var, width=8)
        kd_entry.grid(row=0, column=5, padx=(0, 10))
        
        ttk.Button(param_entry_frame, text="Set PID", 
                  command=self.set_pid_params).grid(row=0, column=6)
        
        # PID Presets
        presets_frame = ttk.Frame(params_frame)
        presets_frame.pack(fill="x", pady=(10, 0))
        
        ttk.Label(presets_frame, text="Presets:").pack(side="left")
        ttk.Button(presets_frame, text="Gentle (P-only)", 
                  command=lambda: self.load_preset(0.1, 0.0, 0.0)).pack(side="left", padx=5)
        ttk.Button(presets_frame, text="Default", 
                  command=lambda: self.load_preset(0.2, 0.01, 0.05)).pack(side="left", padx=5)
        ttk.Button(presets_frame, text="Ultra-smooth", 
                  command=lambda: self.load_preset(0.05, 0.0, 0.02)).pack(side="left", padx=5)
        
        # Initially collapsed
        self.update_pid_expander()
    
    def create_pid_controls(self, parent):
        """Create PID control widgets (simplified since we moved main controls to manual tab)"""
        
        # Status and monitoring
        status_frame = ttk.LabelFrame(parent, text="PID Status & Monitoring", padding="10")
        status_frame.pack(fill="x", padx=10, pady=5)
        
        status_btn_frame = ttk.Frame(status_frame)
        status_btn_frame.pack(fill="x")
        
        ttk.Button(status_btn_frame, text="Get System Status", 
                  command=self.get_status).pack(side="left", padx=(0, 10))
        ttk.Button(status_btn_frame, text="Show Current PID Values", 
                  command=self.show_pid_values).pack(side="left", padx=(0, 10))
        ttk.Button(status_btn_frame, text="Enable Analog Monitoring", 
                  command=self.enable_analog_monitoring).pack(side="left")
        
        # Advanced PID settings (always visible in PID tab)
        advanced_frame = ttk.LabelFrame(parent, text="Advanced PID Configuration", padding="10")
        advanced_frame.pack(fill="x", padx=10, pady=5)
        
        # Angle mapping
        mapping_frame = ttk.Frame(advanced_frame)
        mapping_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(mapping_frame, text="Angle Mapping:").pack(side="left")
        ttk.Label(mapping_frame, text="ADC Min:").pack(side="left", padx=(20, 5))
        self.adc_min_var = tk.IntVar(value=285)
        ttk.Entry(mapping_frame, textvariable=self.adc_min_var, width=6).pack(side="left", padx=(0, 10))
        
        ttk.Label(mapping_frame, text="ADC Max:").pack(side="left", padx=(0, 5))
        self.adc_max_var = tk.IntVar(value=611)
        ttk.Entry(mapping_frame, textvariable=self.adc_max_var, width=6).pack(side="left", padx=(0, 10))
        
        ttk.Label(mapping_frame, text="Angle Min:").pack(side="left", padx=(0, 5))
        self.angle_min_var = tk.DoubleVar(value=-45.0)
        ttk.Entry(mapping_frame, textvariable=self.angle_min_var, width=6).pack(side="left", padx=(0, 10))
        
        ttk.Label(mapping_frame, text="Angle Max:").pack(side="left", padx=(0, 5))
        self.angle_max_var = tk.DoubleVar(value=45.0)
        ttk.Entry(mapping_frame, textvariable=self.angle_max_var, width=6).pack(side="left", padx=(0, 10))
        
        ttk.Button(mapping_frame, text="Set Mapping", 
                  command=self.set_angle_mapping).pack(side="right")
    
    def create_debug_controls(self, parent):
        """Create debug interface widgets"""
        
        # Command input
        cmd_frame = ttk.LabelFrame(parent, text="Send Command", padding="10")
        cmd_frame.pack(fill="x", padx=10, pady=5)
        
        self.command_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.command_var, width=40)
        cmd_entry.pack(side="left", fill="x", expand=True, padx=(0, 10))
        cmd_entry.bind("<Return>", lambda e: self.send_debug_command())
        
        ttk.Button(cmd_frame, text="Send", 
                  command=self.send_debug_command).pack(side="right")
        
        # Quick commands
        quick_frame = ttk.LabelFrame(parent, text="Quick Commands", padding="10")
        quick_frame.pack(fill="x", padx=10, pady=5)
        
        quick_buttons = [
            ("status", "Status"),
            ("help", "Help"),
            ("analog", "Read Analog"),
            ("tune", "Show PID"),
            ("stop", "Stop All")
        ]
        
        for cmd, label in quick_buttons:
            ttk.Button(quick_frame, text=label, 
                      command=lambda c=cmd: self.send_quick_command(c)).pack(side="left", padx=5)
        
        # Serial output
        output_frame = ttk.LabelFrame(parent, text="Serial Output", padding="10")
        output_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.debug_output = scrolledtext.ScrolledText(output_frame, height=15, wrap=tk.WORD)
        self.debug_output.pack(fill="both", expand=True)
        
        # Output controls
        output_ctrl_frame = ttk.Frame(output_frame)
        output_ctrl_frame.pack(fill="x", pady=(5, 0))
        
        ttk.Button(output_ctrl_frame, text="Clear", 
                  command=self.clear_debug).pack(side="left")
        
        self.auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(output_ctrl_frame, text="Auto-scroll", 
                       variable=self.auto_scroll_var).pack(side="right")
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = self.serial_manager.get_available_ports()
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
    
    def toggle_connection(self):
        """Toggle serial connection"""
        if not self.serial_manager.is_connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
            
            if self.serial_manager.connect(port, 115200):
                self.status_label.config(text="Connected", foreground="green")
                self.connect_btn.config(text="Disconnect")
            else:
                messagebox.showerror("Error", f"Failed to connect to {port}")
        else:
            self.serial_manager.disconnect()
            self.status_label.config(text="Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
    
    def set_rudder(self):
        """Send rudder command"""
        value = int(self.rudder_var.get())
        self.serial_manager.send_command(f"rudder {value}")
    
    def center_rudder(self):
        """Center the rudder"""
        self.rudder_var.set(0)
        self.set_rudder()
    
    def set_motor(self):
        """Send motor command"""
        value = int(self.motor_var.get())
        self.serial_manager.send_command(f"motor {value}")
    
    def stop_motor(self):
        """Stop the motor"""
        self.motor_var.set(0)
        self.serial_manager.send_command("motor 0")
    
    def emergency_stop(self):
        """Emergency stop all systems"""
        self.serial_manager.send_command("stop")
        self.rudder_var.set(0)
        self.motor_var.set(0)
        self.pid_enabled_var.set(False)
    
    def set_angle(self):
        """Send angle command"""
        value = self.angle_var.get()
        self.serial_manager.send_command(f"angle {value}")
    
    def toggle_pid(self):
        """Toggle PID control"""
        if self.pid_enabled_var.get():
            self.serial_manager.send_command("pid on")
        else:
            self.serial_manager.send_command("pid off")
    
    def get_status(self):
        """Request system status"""
        self.serial_manager.send_command("status")
    
    def set_pid_params(self):
        """Set PID parameters"""
        kp = self.kp_var.get()
        ki = self.ki_var.get()
        kd = self.kd_var.get()
        self.serial_manager.send_command(f"tune {kp} {ki} {kd}")
    
    def load_preset(self, kp, ki, kd):
        """Load PID preset values"""
        self.kp_var.set(kp)
        self.ki_var.set(ki)
        self.kd_var.set(kd)
        self.set_pid_params()
    
    def send_debug_command(self):
        """Send custom debug command"""
        command = self.command_var.get().strip()
        if command:
            self.serial_manager.send_command(command)
            self.command_var.set("")
    
    def send_quick_command(self, command):
        """Send quick command"""
        self.serial_manager.send_command(command)
    
    def clear_debug(self):
        """Clear debug output"""
        self.debug_output.delete(1.0, tk.END)
    
    def update_rudder_mode(self):
        """Update rudder control mode display"""
        if self.rudder_mode_var.get() == "manual":
            self.manual_frame.pack(fill="x", padx=10, pady=5)
            self.pid_frame.pack_forget()
        else:
            self.manual_frame.pack_forget()
            self.pid_frame.pack(fill="x", padx=10, pady=5)
    
    def toggle_pid_expander(self):
        """Toggle PID settings expander"""
        self.pid_expanded.set(not self.pid_expanded.get())
        self.update_pid_expander()
    
    def update_pid_expander(self):
        """Update PID expander display state"""
        if self.pid_expanded.get():
            self.pid_expander_btn.config(text="▼ PID Settings")
            self.pid_settings_frame.pack(fill="x", padx=10, pady=5)
        else:
            self.pid_expander_btn.config(text="▶ PID Settings")
            self.pid_settings_frame.pack_forget()
    
    def on_rudder_change(self, value=None):
        """Handle rudder slider change (immediate effect)"""
        if self.immediate_effect_var.get():
            self.set_rudder()
    
    def on_motor_change(self, value=None):
        """Handle motor slider change (immediate effect)"""
        if self.immediate_effect_var.get():
            self.set_motor()
    
    def on_angle_change(self, value=None):
        """Handle angle slider change (immediate effect)"""
        if self.immediate_effect_var.get():
            self.set_angle()
    
    def center_angle(self):
        """Center the target angle"""
        self.angle_var.set(0)
        self.set_angle()
    
    def show_pid_values(self):
        """Request current PID values from Arduino"""
        self.serial_manager.send_command("pid")
    
    def enable_analog_monitoring(self):
        """Enable continuous analog monitoring"""
        self.serial_manager.send_command("analog")
    
    def set_angle_mapping(self):
        """Set angle mapping parameters"""
        adc_min = self.adc_min_var.get()
        adc_max = self.adc_max_var.get()
        angle_min = self.angle_min_var.get()
        angle_max = self.angle_max_var.get()
        self.serial_manager.send_command(f"map {adc_min} {adc_max} {angle_min} {angle_max}")
    
    def update_display(self):
        """Update display with current values and serial messages"""
        
        # Update value labels
        self.rudder_value_label.config(text=f"{int(self.rudder_var.get())}%")
        self.motor_value_label.config(text=f"{int(self.motor_var.get())}%")
        self.angle_value_label.config(text=f"{self.angle_var.get():.1f}°")
        
        # Process serial messages
        messages = self.serial_manager.get_messages()
        for message in messages:
            # Add timestamp and display message
            timestamp = time.strftime("%H:%M:%S")
            self.debug_output.insert(tk.END, f"[{timestamp}] {message}\n")
            
            # Auto-scroll to bottom if enabled
            if self.auto_scroll_var.get():
                self.debug_output.see(tk.END)
        
        # Schedule next update
        self.root.after(100, self.update_display)
    
    def on_closing(self):
        """Handle window closing"""
        self.serial_manager.disconnect()
        self.root.destroy()

def main():
    """Main entry point"""
    root = tk.Tk()
    
    # Configure styles
    style = ttk.Style()
    style.configure("Emergency.TButton", foreground="red", font=("TkDefaultFont", 12, "bold"))
    
    app = SailboatUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()