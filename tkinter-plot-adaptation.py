import tkinter as tk # Assuming SensorMonitorApp might pass tk.Tk for status updates
from tkinter import ttk
import numpy as np
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk # type: ignore
from matplotlib.figure import Figure
import matplotlib.dates as mdates
from datetime import datetime
import queue

# Imports needed for CAN handling
import can
import cantools
import re
import os
import sys # Optional: for error handling during DBC/CAN init

# Type hinting imports
from typing import Optional, Tuple, Callable, Any, Dict

# Import the sensor buffer implementation
from sensor_data_structure import ModularSensorBuffer

# --- Constants (Adapt from can_monitor_app.py and can_simulator.py) ---
DBC_FILE_PATH = r"resources\IOT_CAN_v4.0.dbc"  # Make sure this path is correct
KVASER_INTERFACE = "kvaser"
# Use channel 0 for virtual simulation, maybe 1 for real hardware? Make configurable?
KVASER_CHANNEL = 1 # For testing with can_simulator.py which uses channel 0
IS_VIRTUAL = True # Set True for testing with simulator

# CAN FD Bit Timing (Must match monitor and simulator)
ARBITRATION_BITRATE = 1000000
DATA_BITRATE = 4000000

class CanReaderThread(threading.Thread):
    """Reads CAN messages, decodes them, and updates the sensor buffer."""
    def __init__(self, sensor_buffer: ModularSensorBuffer, status_callback: Optional[Callable[[str, bool], None]] = None):
        super().__init__()
        self.sensor_buffer: ModularSensorBuffer = sensor_buffer
        self.status_callback: Optional[Callable[[str, bool], None]] = status_callback
        self.running: bool = False
        self.db: Optional[cantools.db.Database] = None # type: ignore
        self.bus: Optional[can.BusABC] = None # More specific type like can.interfaces.kvaser.KvaserBus could be used if only kvaser
        self.daemon: bool = True

    def _update_status(self, message: str, is_error: bool = False) -> None:
        """Safely updates the status bar via callback."""
        print(f"CAN Status: {message}")
        if self.status_callback:
            # If status_callback needs to interact with Tkinter GUI from this thread,
            # it should use root.after() or a thread-safe queue.
            # For simplicity, we assume the callback itself handles thread safety if needed.
            self.status_callback(message, is_error)

    def _load_dbc(self) -> bool:
        """Loads the DBC file."""
        try:
            if not os.path.exists(DBC_FILE_PATH):
                raise FileNotFoundError(f"DBC file not found at: {DBC_FILE_PATH}")
            self.db = cantools.db.load_file(DBC_FILE_PATH)
            self._update_status(f"DBC file '{os.path.basename(DBC_FILE_PATH)}' loaded.")
            return True
        except FileNotFoundError as e:
            self.db = None
            self._update_status(f"Error loading DBC: {e}", is_error=True)
            return False
        except cantools.db.UnsupportedDatabaseFormatError as e: # More specific error
            self.db = None
            self._update_status(f"Error parsing DBC file (format error): {e}", is_error=True)
            return False
        except Exception as e: # General fallback
            self.db = None
            self._update_status(f"Error loading DBC: {e}", is_error=True)
            return False

    def _init_can_bus(self) -> bool:
        """Initializes the CAN bus."""
        try:
            bit_timing_fd = can.BitTimingFd(
                f_clock=80_000_000,
                nom_brp=1, nom_tseg1=64, nom_tseg2=15, nom_sjw=2,
                data_brp=1, data_tseg1=13, data_tseg2=6, data_sjw=6
            )

            self.bus = can.interface.Bus(
                interface=KVASER_INTERFACE,
                channel=str(KVASER_CHANNEL), # Ensure channel is a string for python-can
                fd=True,
                bitrate=ARBITRATION_BITRATE,
                data_bitrate=DATA_BITRATE,
                timing=bit_timing_fd,
                is_virtual=IS_VIRTUAL
            )
            self._update_status(f"Connected to {KVASER_INTERFACE} channel {KVASER_CHANNEL} (FD Mode {'Virtual' if IS_VIRTUAL else ''}). Monitoring started.")
            return True
        except can.CanInterfaceNotImplementedError as e:
            self.bus = None
            self._update_status(f"Error: '{KVASER_INTERFACE}' interface not implemented or drivers missing: {e}", is_error=True)
            return False
        except can.CanInitializationError as e:
            self.bus = None
            self._update_status(f"Error initializing Kvaser interface: {e}", is_error=True)
            return False
        except Exception as e: # General fallback
            self.bus = None
            self._update_status(f"Failed to initialize CAN bus: {e}", is_error=True)
            return False

    def parse_signal_name(self, signal_name: str) -> Optional[Tuple[str, int, int]]:
        """
        Parses signal names like 'UCellBattPwrHi_1_1' or 'TCellBattEgyHi_12_6'.
        Returns (data_type_str, module_idx, cell_idx) or None.
        'data_type_str' will be "voltage" or "temperature".
        """
        match = re.match(r"(U|T)CellBatt(?:Pwr|Egy)Hi_(\d+)_(\d+)", signal_name)
        if match:
            data_type_char, module_str, cell_str = match.groups()
            data_type_map: Dict[str, str] = {"U": "voltage", "T": "temperature"}
            data_type: Optional[str] = data_type_map.get(data_type_char)

            if data_type:
                try:
                    module_idx: int = int(module_str) - 1
                    cell_idx: int = int(cell_str) - 1
                    return data_type, module_idx, cell_idx
                except ValueError:
                    # This should ideally not happen due to \d+ in regex
                    print(f"Warning: Could not parse module/cell from signal: {signal_name}")
                    return None
        return None

    def run(self) -> None:
        """Main thread loop to read and process CAN messages."""
        if not self._load_dbc() or not self._init_can_bus():
            self.running = False
            self._update_status("CAN Reader Thread: Initialization failed. Thread stopping.", is_error=True)
            return

        self.running = True
        self._update_status("CAN Reader Thread: Running.")
        while self.running and self.bus:
            try:
                msg: Optional[can.Message] = self.bus.recv(timeout=1.0)
                if msg and self.db: # Ensure self.db is not None
                    try:
                        # cantools expects data to be bytes or bytearray
                        decoded_signals: Dict[str, Any] = self.db.decode_message(msg.arbitration_id, bytes(msg.data))
                        timestamp_ms: float = msg.timestamp * 1000.0 # CAN timestamp is in seconds

                        for signal_name, value in decoded_signals.items():
                            parse_result: Optional[Tuple[str, int, int]] = self.parse_signal_name(signal_name)
                            if parse_result:
                                data_type: str
                                module_id: int
                                cell_id: int
                                data_type, module_id, cell_id = parse_result

                                if 0 <= module_id < 5: # Limit to first 5 modules
                                    # Ensure value is float, cantools usually returns numeric types
                                    try:
                                        numeric_value: float = float(value)
                                        if data_type == "voltage":
                                            self.sensor_buffer.update_voltage(module_id, cell_id, timestamp_ms, numeric_value)
                                        elif data_type == "temperature":
                                            self.sensor_buffer.update_temperature(module_id, cell_id, timestamp_ms, numeric_value)
                                    except (ValueError, TypeError) as e:
                                        print(f"Warning: Could not convert signal value '{value}' to float for {signal_name}: {e}")


                    except KeyError: # Message ID not in DBC
                        # print(f"Message ID {hex(msg.arbitration_id)} not in DBC.") # Can be noisy
                        pass
                    except cantools.db.DecodeError as decode_error: # More specific decode error
                        print(f"Decode Error for ID {hex(msg.arbitration_id)}: {decode_error}")
                    except ValueError as val_err: # Catch potential errors from bytes(msg.data) or float conversion
                        print(f"ValueError processing msg ID {hex(msg.arbitration_id)}: {val_err}")
                    except Exception as e: # General processing error
                        print(f"Error decoding/processing msg ID {hex(msg.arbitration_id)}: {e}")

            except can.CanOperationError as e:
                self._update_status(f"CAN Operation Error: {e}. Attempting to continue...", is_error=True)
                # Potentially add logic to try and reset the bus here if it's a recoverable error
                time.sleep(2)
            except Exception as e: # Catch-all for unexpected errors in the loop (e.g., bus becomes None unexpectedly)
                self._update_status(f"CAN Thread Critical Error: {e}", is_error=True)
                self.running = False

        if self.bus:
            try:
                self.bus.shutdown()
                self._update_status("CAN bus shutdown.")
            except Exception as e:
                 self._update_status(f"Error shutting down CAN bus: {e}", is_error=True)
        self._update_status(f"CAN reader thread stopped. Running state: {self.running}")

    def stop(self) -> None:
        """Signals the thread to stop."""
        self._update_status("CAN Reader Thread: Stop requested.")
        self.running = False

class SensorPlotTab:
    """
    Tkinter notebook tab with matplotlib plots for sensor data visualization.
    Shows voltage and temperature data for a specific module in separate subplots.
    """
    def __init__(self, notebook, sensor_buffer, module_id=1):
        self.sensor_buffer = sensor_buffer
        self.module_id = module_id
        self.notebook = notebook  # Store reference to notebook for active tab checking
        
        # Initialize last update time
        self.last_update_time = time.time()
        
        # Create a tab in the notebook
        self.tab = ttk.Frame(notebook)
        notebook.add(self.tab, text=f"Module Data")
        
        # Configure the tab layout
        self.tab.columnconfigure(0, weight=1)
        self.tab.rowconfigure(0, weight=1)
        
        # Create frame for module selection
        self.controls_frame = ttk.Frame(self.tab)
        self.controls_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        
        # Module selection dropdown
        ttk.Label(self.controls_frame, text="Module:").pack(side=tk.LEFT, padx=(0, 5))
        self.module_var = tk.StringVar(value=str(module_id))
        self.module_selector = ttk.Combobox(
            self.controls_frame, 
            textvariable=self.module_var,
            values=[str(i) for i in range(5)],  # 5 modules (0-4)
            width=5,
            state="readonly"
        )
        self.module_selector.pack(side=tk.LEFT, padx=(0, 10))
        self.module_selector.bind("<<ComboboxSelected>>", self.on_module_change)
        
        # Auto-update checkbox
        self.auto_update_var = tk.BooleanVar(value=True)
        self.auto_update_checkbox = ttk.Checkbutton(
            self.controls_frame,
            text="Auto update",
            variable=self.auto_update_var,
            command=self.toggle_auto_update
        )
        self.auto_update_checkbox.pack(side=tk.LEFT, padx=10)
        
        # Update interval selection
        ttk.Label(self.controls_frame, text="Update Interval:").pack(side=tk.LEFT, padx=(10, 5))
        self.update_interval_var = tk.StringVar(value="1")
        self.interval_selector = ttk.Combobox(
            self.controls_frame,
            textvariable=self.update_interval_var,
            values=["0.5", "1", "2", "5", "10"],  # Update intervals in seconds
            width=5,
            state="readonly"
        )
        self.interval_selector.pack(side=tk.LEFT, padx=(0, 10))
        self.interval_selector.bind("<<ComboboxSelected>>", self.on_interval_change)
        
        # Manual update button
        self.update_button = ttk.Button(
            self.controls_frame,
            text="Update Now",
            command=self.update_plots,
            state=tk.DISABLED
        )
        self.update_button.pack(side=tk.LEFT, padx=10)
        
        # Create matplotlib figure with two subplots
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.fig.subplots_adjust(hspace=0.3)
        
        # Create voltage subplot
        self.voltage_ax = self.fig.add_subplot(2, 1, 1)
        self.voltage_ax.set_title(f"Module {module_id} Cell Voltages")
        self.voltage_ax.set_ylabel("Voltage (V)")
        self.voltage_ax.set_ylim(-16, 16)  # Fixed Y axis for voltage
        self.voltage_ax.grid(True)
        
        # Create temperature subplot
        self.temp_ax = self.fig.add_subplot(2, 1, 2)
        self.temp_ax.set_title(f"Module {module_id} Cell Temperatures")
        self.temp_ax.set_ylabel("Temperature (K)")
        self.temp_ax.set_ylim(0, 512)  # Fixed Y axis for temperature
        self.temp_ax.grid(True)
        
        # Initialize line objects for plotting
        self.voltage_lines = []
        self.temp_lines = []
        self.initialize_plot_lines()
        
        # Create canvas for the figure and pack it
        self.canvas_frame = ttk.Frame(self.tab)
        self.canvas_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add toolbar
        self.toolbar_frame = ttk.Frame(self.tab)
        self.toolbar_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=0)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.toolbar_frame)
        self.toolbar.update()
        
        # Status bar
        self.status_frame = ttk.Frame(self.tab)
        self.status_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 5))
        self.status_var = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(self.status_frame, textvariable=self.status_var, anchor=tk.W)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Set up the update thread
        self.update_queue: queue.Queue[bool] = queue.Queue()
        self.running = True
        self.update_thread = threading.Thread(target=self.update_thread_func)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        # Initially update the plots
        self.tab.after(100, self.check_update_queue)
    
    def initialize_plot_lines(self):
        """Initialize the plot lines for voltage and temperature"""
        # Clear any existing lines
        self.voltage_lines = []
        self.temp_lines = []
        
        # Initialize empty lines for voltage
        voltage_colors = plt.cm.tab20(np.linspace(0, 1, 16))
        for cell_id in range(16):
            line, = self.voltage_ax.plot([], [], label=f"Cell {cell_id}", color=voltage_colors[cell_id])
            self.voltage_lines.append(line)
        
        # Initialize empty lines for temperature
        temp_colors = plt.cm.tab10(np.linspace(0, 1, 6))
        for cell_id in range(6):
            line, = self.temp_ax.plot([], [], label=f"Cell {cell_id}", color=temp_colors[cell_id])
            self.temp_lines.append(line)
        
        # Add legends
        self.voltage_ax.legend(loc='upper right', ncol=4, fontsize='small')
        self.temp_ax.legend(loc='upper right', ncol=3, fontsize='small')
    
    def update_plots(self):
        """Update the plots with the latest data"""
        module_id = int(self.module_var.get())
        
        # Update voltage plot
        for cell_id in range(16):
            timestamps, values = self.sensor_buffer.get_voltage_data(module_id, cell_id)
            if timestamps is not None and len(timestamps) > 0:
                # Convert timestamps to datetime objects for better x-axis display
                # Convert numpy.float32 to standard Python float for datetime compatibility
                datetime_timestamps = [datetime.fromtimestamp(float(ts)/1000.0) for ts in timestamps]
                self.voltage_lines[cell_id].set_data(datetime_timestamps, values)
        
        # Update temperature plot
        for cell_id in range(6):
            timestamps, values = self.sensor_buffer.get_temperature_data(module_id, cell_id)
            if timestamps is not None and len(timestamps) > 0:
                # Convert timestamps to datetime objects
                # Convert numpy.float32 to standard Python float for datetime compatibility
                datetime_timestamps = [datetime.fromtimestamp(float(ts)/1000.0) for ts in timestamps]
                self.temp_lines[cell_id].set_data(datetime_timestamps, values)
        
        # Adjust x-axis limits for voltage plot
        if any(len(line.get_xdata()) > 0 for line in self.voltage_lines):
            all_timestamps = []
            for line in self.voltage_lines:
                if len(line.get_xdata()) > 0:
                    all_timestamps.extend(line.get_xdata())
            if all_timestamps:
                self.voltage_ax.set_xlim(min(all_timestamps), max(all_timestamps))
                self.voltage_ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                # Only show x-labels on bottom plot
                self.voltage_ax.tick_params(labelbottom=False)
        
        # Adjust x-axis limits for temperature plot
        if any(len(line.get_xdata()) > 0 for line in self.temp_lines):
            all_timestamps = []
            for line in self.temp_lines:
                if len(line.get_xdata()) > 0:
                    all_timestamps.extend(line.get_xdata())
            if all_timestamps:
                self.temp_ax.set_xlim(min(all_timestamps), max(all_timestamps))
                self.temp_ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                self.fig.autofmt_xdate(rotation=45)
        
        # Update the canvas
        self.canvas.draw_idle()
        
        # Update status
        self.status_var.set(f"Updated: {datetime.now().strftime('%H:%M:%S')}")
    
    def on_module_change(self, event):
        """Handle module selection change"""
        module_id = int(self.module_var.get())
        self.module_id = module_id
        
        # Update subplot titles
        self.voltage_ax.set_title(f"Module {module_id} Cell Voltages")
        self.temp_ax.set_title(f"Module {module_id} Cell Temperatures")
        
        # Update the plots
        self.update_plots()
    
    def toggle_auto_update(self):
        """Toggle auto-update on/off"""
        if self.auto_update_var.get():
            self.update_button.config(state=tk.DISABLED)
        else:
            self.update_button.config(state=tk.NORMAL)
    
    def update_thread_func(self):
        """Thread function to monitor for data updates"""
        while self.running:
            # Wait for data from the sensor buffer
            new_data = self.sensor_buffer.wait_for_data(timeout=0.5)
            
            current_time = time.time()
            update_interval = float(self.update_interval_var.get())
            
            # Check if:
            # 1. Auto-update is enabled
            # 2. We have new data
            # 3. This tab is currently visible/active
            # 4. Enough time has passed since the last update
            if (new_data and 
                self.auto_update_var.get() and 
                self.is_tab_active() and
                (current_time - self.last_update_time >= update_interval)):
                
                # Put an update request in the queue
                self.update_queue.put(True)
                self.last_update_time = current_time
                
            time.sleep(0.1)
    
    def is_tab_active(self):
        """Check if this tab is currently visible/selected in the notebook"""
        return self.notebook.index(self.notebook.select()) == self.notebook.index(self.tab)
    
    def on_interval_change(self, event):
        """Handle update interval change"""
        # Reset the last update time to force an immediate update with the new interval
        self.last_update_time = 0
    
    def check_update_queue(self):
        """Check if there are update requests in the queue"""
        try:
            while True:
                self.update_queue.get_nowait()
                # Only update plots if this tab is currently active and the module matches
                if self.is_tab_active() and int(self.module_var.get()) == self.module_id:
                    self.update_plots()
                self.update_queue.task_done()
        except queue.Empty:
            pass
        
        # Schedule the next check
        self.tab.after(100, self.check_update_queue)
    
    def stop(self):
        """Stop the update thread"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)


class SensorMonitorApp:
    """Main application window with notebook interface"""
    def __init__(self, root):
        self.root = root
        self.root.title("Battery Sensor Monitor")
        self.root.geometry("1200x800")
        
        # Configure the root window
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Create sensor data buffer (already configured for 5 modules)
        # Buffer size might need adjustment based on expected data rate / desired history
        self.sensor_buffer = ModularSensorBuffer(buffer_size=600) # Increased buffer size example

        # # --- Status Bar (Added for CAN status) ---
        # self.status_frame = ttk.Frame(self.root)
        # self.status_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=(5, 0)) # Place below notebook/buttons
        self.status_var = tk.StringVar(value="Initializing...")
        # self.status_label = ttk.Label(self.status_frame, textvariable=self.status_var, anchor=tk.W, relief=tk.SUNKEN, padding=2)
        # self.status_label.pack(fill=tk.X, expand=True)

        # Create and start the CAN reader thread
        # Pass a callback to update the status bar (optional, needs careful implementation)
        self.can_reader = CanReaderThread(self.sensor_buffer, status_callback=self.update_status_bar)
        self.can_reader.start() # Start reading CAN data

        # --- Notebook Setup (Remains similar) ---
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # Create the plot tab(s) - Example for one module
        # You might create multiple tabs or allow dynamic creation
        self.plot_tab = SensorPlotTab(self.notebook, self.sensor_buffer, module_id=0) # Start with Module 0

        # Add Close button
        self.button_frame = ttk.Frame(self.root)
        # TODO: Adjust grid row to accommodate status bar
        self.button_frame.grid(row=1, column=0, sticky="e", padx=10, pady=10)
        self.close_button = ttk.Button(self.button_frame, text="Close", command=self.on_close)
        self.close_button.pack(side=tk.RIGHT)
        
        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def update_status_bar(self, message: str, is_error: bool = False):
         """Thread-safe method to update the status bar."""
         # Use `after` to ensure GUI updates happen in the main thread
         self.root.after(0, lambda: self.status_var.set(message))
         # You could also change label colors here based on is_error

    def on_close(self):
        """Handle application close."""
        print("Closing application...")
        # Stop the plot tab updates first (if it has its own thread/timers)
        if hasattr(self, 'plot_tab'):
             # Assuming SensorPlotTab might have background tasks to stop
             if hasattr(self.plot_tab, 'stop'):
                 self.plot_tab.stop()
        # Stop other plot tabs if they exist...

        # Stop the CAN reader thread
        if hasattr(self, 'can_reader') and self.can_reader.is_alive():
            print("Stopping CAN reader thread...")
            self.can_reader.stop()
            self.can_reader.join(timeout=2.0) # Wait for thread to finish
            if self.can_reader.is_alive():
                 print("CAN reader thread did not stop gracefully.")

        self.root.destroy()
        print("Application closed.")


# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = SensorMonitorApp(root)
    root.mainloop()