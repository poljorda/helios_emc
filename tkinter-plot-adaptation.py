import tkinter as tk  # Assuming SensorMonitorApp might pass tk.Tk for status updates
from tkinter import ttk, filedialog, messagebox, simpledialog # Added filedialog, messagebox, simpledialog
import numpy as np
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk  # type: ignore
from matplotlib.figure import Figure
from matplotlib.axes import Axes  # Added
from matplotlib.lines import Line2D  # Added
import matplotlib.dates as mdates
from datetime import datetime
import queue

# Imports needed for CAN handling
import can
import cantools
import re
import os
import shutil
import sys  # Optional: for error handling during DBC/CAN init

# Type hinting imports
from typing import Optional, Tuple, Callable, Any, Dict, List  # Added List

# Import the sensor buffer implementation
from sensor_data_structure import ModularSensorBuffer

# Import vehicle CAN Communication library
from vehicle_can_communication import VehicleCanCommsThread, SharedValue

# Import the logging manager
from logging_manager import LoggingManager, LoggingCanMessageObserver, LoggingVehicleCanMessageObserver

# --- Constants (Adapt from can_monitor_app.py and can_simulator.py) ---
DBC_FILE_PATH = r"resources\IOT_CAN_v4.0.dbc"  # Make sure this path is correct
KVASER_INTERFACE = "kvaser"
# Use channel 0 for virtual simulation, maybe 1 for real hardware? Make configurable?
KVASER_CHANNEL = 1  # For testing with can_simulator.py which uses channel 0
IS_VIRTUAL = False  # Set True for testing with simulator

# CAN FD Bit Timing (Must match monitor and simulator)
ARBITRATION_BITRATE = 1000000
DATA_BITRATE = 4000000

# Logging Path
LOGGING_BASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "evidences")


class CanReaderThread(threading.Thread):
    """Reads CAN messages, decodes them, and updates the sensor buffer."""

    def __init__(
        self, sensor_buffer: ModularSensorBuffer, 
        status_callback: Optional[Callable[[str, bool], None]] = None
    ):
        super().__init__()
        self.sensor_buffer: ModularSensorBuffer = sensor_buffer
        self.status_callback: Optional[Callable[[str, bool], None]] = status_callback
        self.running: bool = False
        self.db: Optional[cantools.db.Database] = None  # type: ignore
        self.bus: Optional[can.BusABC] = None
        self.daemon: bool = True
        self.logging_observer: Optional[LoggingCanMessageObserver] = None
        
    def set_logging_observer(self, observer: Optional[LoggingCanMessageObserver]) -> None:
        """Set or clear the logging observer."""
        self.logging_observer = observer

    def _update_status(self, message: str, is_error: bool = False) -> None:
        """Safely updates the status bar via callback."""
        print(f"CAN Status: {message}")
        if self.status_callback:
            # If status_callback needs to interact with Tkinter GUI from this thread,
            # it should use root.after() or a thread-safe queue.
            # For simplicity, we assume the callback itself handles thread safety if needed.
            if self.running:
                # Only update status if the thread is running
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
        except cantools.db.UnsupportedDatabaseFormatError as e:  # More specific error
            self.db = None
            self._update_status(f"Error parsing DBC file (format error): {e}", is_error=True)
            return False
        except Exception as e:  # General fallback
            self.db = None
            self._update_status(f"Error loading DBC: {e}", is_error=True)
            return False

    def _init_can_bus(self) -> bool:
        """Initializes the CAN bus."""
        try:
            bit_timing_fd = can.BitTimingFd(
                f_clock=80_000_000,
                nom_brp=1,
                nom_tseg1=64,
                nom_tseg2=15,
                nom_sjw=2,
                data_brp=1,
                data_tseg1=13,
                data_tseg2=6,
                data_sjw=6,
            )

            self.bus = can.interface.Bus(
                interface=KVASER_INTERFACE,
                channel=str(KVASER_CHANNEL),  # Ensure channel is a string for python-can
                fd=True,
                bitrate=ARBITRATION_BITRATE,
                data_bitrate=DATA_BITRATE,
                timing=bit_timing_fd,
                is_virtual=IS_VIRTUAL,
            )
            
            # Remove the references to self.logging_manager and can_listener
            # We'll handle logging through the logging_observer attribute
                
            self._update_status(
                f"Connected to {KVASER_INTERFACE} channel {KVASER_CHANNEL} (FD Mode {'Virtual' if IS_VIRTUAL else ''}). Monitoring started."
            )
            return True
        except can.CanInterfaceNotImplementedError as e:
            self.bus = None
            self._update_status(
                f"Error: '{KVASER_INTERFACE}' interface not implemented or drivers missing: {e}", is_error=True
            )
            return False
        except can.CanInitializationError as e:
            self.bus = None
            self._update_status(f"Error initializing Kvaser interface: {e}", is_error=True)
            return False
        except Exception as e:  # General fallback
            self.bus = None
            self._update_status(f"Failed to initialize CAN bus: {e}", is_error=True)
            return False

    def parse_signal_name(self, signal_name: str) -> Optional[Tuple[str, int, int]]:
        """
        Parses signal names like 'UCellBattPwrHi_1_1' or 'TCellBattEgyHi_12_6'.
        - Only keeps module 1 from Pwr messages (maps to module 5/index 4)
        - Ignores all other Pwr messages
        - Ignores module 5 from Egy messages to prevent overwriting Pwr values
        """
        match = re.match(r"(U|T)CellBatt(Pwr|Egy)Hi_(\d+)_(\d+)", signal_name)
        if match:
            data_type_char, pwr_egy, module_str, cell_str = match.groups()
            data_type_map: Dict[str, str] = {"U": "voltage", "T": "temperature"}
            data_type: Optional[str] = data_type_map.get(data_type_char)
            
            if data_type:
                try:
                    # Handle Pwr messages - only keep module 1
                    if pwr_egy == "Pwr":
                        if module_str == "1":
                            module_idx = 4  # Map to 5th module (index 4)
                        else:
                            return None  # Discard all other Pwr modules
                    # Handle Egy messages
                    elif pwr_egy == "Egy":
                        if module_str == "5":
                            return None  # Skip module 5 of Egy messages
                        else:
                            module_idx = int(module_str) - 1
                    else:
                        return None  # Should not happen with the regex pattern
                        
                    cell_idx = int(cell_str) - 1
                    return data_type, module_idx, cell_idx
                except ValueError:
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
                if msg and self.db:  # Ensure self.db is not None
                    # Log the raw CAN message first if logging is enabled
                    if self.logging_observer:
                        self.logging_observer.log_message(msg)
                        
                    try:
                        # cantools expects data to be bytes or bytearray
                        decoded_signals: Dict[str, Any] = self.db.decode_message(msg.arbitration_id, bytes(msg.data))
                        timestamp_ms: float = msg.timestamp * 1000.0  # CAN timestamp is in seconds

                        for signal_name, value in decoded_signals.items():
                            parse_result: Optional[Tuple[str, int, int]] = self.parse_signal_name(signal_name)
                            if parse_result:
                                data_type: str
                                module_id: int
                                cell_id: int
                                data_type, module_id, cell_id = parse_result

                                if 0 <= module_id < 5:  # Limit to first 5 modules
                                    # Ensure value is float, cantools usually returns numeric types
                                    try:
                                        numeric_value: float = float(value)
                                        if data_type == "voltage":
                                            self.sensor_buffer.update_voltage(
                                                module_id, cell_id, timestamp_ms, numeric_value
                                            )
                                            # Log voltage data if logging is enabled
                                            if self.logging_observer and isinstance(self.logging_observer.logging_manager, LoggingManager):
                                                self.logging_observer.logging_manager.log_voltage(
                                                    module_id, cell_id, timestamp_ms, numeric_value
                                                )
                                        elif data_type == "temperature":
                                            self.sensor_buffer.update_temperature(
                                                module_id, cell_id, timestamp_ms, numeric_value
                                            )
                                            # Log temperature data if logging is enabled
                                            if self.logging_observer and isinstance(self.logging_observer.logging_manager, LoggingManager):
                                                self.logging_observer.logging_manager.log_temperature(
                                                    module_id, cell_id, timestamp_ms, numeric_value
                                                )
                                    except (ValueError, TypeError) as e:
                                        print(
                                            f"Warning: Could not convert signal value '{value}' to float for {signal_name}: {e}"
                                        )

                    except KeyError:  # Message ID not in DBC
                        # print(f"Message ID {hex(msg.arbitration_id)} not in DBC.") # Can be noisy
                        pass
                    except cantools.db.DecodeError as decode_error:  # type: ignore[attr-defined]
                        print(f"Decode Error for ID {hex(msg.arbitration_id)}: {decode_error}")
                    except ValueError as val_err:  # Catch potential errors from bytes(msg.data) or float conversion
                        print(f"ValueError processing msg ID {hex(msg.arbitration_id)}: {val_err}")
                    except Exception as e:  # General processing error
                        print(f"Error decoding/processing msg ID {hex(msg.arbitration_id)}: {e}")

            except can.CanOperationError as e:
                self._update_status(f"CAN Operation Error: {e}. Attempting to continue...", is_error=True)
                # Potentially add logic to try and reset the bus here if it's a recoverable error
                time.sleep(2)
            except Exception as e:  # Catch-all for unexpected errors in the loop (e.g., bus becomes None unexpectedly)
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

    # Type hints for instance variables
    sensor_buffer: ModularSensorBuffer
    module_id: int
    notebook: ttk.Notebook
    last_update_time: float
    tab: ttk.Frame
    controls_frame: ttk.Frame
    module_var: tk.StringVar
    module_selector: ttk.Combobox
    auto_update_var: tk.BooleanVar
    auto_update_checkbox: ttk.Checkbutton
    update_interval_var: tk.StringVar
    interval_selector: ttk.Combobox
    update_button: ttk.Button
    clear_data_button: ttk.Button  # Added
    fig: Figure
    voltage_ax: Axes
    temp_ax: Axes
    voltage_lines: List[Line2D]
    temp_lines: List[Line2D]
    canvas_frame: ttk.Frame
    canvas: FigureCanvasTkAgg
    toolbar_frame: ttk.Frame
    toolbar: NavigationToolbar2Tk
    status_frame: ttk.Frame
    status_var: tk.StringVar
    status_label: ttk.Label
    update_queue: "queue.Queue[bool]"  # Forward reference for queue.Queue
    running: bool
    update_thread: threading.Thread

    def __init__(self, notebook: ttk.Notebook, sensor_buffer: ModularSensorBuffer, module_id: int = 1) -> None:
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
        # Adjust row weights: controls (0), canvas (1, weight=1), toolbar (2), status (3)
        self.tab.rowconfigure(1, weight=1)  # Give plot canvas more weight

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
            state="readonly",
        )
        self.module_selector.pack(side=tk.LEFT, padx=(0, 10))
        self.module_selector.bind("<<ComboboxSelected>>", self.on_module_change)

        # Auto-update checkbox
        self.auto_update_var = tk.BooleanVar(value=True)
        self.auto_update_checkbox = ttk.Checkbutton(
            self.controls_frame, text="Auto update", variable=self.auto_update_var, command=self.toggle_auto_update
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
            state="readonly",
        )
        self.interval_selector.pack(side=tk.LEFT, padx=(0, 10))
        self.interval_selector.bind("<<ComboboxSelected>>", self.on_interval_change)

        # Manual update button
        self.update_button = ttk.Button(
            self.controls_frame, text="Update Now", command=self.update_plots, state=tk.DISABLED
        )
        self.update_button.pack(side=tk.LEFT, padx=10)

        # Clear Data button
        self.clear_data_button = ttk.Button(
            self.controls_frame, text="Clear Data", command=self.on_clear_data
        )
        self.clear_data_button.pack(side=tk.RIGHT, padx=0)

        # Create matplotlib figure with two subplots
        self.fig = Figure(figsize=(10, 6), dpi=100, tight_layout=True)  # Reduced height from 8 to 6
        # self.fig.subplots_adjust(hspace=0.4) # Adjusted hspace if needed

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

    def initialize_plot_lines(self) -> None:
        """Initialize the plot lines for voltage and temperature"""
        # Clear any existing lines
        self.voltage_lines = []
        self.temp_lines = []

        # Initialize empty lines for voltage
        voltage_colors = plt.cm.tab20(np.linspace(0, 1, 16))  # type: ignore[attr-defined]
        for cell_id in range(16):
            (line,) = self.voltage_ax.plot([], [], label=f"Cell {cell_id}", color=voltage_colors[cell_id])
            self.voltage_lines.append(line)

        # Initialize empty lines for temperature
        temp_colors = plt.cm.tab10(np.linspace(0, 1, 6))  # type: ignore[attr-defined]
        for cell_id in range(6):
            (line,) = self.temp_ax.plot([], [], label=f"Cell {cell_id}", color=temp_colors[cell_id])
            self.temp_lines.append(line)

        # Add legends
        self.voltage_ax.legend(loc="upper right", ncol=4, fontsize="small")
        self.temp_ax.legend(loc="upper right", ncol=3, fontsize="small")

    def update_plots(self) -> None:
        """Update the plots with the latest data"""
        module_id: int = int(self.module_var.get())

        # Update voltage plot
        for cell_id in range(16):
            timestamps, values = self.sensor_buffer.get_voltage_data(module_id, cell_id)
            if timestamps is not None and len(timestamps) > 0:
                # Convert timestamps to datetime objects for better x-axis display
                # Convert numpy.float32 to standard Python float for datetime compatibility
                datetime_timestamps_voltages: List[datetime] = [
                    datetime.fromtimestamp(float(ts) / 1000.0) for ts in timestamps
                ]
                self.voltage_lines[cell_id].set_data(datetime_timestamps_voltages, values)  # type: ignore[arg-type]
            else:  # Ensure lines are cleared if no data
                self.voltage_lines[cell_id].set_data([], [])

        # Update temperature plot
        for cell_id in range(6):
            timestamps, values = self.sensor_buffer.get_temperature_data(module_id, cell_id)
            if timestamps is not None and len(timestamps) > 0:
                # Convert timestamps to datetime objects
                # Convert numpy.float32 to standard Python float for datetime compatibility
                datetime_timestamps_temps: List[datetime] = [
                    datetime.fromtimestamp(float(ts) / 1000.0) for ts in timestamps
                ]
                self.temp_lines[cell_id].set_data(datetime_timestamps_temps, values)  # type: ignore[arg-type]
            else:  # Ensure lines are cleared if no data
                self.temp_lines[cell_id].set_data([], [])

        # Adjust x-axis limits for voltage plot
        all_timestamps_volt: List[datetime] = []
        for line in self.voltage_lines:
            if len(line.get_xdata()) > 0:  # type: ignore[arg-type]
                all_timestamps_volt.extend(line.get_xdata())  # type: ignore[arg-type]

        if all_timestamps_volt:
            self.voltage_ax.set_xlim(min(all_timestamps_volt), max(all_timestamps_volt))  # type: ignore[arg-type]
            self.voltage_ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            self.voltage_ax.tick_params(labelbottom=False)
        else:  # Reset limits if no data
            self.voltage_ax.set_xlim(datetime.now(), datetime.now())  # type: ignore[arg-type]
            self.voltage_ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            self.voltage_ax.tick_params(labelbottom=False)

        # Adjust x-axis limits for temperature plot
        all_timestamps_temp: List[datetime] = []
        for line in self.temp_lines:
            if len(line.get_xdata()) > 0:  # type: ignore[arg-type]
                all_timestamps_temp.extend(line.get_xdata())  # type: ignore[arg-type]

        if all_timestamps_temp:
            self.temp_ax.set_xlim(min(all_timestamps_temp), max(all_timestamps_temp))  # type: ignore[arg-type]
            self.temp_ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            self.fig.autofmt_xdate(rotation=45)
        else:  # Reset limits if no data
            self.temp_ax.set_xlim(datetime.now(), datetime.now())  # type: ignore[arg-type]
            self.temp_ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            self.fig.autofmt_xdate(rotation=45)

        # Update the canvas
        self.canvas.draw_idle()

        # Update status
        self.status_var.set(f"Updated: {datetime.now().strftime('%H:%M:%S')}")

    def on_module_change(self, event: Any) -> None:  # tk.Event can also be used if preferred
        """Handle module selection change"""
        module_id: int = int(self.module_var.get())
        self.module_id = module_id

        # Update subplot titles
        self.voltage_ax.set_title(f"Module {module_id} Cell Voltages")
        self.temp_ax.set_title(f"Module {module_id} Cell Temperatures")

        # Update the plots
        self.update_plots()

    def toggle_auto_update(self) -> None:
        """Toggle auto-update on/off"""
        if self.auto_update_var.get():
            self.update_button.config(state=tk.DISABLED)
            # Consider if clear_data_button state should also change
        else:
            self.update_button.config(state=tk.NORMAL)

    def on_clear_data(self) -> None:
        """Clear all data from the buffer and update plots."""
        # Assuming ModularSensorBuffer has a method to clear all data.
        # If not, this part needs to be implemented in ModularSensorBuffer.
        if hasattr(self.sensor_buffer, "clear_all_data"):
            self.sensor_buffer.clear_all_data()
            print("Sensor data cleared.")
        else:
            print("Warning: ModularSensorBuffer does not have a 'clear_all_data' method.")
            # As a fallback, you might try to re-initialize the buffer or clear known structures
            # For now, we'll just clear the plot lines.

        # Clear data from plot lines
        for line in self.voltage_lines:
            line.set_data([], [])
        for line in self.temp_lines:
            line.set_data([], [])

        # Update plots to reflect cleared data
        self.update_plots()
        self.status_var.set(f"Data cleared: {datetime.now().strftime('%H:%M:%S')}")

    def update_thread_func(self) -> None:
        """Thread function to monitor for data updates"""
        while self.running:
            # Wait for data from the sensor buffer
            new_data: bool = self.sensor_buffer.wait_for_data(timeout=0.5)

            current_time: float = time.time()
            update_interval: float = float(self.update_interval_var.get())

            # Check if:
            # 1. Auto-update is enabled
            # 2. We have new data
            # 3. This tab is currently visible/active
            # 4. Enough time has passed since the last update
            if (
                new_data
                and self.auto_update_var.get()
                and self.is_tab_active()
                and (current_time - self.last_update_time >= update_interval)
            ):

                # Put an update request in the queue
                self.update_queue.put(True)
                self.last_update_time = current_time

            time.sleep(0.1)

    def is_tab_active(self) -> Any:
        """Check if this tab is currently visible/selected in the notebook"""
        try:  # Add try-except in case the tab is not found (e.g. during shutdown)
            return self.notebook.index(self.notebook.select()) == self.notebook.index(self.tab)
        except tk.TclError:
            return False  # Tab might have been destroyed or notebook is in an inconsistent state

    def on_interval_change(self, event: Any) -> None:  # tk.Event can also be used
        """Handle update interval change"""
        # Reset the last update time to force an immediate update with the new interval
        self.last_update_time = 0

    def check_update_queue(self) -> None:
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

    def stop(self) -> None:
        """Stop the update thread"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)


class SensorMonitorApp:
    """Main application window with notebook interface"""

    # Type hints for instance variables
    root: tk.Tk
    sensor_buffer: ModularSensorBuffer
    status_var: tk.StringVar
    can_reader: CanReaderThread
    # Add vehicle_can_comm to type hints
    vehicle_can_comm: VehicleCanCommsThread 
    notebook: ttk.Notebook
    plot_tab: SensorPlotTab  # Assuming SensorPlotTab is defined above
    button_frame: ttk.Frame
    close_button: ttk.Button
    status_frame: ttk.Frame  # Added for CAN status
    status_label: ttk.Label  # Added for CAN status
    # Add acquisition control frame and buttons to type hints
    acquisition_controls_frame: ttk.Frame
    # Replace start/stop buttons with a single toggle button
    toggle_acquisition_button: ttk.Button
    is_acquiring: bool # To store acquisition status

    # Logging related attributes
    toggle_logging_button: ttk.Button
    is_logging: bool
    logging_status_label: ttk.Label
    current_log_folder: Optional[str]
    logging_manager: Optional[LoggingManager]


    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("HELIOS BMS Monitor")
        self.root.state("zoomed")  # Set a default size for the window
        self.root.minsize(1200, 700)  # Adjusted minsize if needed

        self.is_acquiring = False  # Initial state: not acquiring
        self.is_logging = False   # Initial state: not logging
        self.current_log_folder = None
        self.logging_manager = None

        # Configure the root window
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=0)  # Acquisition Controls row
        self.root.rowconfigure(1, weight=1)  # Notebook row
        self.root.rowconfigure(2, weight=0)  # Button frame row
        self.root.rowconfigure(3, weight=0)  # CAN Status bar row

        # Create sensor data buffer (already configured for 5 modules)
        # Buffer size might need adjustment based on expected data rate / desired history
        self.sensor_buffer = ModularSensorBuffer(buffer_size=600)  # 600 for 10 minutes approx of data

        # --- Status Bar (Added for CAN status) ---
        self.status_frame = ttk.Frame(self.root)
        # Grid position changed to be at the bottom
        self.status_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(5, 5))
        self.status_var = tk.StringVar(value="Initializing...")
        self.status_label = ttk.Label(self.status_frame, textvariable=self.status_var, anchor=tk.W, relief=tk.SUNKEN, padding=2)
        self.status_label.pack(fill=tk.X, expand=True)

        # Create and start the CAN reader thread
        # LoggingManager will be attached later when logging starts
        self.can_reader = CanReaderThread(self.sensor_buffer, status_callback=self.update_status_bar)
        self.can_reader.start()  # Start reading CAN data
        
        # Create and start the Vehicle CAN Communication thread
        self.vehicle_can_comm = VehicleCanCommsThread(self.update_status_bar, None) # Pass status callback
        self.vehicle_can_comm.start()  # Start vehicle CAN communication

        # --- Style Configuration ---
        style = ttk.Style()
        
        # Configure styles to remove focus ring and set text color for Acquisition Button
        style.configure("RedText.TButton", 
                        foreground="red", 
                        focusthickness=0, 
                        highlightthickness=0)
        style.map("RedText.TButton",
                  focusthickness=[('focus', 0)],
                  highlightthickness=[('focus', 0)])

        style.configure("GreenText.TButton", 
                        foreground="green", 
                        focusthickness=0, 
                        highlightthickness=0)
        style.map("GreenText.TButton",
                  focusthickness=[('focus', 0)],
                  highlightthickness=[('focus', 0)])

        # Configure styles for Logging Button
        style.configure("Logging.TButton",
                        foreground="blue",
                        focusthickness=0,
                        highlightthickness=0)
        style.map("Logging.TButton",
                  focusthickness=[('focus', 0)],
                  highlightthickness=[('focus', 0)])
        
        style.configure("NotLogging.TButton",
                        foreground="black", # Or your default button text color
                        focusthickness=0,
                        highlightthickness=0)
        style.map("NotLogging.TButton",
                  focusthickness=[('focus', 0)],
                  highlightthickness=[('focus', 0)])


        # --- Acquisition Control Buttons ---
        self.acquisition_controls_frame = ttk.Frame(self.root)
        self.acquisition_controls_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(5,0))


        self.toggle_acquisition_button = ttk.Button(
            self.acquisition_controls_frame, text="Start Acquisition", command=self.toggle_acquisition
        )
        self.toggle_acquisition_button.pack(side=tk.LEFT, padx=5)
        
        self.toggle_logging_button = ttk.Button(
            self.acquisition_controls_frame, text="Start Logging", command=self.toggle_logging
        )
        self.toggle_logging_button.pack(side=tk.LEFT, padx=5)

        self.logging_status_label = ttk.Label(self.acquisition_controls_frame, text="Status")
        self.logging_status_label.pack(side=tk.LEFT, padx=10)
        
        # Set initial style for the buttons and status
        self._update_acquisition_button_style()
        self._update_logging_controls()


        # --- Notebook Setup (Remains similar) ---
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        # Create the plot tab(s) - Example for one module
        # You might create multiple tabs or allow dynamic creation
        self.plot_tab = SensorPlotTab(self.notebook, self.sensor_buffer, module_id=0)  # Start with Module 0

        # Add Close button
        self.button_frame = ttk.Frame(self.root)
        # Grid row changed to be above the CAN status bar
        self.button_frame.grid(row=2, column=0, sticky="e", padx=10, pady=(5, 0))
        self.close_button = ttk.Button(self.button_frame, text="Close", command=self.on_close)
        self.close_button.pack(side=tk.RIGHT)

        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)


    def update_status_bar(self, message: str, is_error: bool = False) -> None:
        """Thread-safe method to update the status bar."""
        # Use `after` to ensure GUI updates happen in the main thread
        self.root.after(0, lambda: self.status_var.set(message))
        # You could also change label colors here based on is_error

    def _update_acquisition_button_style(self) -> None:
        """Updates the toggle acquisition button's text and color based on is_acquiring state."""
        if self.is_acquiring:
            self.toggle_acquisition_button.config(text="Stop Acquisition", style="RedText.TButton")
        else:
            self.toggle_acquisition_button.config(text="Start Acquisition", style="GreenText.TButton")

    def _update_logging_controls(self) -> None:
        """Updates logging button style, enabled state, and status label."""
        if self.is_logging:
            self.logging_status_label.config(text="Logging")
            self.toggle_logging_button.config(text="Stop Logging", style="Logging.TButton")
        elif self.is_acquiring:
            self.logging_status_label.config(text="Acquiring")
            self.toggle_logging_button.config(text="Start Logging", style="NotLogging.TButton")
        else:
            self.logging_status_label.config(text="Acquisition Stopped")
            self.toggle_logging_button.config(text="Start Logging", style="NotLogging.TButton")

        if self.is_acquiring:
            self.toggle_logging_button.config(state=tk.NORMAL)
        else:
            self.toggle_logging_button.config(state=tk.DISABLED)


    def toggle_acquisition(self) -> None:
        """Toggles CAN data acquisition on or off."""
        was_acquiring = self.is_acquiring
        self.is_acquiring = not self.is_acquiring

        if self.vehicle_can_comm:
            self.vehicle_can_comm.vehicle_status.set(self.is_acquiring)
            if self.is_acquiring:
                self.update_status_bar("CAN Acquisition Started.", False)
                print("CAN Acquisition Started.")
            else:
                self.update_status_bar("CAN Acquisition Stopped.", False)
                print("CAN Acquisition Stopped.")
                if was_acquiring and self.is_logging: # Was acquiring, now stopping, and was logging
                    self._handle_stop_acquisition_while_logging()
        
        self._update_acquisition_button_style()
        self._update_logging_controls() # Update logging button state and status label
        # Shift focus away from the button to prevent lingering focus highlight
        self.root.focus_set()

    def _start_logging_action(self, folder_path: str) -> None:
        """Start logging operations by creating the LoggingManager and attaching observers to the CAN readers."""
        
        # Create the logging manager
        self.logging_manager = LoggingManager(
            num_voltage_modules=self.sensor_buffer.num_voltage_modules,
            num_voltage_cells=self.sensor_buffer.num_voltage_cells,
            num_temp_modules=self.sensor_buffer.num_temp_modules,
            num_temp_cells=self.sensor_buffer.num_temp_cells
        )
        
        # Start the logging manager
        if self.logging_manager.start_logging():
            self.current_log_folder = folder_path
            print(f"Start logging action triggered. Logging to: {self.current_log_folder}")
            
            # Create observers and attach them to the CAN readers to log messages
            iot_observer = LoggingCanMessageObserver(self.logging_manager)
            self.can_reader.set_logging_observer(iot_observer)
            
            # Create and attach the vehicle CAN observer if it exists
            if hasattr(self.vehicle_can_comm, 'set_logging_observer'):
                vehicle_observer = LoggingVehicleCanMessageObserver(self.logging_manager)
                self.vehicle_can_comm.set_logging_observer(vehicle_observer)
            
            self.update_status_bar("Logging started", False)
        else:
            # Logging failed to start
            self.update_status_bar("Failed to start logging", True)
            messagebox.showerror("Logging Error", "Failed to start logging", parent=self.root)
            self.logging_manager = None

    def _stop_logging_action(self, folder_path: str) -> None:
        """Stop logging operations."""
        if self.logging_manager:
            # Remove the logging observers from the CAN readers
            self.can_reader.set_logging_observer(None)
            
            # Remove the vehicle CAN observer if it exists
            if hasattr(self.vehicle_can_comm, 'set_logging_observer'):
                self.vehicle_can_comm.set_logging_observer(None)
            
            # Stop the logging manager and get the temp directory path
            temp_dir = self.logging_manager.stop_logging()
            
            try:
                # Move logs to the destination folder
                if self.logging_manager.move_logs_to_destination(folder_path):
                    print(f"Logs successfully moved to: {folder_path}")
                else:
                    print(f"Failed to move logs to: {folder_path}")
                    messagebox.showerror("Logging Error", f"Failed to move logs to: {folder_path}", parent=self.root)
            except Exception as e:
                print(f"Error during log file moving: {e}")
                messagebox.showerror("Logging Error", f"Error moving log files: {e}", parent=self.root)
            
            # Cleanup
            self.logging_manager.cleanup()
            self.logging_manager = None
            
        self.current_log_folder = None

    def _prompt_and_save_log(self) -> bool:
        """Prompts user for log folder path using a file dialog, saves log, and updates state."""
        # Ensure LOGGING_BASE_PATH exists, create if not
        if not os.path.exists(LOGGING_BASE_PATH):
            try:
                os.makedirs(LOGGING_BASE_PATH, exist_ok=True)
            except OSError as e:
                messagebox.showerror("Error", f"Could not create base logging directory {LOGGING_BASE_PATH}: {e}", parent=self.root)
                return False # Indicate failure, logging continues

        timestamp_prefix = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        
        # asksaveasfilename will return the full path chosen by the user.
        # We treat this path as the name of the folder to be created.
        full_log_path = filedialog.asksaveasfilename(
            title="Save Log Folder As",
            initialdir=LOGGING_BASE_PATH,
            initialfile=timestamp_prefix,
            defaultextension="", # No specific extension for a folder
            parent=self.root
        )

        if full_log_path: # User selected a path and filename (which we use as folder name)
            try:
                # full_log_path is the path to the folder we want to create
                os.makedirs(full_log_path, exist_ok=True) 
                self._stop_logging_action(full_log_path)
                self.is_logging = False
                messagebox.showinfo("Log Saved", f"Log saved to: {full_log_path}", parent=self.root)
                return True # Logging stopped
            except OSError as e:
                messagebox.showerror("Error", f"Could not create log directory {full_log_path}: {e}", parent=self.root)
                return False # Indicate failure, logging continues
        else: # User cancelled the dialog
            if messagebox.askyesno("Discard Log?", 
                                   "Save operation cancelled. Do you want to discard the current log?\n"
                                   "(Choosing 'No' will continue logging)",
                                   parent=self.root):
                print("Logging discarded by user (cancelled save dialog).")
                if self.logging_manager:
                    # Clean up the logging manager
                    self.logging_manager.stop_logging()
                    self.logging_manager.cleanup()
                    self.logging_manager = None
                
                # Clear the observers from both CAN threads
                self.can_reader.set_logging_observer(None)
                if hasattr(self.vehicle_can_comm, 'set_logging_observer'):
                    self.vehicle_can_comm.set_logging_observer(None)
                
                self.is_logging = False
                return True # Logging stopped (discarded)
            else:
                print("User chose to continue logging after cancelling save dialog.")
                return False # Logging continues
    
    def _handle_stop_acquisition_while_logging(self) -> None:
        """Handles the scenario when acquisition is stopped while logging is active."""
        if messagebox.askyesno("Save Log?", 
                               "Acquisition has been stopped. Do you want to save the current log?\n"
                               "(Choosing 'No' will discard it)",
                               parent=self.root):
            self._prompt_and_save_log() # This will update is_logging
        else:
            print("Log discarded as acquisition stopped.")
            if self.logging_manager:
                self.logging_manager.stop_logging()
                self.logging_manager.cleanup()
                self.logging_manager = None
            self.is_logging = False
        # _update_logging_controls() will be called by toggle_acquisition

    def toggle_logging(self) -> None:
        """Toggles data logging on or off."""
        if not self.is_acquiring:
            messagebox.showwarning("Logging Disabled", "Cannot start logging when acquisition is stopped.", parent=self.root)
            return

        if not self.is_logging: # Start logging
            # For starting, we need a temporary folder or decide how to handle this.
            # Let's create a temporary unique name for now, actual saving happens on stop.
            # For now, _start_logging_action is a placeholder.
            # We can pass a conceptual "session_id" or similar if needed later.
            # For this iteration, let's assume _start_logging_action prepares for logging.
            # The actual folder is determined when stopping.
            
            # Let's define a temporary log path for the active session
            # This path might not be created until the first data is written or when stopped.
            temp_session_name = f"active_log_{datetime.now().strftime('%Y%m%d%H%M%S')}"
            self.current_log_folder = os.path.join(LOGGING_BASE_PATH, temp_session_name) # Conceptual path
            
            self._start_logging_action(self.current_log_folder) # Pass the conceptual path
            self.is_logging = True
        else: # Stop logging
            self._prompt_and_save_log()
            # is_logging is handled by _prompt_and_save_log

        self._update_logging_controls()
        self.root.focus_set()


    def on_close(self) -> None:
        """Handle application close."""
        if self.is_logging:
            if messagebox.askyesno("Logging Active", 
                                   "Logging is currently active. Do you want to save the log before closing?\n"
                                   "(Choosing 'No' will discard the log)", 
                                   parent=self.root):
                if not self._prompt_and_save_log(): # If save was cancelled and user chose to continue logging
                    messagebox.showwarning("Close Cancelled", "Application close cancelled to continue logging.", parent=self.root)
                    return # Do not close
            else: # Discard log
                print("Log discarded on application close.")
                if self.logging_manager:
                    self.logging_manager.stop_logging()
                    self.logging_manager.cleanup()
                    self.logging_manager = None

        print("Closing application...")
        # Stop the plot tab updates first (if it has its own thread/timers)
        if hasattr(self, "plot_tab"):
            # Assuming SensorPlotTab might have background tasks to stop
            if hasattr(self.plot_tab, "stop"):
                self.plot_tab.stop()
        # Stop other plot tabs if they exist...

        # Stop the CAN reader thread
        if hasattr(self, "can_reader") and self.can_reader.is_alive():
            print("Stopping CAN reader thread...")
            self.can_reader.stop()
            self.can_reader.join(timeout=2.0)  # Wait for thread to finish
            if self.can_reader.is_alive():
                print("CAN reader thread did not stop gracefully.")

        # Ensure vehicle communication is stopped
        if self.vehicle_can_comm:
            self.vehicle_can_comm.vehicle_status.set(False) # Explicitly stop acquisition
            if self.vehicle_can_comm.is_alive():
                print("Stopping Vehicle CAN Comm thread...")
                self.vehicle_can_comm.stop()
                self.vehicle_can_comm.join(timeout=2.0)
                if self.vehicle_can_comm.is_alive():
                    print("Vehicle CAN Comm thread did not stop gracefully.")

        self.root.destroy()
        print("Application closed.")


# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = SensorMonitorApp(root)
    root.mainloop()
