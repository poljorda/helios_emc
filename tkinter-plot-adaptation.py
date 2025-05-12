import tkinter as tk
from tkinter import ttk
import numpy as np
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.dates as mdates
from datetime import datetime
import queue

# Import the sensor buffer implementation
from sensor_data_structure import ModularSensorBuffer, SensorDataProducer


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
        notebook.add(self.tab, text=f"Module {module_id} Data")
        
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
        self.voltage_ax.set_ylim(2.7, 4.2)  # Fixed Y axis for voltage
        self.voltage_ax.grid(True)
        
        # Create temperature subplot
        self.temp_ax = self.fig.add_subplot(2, 1, 2)
        self.temp_ax.set_title(f"Module {module_id} Cell Temperatures")
        self.temp_ax.set_ylabel("Temperature (°C)")
        self.temp_ax.set_ylim(15, 45)  # Fixed Y axis for temperature
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
        self.update_queue = queue.Queue()
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
        
        # Create a notebook widget
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Create sensor data buffer
        self.sensor_buffer = ModularSensorBuffer(buffer_size=60)
        
        # Create and start the data producer (simulating CAN bus)
        self.producer = SensorDataProducer(self.sensor_buffer)
        self.producer.start()
        
        # Create the first plot tab for Module 1
        self.plot_tab = SensorPlotTab(self.notebook, self.sensor_buffer, module_id=1)
        
        # Add Close button
        self.button_frame = ttk.Frame(self.root)
        self.button_frame.grid(row=1, column=0, sticky="e", padx=10, pady=10)
        self.close_button = ttk.Button(self.button_frame, text="Close", command=self.on_close)
        self.close_button.pack(side=tk.RIGHT)
        
        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def on_close(self):
        """Handle application close"""
        # Stop all threads
        if hasattr(self, 'plot_tab'):
            self.plot_tab.stop()
        
        if hasattr(self, 'producer'):
            self.producer.stop()
            self.producer.join(timeout=1.0)
        
        self.root.destroy()


# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = SensorMonitorApp(root)
    root.mainloop()