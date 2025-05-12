import numpy as np
import threading
import time
from collections import deque
from dataclasses import dataclass


@dataclass
class SensorReading:
    """Simple structure to hold a single sensor reading"""
    timestamp: float  # Unix timestamp in milliseconds (as float for precision)
    value: float      # Sensor value (voltage or temperature)


class ModularSensorBuffer:
    """
    Thread-safe circular buffer for voltage and temperature sensors organized in modules and cells.
    Optimized for efficient memory usage and fast plot data retrieval.
    """
    def __init__(self, buffer_size=600):
        self.buffer_size = buffer_size
        
        # Define sensor structure dimensions
        self.num_voltage_modules = 5
        self.num_voltage_cells = 16
        self.num_temp_modules = 5
        self.num_temp_cells = 6
        
        # Pre-allocate numpy arrays for timestamps and values
        # Using numpy structured arrays for improved memory usage and access speed
        # For timestamps, use float64 to ensure compatibility with datetime operations
        self.voltage_data = np.zeros((self.num_voltage_modules, self.num_voltage_cells, buffer_size, 2), 
                                    dtype=np.float64)
        self.temp_data = np.zeros((self.num_temp_modules, self.num_temp_cells, buffer_size, 2), 
                                 dtype=np.float64)
        
        # Indices to track current position in circular buffer for each sensor
        self.voltage_indices = np.zeros((self.num_voltage_modules, self.num_voltage_cells), dtype=np.int32)
        self.temp_indices = np.zeros((self.num_temp_modules, self.num_temp_cells), dtype=np.int32)
        
        # Is buffer full indicators 
        self.voltage_full = np.zeros((self.num_voltage_modules, self.num_voltage_cells), dtype=bool)
        self.temp_full = np.zeros((self.num_temp_modules, self.num_temp_cells), dtype=bool)
        
        # Thread locks - use RLock to allow the same thread to acquire the lock multiple times
        self.voltage_lock = threading.RLock()
        self.temp_lock = threading.RLock()
        
        # Track latest update timestamp for each sensor type for plot refresh decisions
        self.last_voltage_update = 0
        self.last_temp_update = 0
        
        # Flag to indicate if data is ready for plotting
        self.data_ready = threading.Event()

    def update_voltage(self, module_id, cell_id, timestamp, value):
        """Thread-safe update of voltage sensor data"""
        if not (0 <= module_id < self.num_voltage_modules and 0 <= cell_id < self.num_voltage_cells):
            return False
            
        with self.voltage_lock:
            idx = self.voltage_indices[module_id, cell_id]
            self.voltage_data[module_id, cell_id, idx, 0] = timestamp
            self.voltage_data[module_id, cell_id, idx, 1] = value
            
            # Update circular buffer index
            self.voltage_indices[module_id, cell_id] = (idx + 1) % self.buffer_size
            
            # Mark as full if we've wrapped around
            if idx == self.buffer_size - 1:
                self.voltage_full[module_id, cell_id] = True
                
            self.last_voltage_update = max(self.last_voltage_update, timestamp)
            
            # Signal that new data is available
            self.data_ready.set()
            
        return True

    def update_temperature(self, module_id, cell_id, timestamp, value):
        """Thread-safe update of temperature sensor data"""
        if not (0 <= module_id < self.num_temp_modules and 0 <= cell_id < self.num_temp_cells):
            return False
            
        with self.temp_lock:
            idx = self.temp_indices[module_id, cell_id]
            self.temp_data[module_id, cell_id, idx, 0] = timestamp
            self.temp_data[module_id, cell_id, idx, 1] = value
            
            # Update circular buffer index
            self.temp_indices[module_id, cell_id] = (idx + 1) % self.buffer_size
            
            # Mark as full if we've wrapped around
            if idx == self.buffer_size - 1:
                self.temp_full[module_id, cell_id] = True
                
            self.last_temp_update = max(self.last_temp_update, timestamp)
            
            # Signal that new data is available
            self.data_ready.set()
            
        return True

    def get_voltage_data(self, module_id, cell_id):
        """Get all data points for a specific voltage sensor in plot-ready format"""
        if not (0 <= module_id < self.num_voltage_modules and 0 <= cell_id < self.num_voltage_cells):
            return None, None
            
        with self.voltage_lock:
            idx = self.voltage_indices[module_id, cell_id]
            is_full = self.voltage_full[module_id, cell_id]
            
            if is_full:
                # If buffer is full, we need to reorder data to get chronological order
                timestamps = np.concatenate([
                    self.voltage_data[module_id, cell_id, idx:, 0],
                    self.voltage_data[module_id, cell_id, :idx, 0]
                ])
                values = np.concatenate([
                    self.voltage_data[module_id, cell_id, idx:, 1],
                    self.voltage_data[module_id, cell_id, :idx, 1]
                ])
            else:
                # If not full, just get the data up to current index
                timestamps = self.voltage_data[module_id, cell_id, :idx, 0]
                values = self.voltage_data[module_id, cell_id, :idx, 1]
                
            return timestamps, values

    def get_temperature_data(self, module_id, cell_id):
        """Get all data points for a specific temperature sensor in plot-ready format"""
        if not (0 <= module_id < self.num_temp_modules and 0 <= cell_id < self.num_temp_cells):
            return None, None
            
        with self.temp_lock:
            idx = self.temp_indices[module_id, cell_id]
            is_full = self.temp_full[module_id, cell_id]
            
            if is_full:
                # If buffer is full, we need to reorder data to get chronological order
                timestamps = np.concatenate([
                    self.temp_data[module_id, cell_id, idx:, 0],
                    self.temp_data[module_id, cell_id, :idx, 0]
                ])
                values = np.concatenate([
                    self.temp_data[module_id, cell_id, idx:, 1],
                    self.temp_data[module_id, cell_id, :idx, 1]
                ])
            else:
                # If not full, just get the data up to current index
                timestamps = self.temp_data[module_id, cell_id, :idx, 0]
                values = self.temp_data[module_id, cell_id, :idx, 1]
                
            return timestamps, values
            
    def get_latest_voltage_values(self):
        """Get the most recent voltage values as a 2D array (module x cell)"""
        with self.voltage_lock:
            latest_values = np.zeros((self.num_voltage_modules, self.num_voltage_cells), dtype=np.float32)
            
            for module_id in range(self.num_voltage_modules):
                for cell_id in range(self.num_voltage_cells):
                    # Get the index of the previous element (most recent)
                    idx = (self.voltage_indices[module_id, cell_id] - 1) % self.buffer_size
                    if idx < 0 or (idx == self.buffer_size - 1 and not self.voltage_full[module_id, cell_id]):
                        # No data yet
                        latest_values[module_id, cell_id] = np.nan
                    else:
                        latest_values[module_id, cell_id] = self.voltage_data[module_id, cell_id, idx, 1]
                        
            return latest_values
            
    def get_latest_temperature_values(self):
        """Get the most recent temperature values as a 2D array (module x cell)"""
        with self.temp_lock:
            latest_values = np.zeros((self.num_temp_modules, self.num_temp_cells), dtype=np.float32)
            
            for module_id in range(self.num_temp_modules):
                for cell_id in range(self.num_temp_cells):
                    # Get the index of the previous element (most recent)
                    idx = (self.temp_indices[module_id, cell_id] - 1) % self.buffer_size
                    if idx < 0 or (idx == self.buffer_size - 1 and not self.temp_full[module_id, cell_id]):
                        # No data yet
                        latest_values[module_id, cell_id] = np.nan
                    else:
                        latest_values[module_id, cell_id] = self.temp_data[module_id, cell_id, idx, 1]
                        
            return latest_values
    
    def get_multi_plot_data(self, sensor_type, module_ids, cell_ids):
        """
        Get data for multiple sensors for plotting
        sensor_type: 'voltage' or 'temperature'
        module_ids, cell_ids: lists of IDs to plot
        Returns: dict mapping (module,cell) to (timestamps, values)
        """
        result = {}
        
        if sensor_type == 'voltage':
            for module_id in module_ids:
                for cell_id in cell_ids:
                    if 0 <= module_id < self.num_voltage_modules and 0 <= cell_id < self.num_voltage_cells:
                        timestamps, values = self.get_voltage_data(module_id, cell_id)
                        if timestamps is not None and len(timestamps) > 0:
                            result[(module_id, cell_id)] = (timestamps, values)
        else:  # temperature
            for module_id in module_ids:
                for cell_id in cell_ids:
                    if 0 <= module_id < self.num_temp_modules and 0 <= cell_id < self.num_temp_cells:
                        timestamps, values = self.get_temperature_data(module_id, cell_id)
                        if timestamps is not None and len(timestamps) > 0:
                            result[(module_id, cell_id)] = (timestamps, values)
        
        return result
    
    def wait_for_data(self, timeout=1.0):
        """Wait for new data to be available, returns True if new data available, False on timeout"""
        result = self.data_ready.wait(timeout)
        if result:
            # Reset the event for next wait
            self.data_ready.clear()
        return result


# Example producer-consumer implementation

class SensorDataProducer(threading.Thread):
    """Simulates the CAN bus data producer"""
    def __init__(self, sensor_buffer):
        super().__init__()
        self.sensor_buffer = sensor_buffer
        self.running = True
        self.daemon = True  # Thread will exit when main program exits
    
    def run(self):
        """Main thread loop that simulates CAN bus messages"""
        while self.running:
            # Simulate voltage sensor data coming in (randomly selected module/cell)
            module_id = np.random.randint(0, self.sensor_buffer.num_voltage_modules)
            cell_id = np.random.randint(0, self.sensor_buffer.num_voltage_cells)
            timestamp = time.time() * 1000.0  # current time in ms as float
            voltage = 3.3 + 0.2 * np.random.randn()  # simulated voltage value
            self.sensor_buffer.update_voltage(module_id, cell_id, timestamp, voltage)
            
            # Simulate temperature sensor data (less frequent updates)
            if np.random.random() < 0.3:  # 30% chance of temperature update with each voltage update
                module_id = np.random.randint(0, self.sensor_buffer.num_temp_modules)
                cell_id = np.random.randint(0, self.sensor_buffer.num_temp_cells)
                timestamp = time.time() * 1000.0  # current time in ms as float
                temperature = 25.0 + 5.0 * np.random.randn()  # simulated temp value
                self.sensor_buffer.update_temperature(module_id, cell_id, timestamp, temperature)
            
            # Small sleep to simulate message intervals (adjust based on actual CAN bus rate)
            time.sleep(0.01)  # 10ms between messages
    
    def stop(self):
        """Stop the producer thread"""
        self.running = False


class GUIUpdater(threading.Thread):
    """Simulates GUI thread that consumes sensor data for plotting"""
    def __init__(self, sensor_buffer):
        super().__init__()
        self.sensor_buffer = sensor_buffer
        self.running = True
        self.daemon = True
    
    def run(self):
        """Main thread loop for updating plots"""
        while self.running:
            # Wait for new data (with timeout)
            new_data_available = self.sensor_buffer.wait_for_data(timeout=0.5)
            
            if new_data_available:
                # For the sliding window plot that needs to show 22 lines
                # Example: plot the first 11 voltage sensors and first 11 temperature sensors
                voltage_module_ids = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]
                voltage_cell_ids = [0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 5]
                
                temp_module_ids = [0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2]
                temp_cell_ids = [0, 1, 2, 0, 1, 2, 0, 1, 2, 3, 4]
                
                # Get plot data
                voltage_data = self.sensor_buffer.get_multi_plot_data('voltage', voltage_module_ids, voltage_cell_ids)
                temp_data = self.sensor_buffer.get_multi_plot_data('temperature', temp_module_ids, temp_cell_ids)
                
                # Here you would update your actual GUI plot
                self.update_plot(voltage_data, temp_data)
            
            # No need for sleep as wait_for_data handles timing
    
    def update_plot(self, voltage_data, temp_data):
        """Update the plot with new data (this would interface with GUI toolkit)"""
        # This is a placeholder - in real implementation, this would update a Matplotlib,
        # PyQtGraph, or other GUI toolkit plot
        print(f"Updating plot with {len(voltage_data)} voltage and {len(temp_data)} temperature lines")
        
        # Example of what real implementation might look like:
        # for (module_id, cell_id), (timestamps, values) in voltage_data.items():
        #     self.voltage_plot_lines[(module_id, cell_id)].setData(timestamps, values)
        
        # for (module_id, cell_id), (timestamps, values) in temp_data.items():
        #     self.temp_plot_lines[(module_id, cell_id)].setData(timestamps, values)
    
    def stop(self):
        """Stop the GUI updater thread"""
        self.running = False


# Usage example
if __name__ == "__main__":
    # Create the sensor buffer
    sensor_buffer = ModularSensorBuffer(buffer_size=600)
    
    # Create and start producer thread (simulates CAN bus)
    producer = SensorDataProducer(sensor_buffer)
    producer.start()
    
    # Create and start GUI updater thread
    gui_updater = GUIUpdater(sensor_buffer)
    gui_updater.start()
    
    try:
        # Main thread can handle user input or other tasks
        while True:
            command = input("Press q to quit: ")
            if command.lower() == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        producer.stop()
        gui_updater.stop()
        producer.join(timeout=1.0)
        gui_updater.join(timeout=1.0)
        print("Shutdown complete")