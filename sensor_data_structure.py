import numpy as np
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List, Any  # Added Optional, Tuple, Dict, List, Any
from numpy import ndarray  # Added ndarray


@dataclass
class SensorReading:
    """Simple structure to hold a single sensor reading"""

    timestamp: float  # Unix timestamp in milliseconds (as float for precision)
    value: float  # Sensor value (voltage or temperature)


class ModularSensorBuffer:
    """
    Thread-safe circular buffer for voltage and temperature sensors organized in modules and cells.
    Optimized for efficient memory usage and fast plot data retrieval.
    """

    buffer_size: int
    num_voltage_modules: int
    num_voltage_cells: int
    num_temp_modules: int
    num_temp_cells: int
    voltage_data: ndarray[Any, Any]  # np.float64
    temp_data: ndarray[Any, Any]  # np.float64
    voltage_indices: ndarray[Any, Any]  # np.int32
    temp_indices: ndarray[Any, Any]  # np.int32
    voltage_full: ndarray[Any, Any]  # bool
    temp_full: ndarray[Any, Any]  # bool
    voltage_lock: threading.RLock
    temp_lock: threading.RLock
    last_voltage_update: float
    last_temp_update: float
    data_ready: threading.Event

    def __init__(self, buffer_size: int = 600) -> None:
        self.buffer_size = buffer_size

        # Define sensor structure dimensions
        self.num_voltage_modules = 5
        self.num_voltage_cells = 16
        self.num_temp_modules = 5
        self.num_temp_cells = 6

        # Pre-allocate numpy arrays for timestamps and values
        # Using numpy structured arrays for improved memory usage and access speed
        # For timestamps, use float64 to ensure compatibility with datetime operations
        self.voltage_data = np.zeros(
            (self.num_voltage_modules, self.num_voltage_cells, buffer_size, 2), dtype=np.float64
        )
        self.temp_data = np.zeros((self.num_temp_modules, self.num_temp_cells, buffer_size, 2), dtype=np.float64)

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

    def update_voltage(self, module_id: int, cell_id: int, timestamp: float, value: float) -> bool:
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

    def update_temperature(self, module_id: int, cell_id: int, timestamp: float, value: float) -> bool:
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

    def get_voltage_data(
        self, module_id: int, cell_id: int
    ) -> Tuple[Optional[ndarray[Any, Any]], Optional[ndarray[Any, Any]]]:
        """Get all data points for a specific voltage sensor in plot-ready format"""
        if not (0 <= module_id < self.num_voltage_modules and 0 <= cell_id < self.num_voltage_cells):
            return None, None

        with self.voltage_lock:
            idx = self.voltage_indices[module_id, cell_id]
            is_full = self.voltage_full[module_id, cell_id]

            if is_full:
                # If buffer is full, we need to reorder data to get chronological order
                timestamps = np.concatenate(
                    [self.voltage_data[module_id, cell_id, idx:, 0], self.voltage_data[module_id, cell_id, :idx, 0]]
                )
                values = np.concatenate(
                    [self.voltage_data[module_id, cell_id, idx:, 1], self.voltage_data[module_id, cell_id, :idx, 1]]
                )
            else:
                # If not full, just get the data up to current index
                timestamps = self.voltage_data[module_id, cell_id, :idx, 0]
                values = self.voltage_data[module_id, cell_id, :idx, 1]

            return timestamps, values

    def get_temperature_data(
        self, module_id: int, cell_id: int
    ) -> Tuple[Optional[ndarray[Any, Any]], Optional[ndarray[Any, Any]]]:
        """Get all data points for a specific temperature sensor in plot-ready format"""
        if not (0 <= module_id < self.num_temp_modules and 0 <= cell_id < self.num_temp_cells):
            return None, None

        with self.temp_lock:
            idx = self.temp_indices[module_id, cell_id]
            is_full = self.temp_full[module_id, cell_id]

            if is_full:
                # If buffer is full, we need to reorder data to get chronological order
                timestamps = np.concatenate(
                    [self.temp_data[module_id, cell_id, idx:, 0], self.temp_data[module_id, cell_id, :idx, 0]]
                )
                values = np.concatenate(
                    [self.temp_data[module_id, cell_id, idx:, 1], self.temp_data[module_id, cell_id, :idx, 1]]
                )
            else:
                # If not full, just get the data up to current index
                timestamps = self.temp_data[module_id, cell_id, :idx, 0]
                values = self.temp_data[module_id, cell_id, :idx, 1]

            return timestamps, values

    def get_latest_voltage_values(self) -> ndarray[Any, Any]:  # np.float32
        """Get the most recent voltage values as a 2D array (module x cell)"""
        with self.voltage_lock:
            latest_values: ndarray[Any, Any] = np.zeros(
                (self.num_voltage_modules, self.num_voltage_cells), dtype=np.float32
            )

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

    def get_latest_temperature_values(self) -> ndarray[Any, Any]:  # np.float32
        """Get the most recent temperature values as a 2D array (module x cell)"""
        with self.temp_lock:
            latest_values: ndarray[Any, Any] = np.zeros((self.num_temp_modules, self.num_temp_cells), dtype=np.float32)

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

    def wait_for_data(self, timeout: float = 1.0) -> bool:
        """Wait for new data to be available, returns True if new data available, False on timeout"""
        result: bool = self.data_ready.wait(timeout)
        if result:
            # Reset the event for next wait
            self.data_ready.clear()
        return result
