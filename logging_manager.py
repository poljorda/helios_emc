import os
import csv
import shutil
import threading
import queue
import tempfile
import time
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
import numpy as np

import can
from can.io import BLFWriter

# Type definition for sensor data entry
SensorDataEntry = Tuple[int, int, float, float]  # module_id, cell_id, timestamp, value

class LoggingManager:
    """Manages logging of sensor data and raw CAN messages to files."""
    
    def __init__(self, num_voltage_modules: int, num_voltage_cells: int, 
                 num_temp_modules: int, num_temp_cells: int):
        """Initialize the logging manager.
        
        Args:
            num_voltage_modules: Number of voltage modules to monitor
            num_voltage_cells: Number of cells per voltage module
            num_temp_modules: Number of temperature modules to monitor
            num_temp_cells: Number of cells per temperature module
        """
        self.num_voltage_modules = num_voltage_modules
        self.num_voltage_cells = num_voltage_cells
        self.num_temp_modules = num_temp_modules
        self.num_temp_cells = num_temp_cells
        
        # Create temporary directory for storing files while logging
        self.temp_dir = tempfile.mkdtemp(prefix="helios_log_")
        
        # Create queues for voltage and temperature data
        self.voltage_queue: queue.Queue[Tuple[int, int, float, float]] = queue.Queue()
        self.temp_queue: queue.Queue[Tuple[int, int, float, float]] = queue.Queue()
        self.can_message_queue: queue.Queue[can.Message] = queue.Queue()
        self.vehicle_can_message_queue: queue.Queue[can.Message] = queue.Queue()  # New queue for vehicle CAN
        
        # Structures to hold file handles for CSV files
        self.voltage_files: Dict[Tuple[int, int], Tuple[Any, Any]] = {}  # (module_id, cell_id) -> (file_obj, csv_writer)
        self.temp_files: Dict[Tuple[int, int], Tuple[Any, Any]] = {}  # (module_id, cell_id) -> (file_obj, csv_writer)
        
        # BLF writer for raw CAN data
        self.blf_writer: Optional[BLFWriter] = None
        self.blf_path: Optional[str] = None
        
        # BLF writer for raw vehicle CAN data
        self.vehicle_blf_writer: Optional[BLFWriter] = None
        self.vehicle_blf_path: Optional[str] = None
        
        # Thread for processing queued data and writing to files
        self.logging_thread: Optional[threading.Thread] = None
        self.running = False
        
        # Track if we've created the folder structure
        self.structure_created = False
        
        # Lock for thread safety
        self.lock = threading.RLock()
        
    def _create_folder_structure(self) -> None:
        """Create folder structure for logs."""
        with self.lock:
            # Main data type folders
            voltage_dir = os.path.join(self.temp_dir, "Voltage")
            temp_dir = os.path.join(self.temp_dir, "Temperature")
            os.makedirs(voltage_dir, exist_ok=True)
            os.makedirs(temp_dir, exist_ok=True)
            
            # Create module folders for voltage
            for module_id in range(self.num_voltage_modules):
                module_dir = os.path.join(voltage_dir, f"Module_{module_id:02d}")
                os.makedirs(module_dir, exist_ok=True)
                
                # Create CSV file for each cell in this module
                for cell_id in range(self.num_voltage_cells):
                    file_path = os.path.join(module_dir, f"voltage_{module_id:02d}_{cell_id:02d}.csv")
                    file_obj = open(file_path, 'w', newline='')
                    csv_writer = csv.writer(file_obj)
                    csv_writer.writerow(["timestamp", "value"])  # Write header
                    self.voltage_files[(module_id, cell_id)] = (file_obj, csv_writer)
            
            # Create module folders for temperature
            for module_id in range(self.num_temp_modules):
                module_dir = os.path.join(temp_dir, f"Module_{module_id:02d}")
                os.makedirs(module_dir, exist_ok=True)
                
                # Create CSV file for each cell in this module
                for cell_id in range(self.num_temp_cells):
                    file_path = os.path.join(module_dir, f"temperature_{module_id:02d}_{cell_id:02d}.csv")
                    file_obj = open(file_path, 'w', newline='')
                    csv_writer = csv.writer(file_obj)
                    csv_writer.writerow(["timestamp", "value"])  # Write header
                    self.temp_files[(module_id, cell_id)] = (file_obj, csv_writer)
            
            # Create the BLF file for raw CAN data
            self.blf_path = os.path.join(self.temp_dir, "iot_can_raw.blf")
            self.blf_writer = BLFWriter(self.blf_path)
            
            # Create the BLF file for raw vehicle CAN data
            self.vehicle_blf_path = os.path.join(self.temp_dir, "vehicle_can_raw.blf")
            self.vehicle_blf_writer = BLFWriter(self.vehicle_blf_path)
            
            self.structure_created = True
        
    def _write_voltage_data(self, module_id: int, cell_id: int, timestamp: float, value: float) -> None:
        """Write a voltage data point to its CSV file."""
        with self.lock:
            if (module_id, cell_id) in self.voltage_files:
                _, writer = self.voltage_files[(module_id, cell_id)]
                # Convert milliseconds to seconds for better readability
                timestamp_sec = timestamp / 1000.0
                # Format: timestamp in seconds since epoch, value
                writer.writerow([timestamp_sec, value])
    
    def _write_temp_data(self, module_id: int, cell_id: int, timestamp: float, value: float) -> None:
        """Write a temperature data point to its CSV file."""
        with self.lock:
            if (module_id, cell_id) in self.temp_files:
                _, writer = self.temp_files[(module_id, cell_id)]
                # Convert milliseconds to seconds for better readability
                timestamp_sec = timestamp / 1000.0
                # Format: timestamp in seconds since epoch, value
                writer.writerow([timestamp_sec, value])
    
    def _write_can_message(self, msg: can.Message) -> None:
        """Write a CAN message to the BLF file."""
        with self.lock:
            if self.blf_writer:
                self.blf_writer.on_message_received(msg)
    
    def _write_vehicle_can_message(self, msg: can.Message) -> None:
        """Write a vehicle CAN message to the vehicle BLF file."""
        with self.lock:
            if self.vehicle_blf_writer:
                self.vehicle_blf_writer.on_message_received(msg)
    
    def _logging_thread_func(self) -> None:
        """Background thread to process queued data and write to files."""
        while self.running:
            # Process voltage data
            try:
                while True:  # Process all available items
                    module_id, cell_id, timestamp, value = self.voltage_queue.get_nowait()
                    self._write_voltage_data(module_id, cell_id, timestamp, value)
                    self.voltage_queue.task_done()
            except queue.Empty:
                pass
            
            # Process temperature data
            try:
                while True:  # Process all available items
                    module_id, cell_id, timestamp, value = self.temp_queue.get_nowait()
                    self._write_temp_data(module_id, cell_id, timestamp, value)
                    self.temp_queue.task_done()
            except queue.Empty:
                pass
            
            # Process raw CAN messages
            try:
                while True:  # Process all available items
                    msg = self.can_message_queue.get_nowait()
                    self._write_can_message(msg)
                    self.can_message_queue.task_done()
            except queue.Empty:
                pass
                
            # Process raw vehicle CAN messages
            try:
                while True:  # Process all available items
                    msg = self.vehicle_can_message_queue.get_nowait()
                    self._write_vehicle_can_message(msg)
                    self.vehicle_can_message_queue.task_done()
            except queue.Empty:
                pass
            
            # Sleep briefly to avoid CPU overuse
            time.sleep(0.01)
    
    def start_logging(self) -> bool:
        """Start the logging process.
        
        Returns:
            bool: True if logging started successfully, False otherwise
        """
        if not self.running:
            try:
                if not self.structure_created:
                    self._create_folder_structure()
                
                self.running = True
                self.logging_thread = threading.Thread(target=self._logging_thread_func)
                self.logging_thread.daemon = True
                self.logging_thread.start()
                return True
            except Exception as e:
                print(f"Error starting logging: {e}")
                self.cleanup()
                return False
        return False  # Already running
    
    def stop_logging(self) -> str:
        """Stop the logging process and return the path to the temp directory.
        
        Returns:
            str: Path to the temporary directory containing log files
        """
        if self.running:
            self.running = False
            
            # Wait for the logging thread to finish
            if self.logging_thread and self.logging_thread.is_alive():
                self.logging_thread.join(timeout=2.0)
            
            with self.lock:
                # Close all voltage files
                for file_obj, _ in self.voltage_files.values():
                    file_obj.close()
                self.voltage_files.clear()
                
                # Close all temperature files
                for file_obj, _ in self.temp_files.values():
                    file_obj.close()
                self.temp_files.clear()
                
                # Close the BLF writer
                if self.blf_writer:
                    self.blf_writer.stop()
                    self.blf_writer = None
                
                # Close the vehicle BLF writer
                if self.vehicle_blf_writer:
                    self.vehicle_blf_writer.stop()
                    self.vehicle_blf_writer = None
            
            # Create a summary file
            self._create_summary_file()
        
        return self.temp_dir
    
    def _create_summary_file(self) -> None:
        """Create a summary file with metadata about the logging session."""
        summary_path = os.path.join(self.temp_dir, "log_summary.txt")
        try:
            with open(summary_path, 'w') as f:
                f.write(f"HELIOS BMS Logging Session\n")
                f.write(f"Stopped at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Configuration:\n")
                f.write(f"  Voltage Modules: {self.num_voltage_modules}\n")
                f.write(f"  Voltage Cells per Module: {self.num_voltage_cells}\n")
                f.write(f"  Temperature Modules: {self.num_temp_modules}\n")
                f.write(f"  Temperature Cells per Module: {self.num_temp_cells}\n")
        except Exception as e:
            print(f"Error creating summary file: {e}")
    
    def move_logs_to_destination(self, destination_path: str) -> bool:
        """Move logs from temporary directory to final destination.
        
        Args:
            destination_path: Path to move logs to
            
        Returns:
            bool: True if move was successful, False otherwise
        """
        try:
            # Ensure destination exists
            os.makedirs(os.path.dirname(destination_path), exist_ok=True)
            
            # Copy contents of temp_dir to destination
            if os.path.exists(destination_path):
                # If destination exists, merge the contents
                for item in os.listdir(self.temp_dir):
                    src_path = os.path.join(self.temp_dir, item)
                    dst_path = os.path.join(destination_path, item)
                    if os.path.isdir(src_path):
                        shutil.copytree(src_path, dst_path)
                    else:
                        shutil.copy2(src_path, dst_path)
            else:
                # If destination doesn't exist, simply move the directory
                shutil.move(self.temp_dir, destination_path)
            
            return True
        except Exception as e:
            print(f"Error moving logs to destination: {e}")
            return False
    
    def log_voltage(self, module_id: int, cell_id: int, timestamp: float, value: float) -> None:
        """Add voltage data to the logging queue."""
        if self.running:
            self.voltage_queue.put((module_id, cell_id, timestamp, value))
    
    def log_temperature(self, module_id: int, cell_id: int, timestamp: float, value: float) -> None:
        """Add temperature data to the logging queue."""
        if self.running:
            self.temp_queue.put((module_id, cell_id, timestamp, value))
    
    def log_can_message(self, msg: can.Message) -> None:
        """Add a CAN message to the logging queue."""
        if self.running:
            self.can_message_queue.put(msg)
    
    def log_vehicle_can_message(self, msg: can.Message) -> None:
        """Add a vehicle CAN message to the vehicle logging queue."""
        if self.running:
            self.vehicle_can_message_queue.put(msg)
    
    def cleanup(self) -> None:
        """Clean up resources and temporary files."""
        # Stop logging if still running
        if self.running:
            self.stop_logging()
        
        # Try to remove the temporary directory if it exists
        try:
            if os.path.exists(self.temp_dir):
                shutil.rmtree(self.temp_dir)
        except Exception as e:
            print(f"Error cleaning up temporary directory: {e}")

class LoggingCanMessageObserver:
    """Observer that receives CAN messages and forwards them to a LoggingManager."""
    
    def __init__(self, logging_manager: LoggingManager):
        """Initialize the observer.
        
        Args:
            logging_manager: LoggingManager to forward messages to
        """
        self.logging_manager = logging_manager
    
    def log_message(self, msg: can.Message) -> None:
        """Forward received messages to the logging manager."""
        self.logging_manager.log_can_message(msg)

class LoggingVehicleCanMessageObserver:
    """Observer that receives vehicle CAN messages and forwards them to a LoggingManager."""
    
    def __init__(self, logging_manager: LoggingManager):
        """Initialize the observer.
        
        Args:
            logging_manager: LoggingManager to forward messages to
        """
        self.logging_manager = logging_manager
    
    def log_message(self, msg: can.Message) -> None:
        """Forward received messages to the logging manager."""
        self.logging_manager.log_vehicle_can_message(msg)
