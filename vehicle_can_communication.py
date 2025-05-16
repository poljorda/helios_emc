from datetime import datetime
import os
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple, Union

import can
import cantools

from cantools.database import Database as CantoolsDatabase # type: ignore[attr-defined]

from logging_manager import LoggingVehicleCanMessageObserver

DBC_FILE_PATH = r"resources\Vehicle_CAN_V2.1.dbc"  # Make sure this path is correct
KVASER_INTERFACE = "kvaser"
# Use channel 0 for virtual simulation, maybe 1 for real hardware? Make configurable?
KVASER_CHANNEL = 2
IS_VIRTUAL = False  # Set True for testing with simulator

# CAN 2.0 bit timing
ARBITRATION_BITRATE = 500000  # 500 kbps
BRP = 1
TSEG1 = 12
TSEG2 = 3
SJW = 2

SEND_MESSAGE_ID = 0x25
RECEIVE_MESSAGE_ID = 0x19
RECEIVE_MESSAGE_MSG_NAME = "Battery_Info"
SEND_PERIOD_S = 1  # Send every 1s

class SharedValue:
    def __init__(self, initial_value: bool):
        self._value = initial_value
        self._lock = threading.Lock()

    def get(self) -> bool:
        with self._lock:
            return self._value

    def set(self, new_value: bool) -> None:
        with self._lock:
            self._value = new_value

# --- Custom CAN Listener ---
class MySignalListener(can.Listener):
    """A custom CAN listener to process specific messages."""
    target_id: int
    shared_value_store: SharedValue

    def __init__(self, db: CantoolsDatabase, shared_value_store: SharedValue, response_callback: Optional[Callable[[str], None]]) -> None:
        super().__init__()
        self.db = db
        self.shared_value_store = shared_value_store
        self.response_callback = response_callback
        try:
            self._message_definition = self.db.get_message_by_name(RECEIVE_MESSAGE_MSG_NAME)
            print(f"Listener initialized for message '{RECEIVE_MESSAGE_MSG_NAME}' (ID: {hex(self._message_definition.frame_id)}).")
        except KeyError:
            self._message_definition = None
            print(f"Listener WARNING: Message name '{RECEIVE_MESSAGE_MSG_NAME}' not found in DBC. Listener will not decode this message.")

    def on_message_received(self, msg: can.Message) -> None:
        """Called when a message is received on the bus."""
        if self._message_definition is None or msg.arbitration_id != self._message_definition.frame_id:
            # Not the message we are configured to decode, or DBC message definition was not found initially
            return

        try:
            # Decode the message using the DBC database
            decoded_signals: Dict[str, Any] = self.db.decode_message(
                frame_id_or_name=self._message_definition.frame_id, # Use frame_id for direct lookup
                data=msg.data,
                decode_choices=True,  # Decodes choice values to their string representations
            )

            # CAN timestamp is in seconds, convert to datetime object
            dt_object = datetime.fromtimestamp(msg.timestamp)
            formatted_timestamp = dt_object.strftime("%H:%M:%S") # Format to HH:MM:SS

            self._update_response(
                f"Battery_State: {decoded_signals['Battery_State']}. "
                f"Battery_Failure: {decoded_signals['Battery_Failure']}. "
                f"Timestamp: {formatted_timestamp}"
            )

        except cantools.database.errors.DecodeError as e:
            print(f"Listener: DBC DecodeError for ID {hex(msg.arbitration_id)} ('{self._message_definition.name}'): {e}")
        except Exception as e: # Catch any other unexpected errors during decoding
            print(f"Listener: Unexpected error processing message ID {hex(msg.arbitration_id)} ('{self._message_definition.name}') with DBC: {e}")

    def on_error(self, exc: Exception) -> None:
        """Called when an error occurs in the Notifier's processing of this Listener."""
        print(f"Listener: An error occurred: {exc}")
        
    def _update_response(self, message: str) -> None:
        """Safely updates the response via callback."""
        print(f"CAN Response: {message}")
        if self.response_callback:
            # If response_callback needs to interact with Tkinter GUI from this thread,
            # it should use root.after() or a thread-safe queue.
            # For simplicity, we assume the callback itself handles thread safety if needed.
            self.response_callback(message)

class VehicleCanCommsThread(threading.Thread):
    """Reads CAN messages, decodes them, and updates the sensor buffer."""

    def __init__(
        self, status_callback: Optional[Callable[[str, bool], None]] = None, response_callback: Optional[Callable[[str], None]] = None
    ):
        super().__init__()
        self.status_callback: Optional[Callable[[str, bool], None]] = status_callback
        self.response_callback: Optional[Callable[[str], None]] = response_callback
        self.running: bool = False
        self.db: Optional[cantools.db.Database] = None  # type: ignore
        self.bus: Optional[can.BusABC] = None
        
        # Flag to activate or disable communications (Vehicle_Status signal value)
        self.vehicle_status: SharedValue = SharedValue(False)  # Default to False (disabled)
        
        self.vehicle_status_start_time: Optional[float] = None  # Start time for vehicle status timer
        self.logging_observer: Optional[LoggingVehicleCanMessageObserver] = None  # Add this attribute for logging

        self.daemon: bool = True

    def set_logging_observer(self, observer: Union[LoggingVehicleCanMessageObserver | None]) -> None:
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
            bit_timing = can.BitTiming(f_clock=16_000_000, brp=BRP, tseg1=TSEG1, tseg2=TSEG2, sjw=SJW, strict=True)
            
            self.bus = can.interface.Bus(
                interface=KVASER_INTERFACE,
                channel=str(KVASER_CHANNEL),  # Ensure channel is a string for python-can
                fd=False,
                bitrate=ARBITRATION_BITRATE,
                timing=bit_timing,
                is_virtual=IS_VIRTUAL,
            )
            
            signal_listener = MySignalListener(self.db, self.vehicle_status, self.response_callback)
            self.notifier = can.Notifier(self.bus, [signal_listener])
            
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
    
    def run(self) -> None:
        """Main thread loop to read and process CAN messages."""
        if not self._load_dbc() or not self._init_can_bus():
            self.running = False
            self._update_status("VehicleCanComms Thread: Initialization failed. Thread stopping.", is_error=True)
            return

        self.running = True
        self._update_status("VehicleCanComms Thread: Running.")
        
        while self.running:
            try:
                if self.db:
                    # Create the message data with Vehicle_Mode set according to status
                    message_data = self.db.encode_message(
                        'Vehicle_Status',  # Message name from DBC
                        {
                            'Vehicle_Mode': 1 if self.vehicle_status.get() else 0,
                            'Vehicle_Speed': 0,
                            'Emergency_Stop': 0,
                            'Fault_Reset':0
                        }
                    )
                    
                    # Create and send the CAN message
                    msg = can.Message(
                        arbitration_id=SEND_MESSAGE_ID,
                        data=message_data,
                        is_extended_id=False
                    )
                    if self.bus:
                        self.bus.send(msg)
                        
                        # If we have a logging observer, log the sent message
                        if self.logging_observer:
                            self.logging_observer.log_message(msg)
                    
                time.sleep(SEND_PERIOD_S)
            except Exception as e:
                self._update_status(f"Error sending CAN message: {e}", is_error=True)
                break
        
        if self.bus:
            try:
                # Close the notifier first to prevent any more callbacks
                self.notifier.stop()
                self.bus.shutdown()
                self._update_status("CAN bus shutdown.")
            except Exception as e:
                self._update_status(f"Error shutting down CAN bus: {e}", is_error=True)
        self._update_status(f"VehicleCanComms thread stopped. Running state: {self.running}")

    def stop(self) -> None:
        """Signals the thread to stop."""
        self._update_status("VehicleCanComms Thread: Stop requested.")
        self.running = False

