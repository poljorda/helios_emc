import logging
import subprocess
import sys

import can
import cantools
from can.interfaces.kvaser.canlib import KvaserBus
from can.interfaces.socketcan import SocketcanBus

from .constants import (
    CAN_ALREADY_SETUP_STDERR,
    LINUX_VIRTUAL_CAN_CHANNEL,
    VIRTUAL_CAN_ALREADY_SETUP_STDERR,
    WINDOWS_VIRTUAL_CAN_CHANNEL,
)
from .datatypes import (
    CANBuffer,
    CANBus,
    CANDatabase,
    CANMessage,
    CANNotifier,
    IncompatibleOSException,
    RawCANMessage,
)


class Driver:
    def _run_command(self, command: str) -> None:
        try:
            self._logger.debug(f"Executing command '{command}'.")
            subprocess.run(command.split(), capture_output=True, check=True)
            self._logger.debug("Command executed.")

        except subprocess.CalledProcessError as error:
            if error.returncode != 2 or error.stderr not in (
                VIRTUAL_CAN_ALREADY_SETUP_STDERR,
                CAN_ALREADY_SETUP_STDERR,
            ):
                raise

    def __init__(self) -> None:
        self._logger = logging.getLogger(__name__)
        self._logger.debug("Initializing CAN modules.")

        if sys.platform == "linux":
            self._run_command("sudo modprobe vcan")
            self._run_command("sudo modprobe can")
            self._run_command("sudo modprobe can-raw")

        self._logger.debug("CAN modules initialized.")

    def vcan_bus(self, *, receive_own_messages: bool = False) -> CANBus:
        self._logger.debug(
            "Initializing VCAN bus with 'receive_own_messages' "
            + f"flag set to {LINUX_VIRTUAL_CAN_CHANNEL}."
        )

        if sys.platform == "linux":
            self._run_command(
                f"sudo ip link add dev {LINUX_VIRTUAL_CAN_CHANNEL} type vcan"
            )
            self._run_command(f"sudo ip link set up {LINUX_VIRTUAL_CAN_CHANNEL}")

        if sys.platform == "linux":
            bus = SocketcanBus(
                channel=LINUX_VIRTUAL_CAN_CHANNEL,
                receive_own_messages=receive_own_messages,
            )
        elif sys.platform == "win32":
            bus = KvaserBus(
                int(WINDOWS_VIRTUAL_CAN_CHANNEL),
                receive_own_messages=receive_own_messages,
            )
        else:
            raise IncompatibleOSException()

        self._logger.debug("VCAN bus initialized.")
        return bus

    def can_bus(
        self, channel: str, bitrate: int, *, receive_own_messages: bool = False
    ) -> CANBus:
        self._logger.debug(
            f"Initializing CAN bus with channel {channel}, bitrate {bitrate} "
            f"and 'receive_own_messages' flag set to {receive_own_messages}.",
        )

        if sys.platform == "linux":
            self._run_command(f"sudo ip link set {channel} type can bitrate {bitrate}")
            self._run_command(f"sudo ip link set up {channel}")

        if sys.platform == "linux":
            bus = SocketcanBus(
                channel=channel,
                receive_own_messages=receive_own_messages,
            )
        elif sys.platform == "win32":
            bus = KvaserBus(
                int(channel), bitrate=bitrate, receive_own_messages=receive_own_messages
            )
        else:
            raise IncompatibleOSException()

        self._logger.debug("CAN bus initialized.")
        return bus

    def can_fd_bus(
        self,
        channel: str,
        arbitration_bitrate: int,
        data_bitrate: int,
        *,
        receive_own_messages: bool = False,
    ) -> CANBus:
        self._logger.debug(
            f"Initializing CAN FD bus with channel {channel}, arbitration bitrate {arbitration_bitrate}, "
            f"data bitrate {data_bitrate} and 'receive_own_messages' flag set to {receive_own_messages}.",
        )

        if sys.platform == "linux":
            self._run_command(
                f"sudo ip link set {channel} type can "
                + f"bitrate {arbitration_bitrate} dbitrate {data_bitrate} fd on"
            )
            self._run_command(f"sudo ip link set up {channel}")

        if sys.platform == "linux":
            bus = SocketcanBus(
                channel=channel, receive_own_messages=receive_own_messages, fd=True
            )
        elif sys.platform == "win32":
            bus = KvaserBus(
                int(channel),
                bitrate=arbitration_bitrate,
                receive_own_messages=receive_own_messages,
                fd=True,
                data_bitrate=data_bitrate,
            )
        else:
            raise IncompatibleOSException()

        self._logger.debug("CAN FD bus initialized.")
        return bus

    def shutdown_can_bus(self, can_bus: CANBus) -> None:
        self._logger.debug(f"Shutting down CAN bus {can_bus}.")
        can_bus.shutdown()
        self._logger.debug("CAN bus shut down")

    def can_dbc(self, dbc_filename: str) -> CANDatabase:
        self._logger.debug(f"Initializing CAN DBC from filename '{dbc_filename}'.")

        can_dbc = cantools.db.load_file(dbc_filename)
        assert isinstance(can_dbc, CANDatabase)

        self._logger.debug("CAN DBC Initialized.")
        return can_dbc

    def can_buffer(self) -> CANBuffer:
        return can.BufferedReader()

    def can_notifier(self, buses: list[CANBus], buffer: CANBuffer) -> CANNotifier:
        self._logger.debug(
            f"Initializing CAN notifier with buses {buses} and a buffer."
        )
        notifier = can.Notifier(bus=buses, listeners=[buffer])  # type: ignore[arg-type]
        self._logger.debug("CAN notifier initialized.")

        return notifier

    def receive_can_message(
        self, can_buffer: CANBuffer, *, timeout: float | None = None
    ) -> RawCANMessage | None:
        self._logger.debug(
            "Retrieving message from CAN buffer with a timeout of %s.", str(timeout)
        )
        message = can_buffer.get_message(timeout=timeout)  # type: ignore[arg-type]

        if message is not None:
            self._logger.debug(
                f"Retrieved message from channel {message.channel} "
                + f"with frame id {message.arbitration_id}."
            )
        else:
            self._logger.debug("No CAN message was received.")

        return message

    def decode_can_message(
        self,
        can_message: RawCANMessage,
        can_dbc: CANDatabase,
    ) -> CANMessage:
        self._logger.debug(f"Decoding raw CAN message '{can_message}'.")

        decoded_data = can_dbc.decode_message(
            can_message.arbitration_id, can_message.data
        )

        self._logger.debug(f"Decoded CAN message: '{decoded_data}'.")
        return decoded_data  # type: ignore[no-any-return]

    def send_can_message(
        self,
        data: CANMessage,
        can_frame_id: int,
        can_bus: CANBus,
        can_dbc: CANDatabase,
    ) -> None:
        self._logger.debug(
            f"Sending message '{data}' with frame id {can_frame_id} to CAN bus {can_bus}.",
        )

        encoded_data = can_dbc.encode_message(can_frame_id, data)
        can_message = can.Message(
            arbitration_id=can_frame_id, data=encoded_data, is_extended_id=False
        )
        can_bus.send(can_message)
        self._logger.debug("Message sent.")
