import logging

from .datatypes import CANChannel, CANMessage
from .driver import Driver


class Middleware:
    def __init__(self) -> None:
        self._logger = logging.getLogger(__name__)
        self._driver = Driver()

        self._can_channels: dict[str, CANChannel] = {}
        self._can_buffer = self._driver.can_buffer()
        self._can_notifier = self._driver.can_notifier(
            buses=[], buffer=self._can_buffer
        )

    @property
    def can_channels(self) -> dict[str, CANChannel]:
        return self._can_channels

    def start(self) -> None:
        self._logger.info("Initializing middleware.")
        self._logger.info("Middleware set up.")

    def stop(self) -> None:
        self._logger.info("Stopping middleware.")

        for channel in self._can_channels.copy():
            self.shutdown_can_channel(channel)

        self._logger.info("Middleware stopped.")

    def setup_vcan(
        self, dbc_filename: str, *, receive_own_messages: bool = False
    ) -> CANChannel:
        self._logger.info(
            f"Initializing VCAN bus with dbc_filename '{dbc_filename}' "
            + f"and 'receive_own_messages' flag set to {receive_own_messages}.",
        )

        bus = self._driver.vcan_bus(receive_own_messages=receive_own_messages)
        dbc = self._driver.can_dbc(dbc_filename)
        can_channel = CANChannel(bus, dbc)
        self._logger.info("VCAN bus initialized.")

        self._logger.info("Adding VCAN bus to middleware's buses set and notifier.")
        self._can_channels[str(can_channel.bus.channel)] = can_channel
        self._can_notifier.add_bus(bus)
        self._logger.info("VCAN bus added.")

        return can_channel

    def setup_can(
        self,
        channel: str,
        bitrate: int,
        dbc_filename: str,
        *,
        receive_own_messages: bool = False,
    ) -> CANChannel:
        self._logger.debug(
            f"Initializing CAN bus with channel {channel}, "
            + f"bitrate {bitrate}, "
            + f"dbc_filename '{dbc_filename}' and "
            + f"'receive_own_messages' flag set to {receive_own_messages}."
        )

        bus = self._driver.can_bus(
            channel, bitrate, receive_own_messages=receive_own_messages
        )
        dbc = self._driver.can_dbc(dbc_filename)
        can_channel = CANChannel(bus, dbc)
        self._logger.info("CAN bus initialized.")

        self._logger.info("Adding CAN bus to middleware's buses set and notifier.")
        self._can_channels[str(channel)] = can_channel
        self._can_notifier.add_bus(bus)
        self._logger.info("CAN bus added.")

        return can_channel

    def setup_can_fd(
        self,
        channel: str,
        arbitration_bitrate: int,
        data_bitrate: int,
        dbc_filename: str,
        *,
        receive_own_messages: bool = False,
    ) -> CANChannel:
        self._logger.debug(
            f"Initializing CAN FD bus with channel {channel}, "
            + f"arbitration bitrate {arbitration_bitrate}, "
            + f"data bitrate {data_bitrate}, dbc_filename '{dbc_filename}' and "
            + f"'receive_own_messages' flag set to {receive_own_messages}."
        )

        bus = self._driver.can_fd_bus(
            channel,
            arbitration_bitrate,
            data_bitrate,
            receive_own_messages=receive_own_messages,
        )
        dbc = self._driver.can_dbc(dbc_filename)
        can_channel = CANChannel(bus=bus, dbc=dbc)
        self._logger.info("CAN FD bus initialized.")

        self._logger.info("Adding CAN FD bus to middleware's buses set and notifier.")
        self._can_channels[str(channel)] = can_channel
        self._can_notifier.add_bus(bus)
        self._logger.info("CAN FD bus added.")

        return can_channel

    def shutdown_can_channel(self, channel: str) -> None:
        self._logger.info(f"Shutting down CAN bus {channel}.")
        self._driver.shutdown_can_bus(self._can_channels[channel].bus)
        self._logger.info(f"CAN bus {channel} shut down.")

        self._logger.info(
            f"Removing CAN bus {channel} from middleware's set and notifier."
        )
        self._can_channels.pop(channel)
        self._logger.info(f"CAN bus {channel} removed.")

    def receive_can_message(
        self, *, timeout: float | None = None
    ) -> tuple[int | None, CANMessage | None, str | None]:
        can_message = self._driver.receive_can_message(
            can_buffer=self._can_buffer, timeout=timeout
        )

        if can_message is not None:
            self._logger.info(
                f"Retrieved message from channel {can_message.channel} "
                + f"with frame id {can_message.arbitration_id}.",
            )

            self._logger.info("Decoding CAN message.")

            try:
                decoded_data = self._driver.decode_can_message(
                    can_message,
                    can_dbc=self._can_channels[str(can_message.channel)].dbc,
                )
            except KeyError:
                self._logger.warning(
                    "The retrieved message cannot be decoded with the actual DBC."
                )
                return None, None, None

            self._logger.info(f"Decoded CAN message: {decoded_data}.")

            return (can_message.arbitration_id, decoded_data, str(can_message.channel))

        else:
            return None, None, None

    def send_can_message(
        self, data: CANMessage, can_frame_id: int, can_channel: CANChannel
    ) -> None:
        self._logger.info(
            f"Sending message '{data}' "
            + f"with frame id {can_frame_id} to CAN bus {can_channel.bus}.",
        )

        self._driver.send_can_message(
            data,
            can_frame_id=can_frame_id,
            can_bus=can_channel.bus,
            can_dbc=can_channel.dbc,
        )

        self._logger.info("Message sent.")
