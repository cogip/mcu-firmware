#!/usr/bin/env python3
import base64
import binascii
from pathlib import Path

from google import protobuf
from google.protobuf import json_format
from serial import Serial
import typer

from PB_Sysmon_pb2 import PB_Sysmon

serial_port = Serial()

sysmon_uuid = 1509576639


def pb_exception_handler(func):
    def inner_function(*args, **kwargs):
        try:
            func(*args, **kwargs)
        except protobuf.message.DecodeError as exc:
            print(exc)
    return inner_function


@pb_exception_handler
def handle_message_sysmon(message: bytes) -> None:
    sysmon = PB_Sysmon()
    sysmon.ParseFromString(message)
    print("Received new Sysmon:")
    print(json_format.MessageToDict(sysmon))


request_handlers = {
    sysmon_uuid: handle_message_sysmon,
}


def handle_message(uuid: int, pb_message: bytes = b"") -> None:
    request_handler = request_handlers.get(uuid)
    if not request_handler:
        print(f"No handler found for message uuid '{uuid}'")
        return

    if not pb_message:
        request_handler()
    else:
        request_handler(pb_message)


def main(
        port: Path = typer.Argument(
            "/dev/ttyUSB0", exists=True, file_okay=True,
            help="Serial port connected to STM32 device"),
        baud: int = typer.Argument(
            230400,
            help="Baud rate")) -> None:
    serial_port.port = str(port)
    serial_port.baudrate = baud
    serial_port.open()

    while(True):
        # Read next message
        message = serial_port.readline()
        message = message.rstrip(b"\n")

        # Get message uuid on first bytes
        uuid = int.from_bytes(message[:4], "little")

        if len(message) == 4:
            handle_message(uuid)
            continue

        try:
            pb_message = base64.decodebytes(message[4:])
        except binascii.Error:
            print(f"Failed to decode base64 message (uuid={uuid}).")
            continue

        handle_message(uuid, pb_message)


if __name__ == "__main__":
    typer.run(main)
