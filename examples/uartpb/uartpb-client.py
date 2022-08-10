#!/usr/bin/env python3
import base64
import binascii
from pathlib import Path
from typing import Union

from google import protobuf
from google.protobuf import json_format
from serial import Serial
import typer

from PB_Color_pb2 import PB_Color
from PB_ReqHello_pb2 import PB_ReqHello
from PB_ReqPing_pb2 import PB_ReqPing
from PB_ReqPong_pb2 import PB_ReqPong
from PB_RespHello_pb2 import PB_RespHello
from PB_RespPing_pb2 import PB_RespPing
from PB_RespPong_pb2 import PB_RespPong
from PB_Menu_pb2 import PB_Menu


serial_port = Serial()

menu_uuid = 1485239280
reset_uuid = 3351980141
req_hello_uuid = 3938291130
req_ping_uuid = 2537089183
req_pong_uuid = 3650317449
resp_hello_uuid = 4187249687
resp_ping_uuid = 4288740491
resp_pong_uuid = 2687718320


def send_response(
        uuid: int,
        response: Union[None, PB_RespHello, PB_RespPing, PB_RespPong] = None) -> None:
    serial_port.write(uuid.to_bytes(4, "little"))
    if response:
        response_serialized = response.SerializeToString()
        response_base64 = base64.encodebytes(response_serialized)
        serial_port.write(response_base64)
    serial_port.write(b"\0")


def pb_exception_handler(func):
    def inner_function(*args, **kwargs):
        try:
            func(*args, **kwargs)
        except protobuf.message.DecodeError as exc:
            print(exc)
    return inner_function


def handle_message_reset() -> None:
    print("Reset received")


@pb_exception_handler
def handle_message_menu(message: bytes) -> None:
    menu = PB_Menu()
    menu.ParseFromString(message)
    print("Received new menu:")
    print(json_format.MessageToDict(menu))


@pb_exception_handler
def handle_message_hello(message: bytes) -> None:
    hello = PB_ReqHello()
    hello.ParseFromString(message)
    print(f"Received Hello request with number={hello.number} and message='{hello.message}'")
    response = PB_RespHello()
    response.number = -hello.number
    send_response(resp_hello_uuid, response)


@pb_exception_handler
def handle_message_ping(message: bytes) -> None:
    ping = PB_ReqPing()
    ping.ParseFromString(message)
    print(f"Received Ping request with color={PB_Color.Name(ping.color)}")
    response = PB_RespPing()
    response.color = PB_Color.BLUE
    send_response(resp_ping_uuid, response)


@pb_exception_handler
def handle_message_pong(message: bytes) -> None:
    pong = PB_ReqPong()
    pong.ParseFromString(message)
    print(f"Received Pong request with pose={{x={pong.pose.x}, y={pong.pose.y}, angle={pong.pose.O}}}")
    response = PB_RespPong()
    response.new_pose.x = -pong.pose.x
    response.new_pose.y = -pong.pose.y
    response.new_pose.O = -pong.pose.O
    send_response(resp_pong_uuid, response)


request_handlers = {
    reset_uuid: handle_message_reset,
    menu_uuid: handle_message_menu,
    req_hello_uuid: handle_message_hello,
    req_ping_uuid: handle_message_ping,
    req_pong_uuid: handle_message_pong
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
