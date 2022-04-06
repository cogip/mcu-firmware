#!/usr/bin/env python3
import base64
import binascii
from enum import IntEnum
from pathlib import Path
from typing import Union

from google import protobuf
from google.protobuf import json_format
from serial import Serial
import typer

from PB_Pose_pb2 import PB_Pose
from PB_Color_pb2 import PB_Color
from PB_ReqHello_pb2 import PB_ReqHello
from PB_ReqPing_pb2 import PB_ReqPing
from PB_ReqPong_pb2 import PB_ReqPong
from PB_RespHello_pb2 import PB_RespHello
from PB_RespPing_pb2 import PB_RespPing
from PB_RespPong_pb2 import PB_RespPong
from PB_Menu_pb2 import PB_Menu
from PB_AppOutputMessage_pb2 import PB_AppOutputMessage


class MessageType(IntEnum):
    MENU = 0
    RESET = 1
    HELLO = 2
    PING = 3
    PONG = 4


serial_port = Serial()


def send_response(message_type: MessageType, response: Union[PB_RespHello, PB_RespPing, PB_RespPong]) -> None:
    message_type_byte = int.to_bytes(message_type, 1, byteorder='little', signed=False)
    response_encoded = response.SerializeToString()
    length_bytes = int.to_bytes(len(response_encoded), 4, byteorder='little', signed=False)
    serial_port.write(message_type_byte)
    serial_port.write(length_bytes)
    serial_port.write(response_encoded)


def handle_message_reset(reset: bool) -> None:
    print("Reset received:", reset)


def handle_message_menu(menu: PB_Menu) -> None:
    print("Received new menu:")
    print(json_format.MessageToDict(menu))


def handle_message_hello(hello: PB_ReqHello) -> None:
    print(f"Received Hello request with number={hello.number} and message='{hello.message}'")
    response = PB_RespHello()
    response.number = -hello.number
    send_response(MessageType.HELLO, response)


def handle_message_ping(ping: PB_ReqPing) -> None:
    print(f"Received Ping request with color={PB_Color.Name(ping.color)}")
    response = PB_RespPing()
    response.color = PB_Color.BLUE
    send_response(MessageType.PING, response)


def handle_message_pong(pong: PB_ReqPong) -> None:
    print(f"Received Pong request with pose={{x={pong.pose.x}, y={pong.pose.y}, angle={pong.pose.O}}}")
    response = PB_RespPong(
        new_pose=PB_Pose(
            x=-pong.pose.x,
            y=-pong.pose.y,
            O=-pong.pose.O
        )
    )
    send_response(MessageType.PONG, response)


request_handlers = {
    "reset": handle_message_reset,
    "menu": handle_message_menu,
    "req_hello": handle_message_hello,
    "req_ping": handle_message_ping,
    "req_pong": handle_message_pong
}


def decode_message(encoded_message: bytes) -> None:
    try:
        message = PB_AppOutputMessage()
        message.ParseFromString(encoded_message)
    except protobuf.message.DecodeError as exc:
        print(exc)
        return -1

    message_type = message.WhichOneof("type")

    request_handler = request_handlers.get(message_type)
    if request_handler:
        request_handler(getattr(message, message_type))
    else:
        print(f"No handler found for message type '{message_type}'")


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
        # Read next base64 message
        base64_message = serial_port.readline()

        # Base64 decoding
        try:
            pb_message = base64.decodebytes(base64_message)
        except binascii.Error:
            print("Failed to decode base64 message.")
            continue

        decode_message(pb_message)


if __name__ == "__main__":
    typer.run(main)
