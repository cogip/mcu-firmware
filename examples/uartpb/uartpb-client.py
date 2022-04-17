#!/usr/bin/env python3
import base64
import binascii
from pathlib import Path

from google import protobuf
from google.protobuf import json_format
from serial import Serial
import typer

from PB_Color_pb2 import PB_Color
from PB_ReqHello_pb2 import PB_ReqHello
from PB_ReqPing_pb2 import PB_ReqPing
from PB_ReqPong_pb2 import PB_ReqPong
from PB_Menu_pb2 import PB_Menu
from PB_ExampleInputMessage_pb2 import PB_ExampleInputMessage
from PB_ExampleOutputMessage_pb2 import PB_ExampleOutputMessage


serial_port = Serial()


def send_response(response: PB_ExampleInputMessage) -> None:
    response_serialized = response.SerializeToString()
    response_base64 = base64.encodebytes(response_serialized)
    serial_port.write(response_base64)
    serial_port.write(b"\n")


def handle_message_reset(reset: bool) -> None:
    print("Reset received:", reset)


def handle_message_menu(menu: PB_Menu) -> None:
    print("Received new menu:")
    print(json_format.MessageToDict(menu))


def handle_message_hello(hello: PB_ReqHello) -> None:
    print(f"Received Hello request with number={hello.number} and message='{hello.message}'")
    response = PB_ExampleInputMessage()
    response.resp_hello.number = -hello.number
    send_response(response)


def handle_message_ping(ping: PB_ReqPing) -> None:
    print(f"Received Ping request with color={PB_Color.Name(ping.color)}")
    response = PB_ExampleInputMessage()
    response.resp_ping.color = PB_Color.BLUE
    send_response(response)


def handle_message_pong(pong: PB_ReqPong) -> None:
    print(f"Received Pong request with pose={{x={pong.pose.x}, y={pong.pose.y}, angle={pong.pose.O}}}")
    response = PB_ExampleInputMessage()
    response.resp_pong.new_pose.x = -pong.pose.x
    response.resp_pong.new_pose.y = -pong.pose.y
    response.resp_pong.new_pose.O = -pong.pose.O
    send_response(response)


request_handlers = {
    "reset": handle_message_reset,
    "menu": handle_message_menu,
    "req_hello": handle_message_hello,
    "req_ping": handle_message_ping,
    "req_pong": handle_message_pong
}


def decode_message(encoded_message: bytes) -> None:
    try:
        message = PB_ExampleOutputMessage()
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
