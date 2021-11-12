#!/usr/bin/env python3
from pathlib import Path

from google import protobuf
from serial import Serial
import typer

from pingpong_pb2 import (
    PB_Request, PB_ReqHello, PB_ReqPing, PB_ReqPong,
    PB_Response, PB_RespPong,
    PB_Pose, PB_ColorType
)


serial_port = Serial()


def send_response(response: PB_Response) -> None:
    response_encoded = response.SerializeToString()
    length_bytes = int.to_bytes(len(response_encoded), 4, byteorder='little', signed=False)
    serial_port.write(length_bytes)
    serial_port.write(response_encoded)


def handle_message_hello(hello: PB_ReqHello) -> None:
    print(f"Received Hello request with number={hello.number}")
    response = PB_Response()
    response.hello.number = -hello.number
    send_response(response)


def handle_message_ping(ping: PB_ReqPing) -> None:
    print(f"Received Ping request with color={PB_ColorType.Name(ping.color)}")
    response = PB_Response()
    response.ping.color = PB_ColorType.BLUE
    send_response(response)


def handle_message_pong(pong: PB_ReqPong) -> None:
    print(f"Received Pong request with pose={{x={pong.pose.x}, y={pong.pose.y}, angle={pong.pose.angle}}}")
    response = PB_Response()
    response_pong = PB_RespPong(
        new_pose=PB_Pose(
            x=-pong.pose.x,
            y=-pong.pose.y,
            angle=-pong.pose.angle
        )
    )
    response.pong.CopyFrom(response_pong)
    send_response(response)


request_handlers = {
    "hello": handle_message_hello,
    "ping": handle_message_ping,
    "pong": handle_message_pong
}


def decode_message(encoded_message: str) -> None:
    message = PB_Request()
    try:
        message.ParseFromString(encoded_message)
    except protobuf.message.DecodeError as exc:
        print(exc)
        return -1
    request_type = message.WhichOneof("request")
    request_handler = request_handlers.get(request_type)
    if request_handler:
        request_handler(getattr(message, request_type))
    else:
        print(f"No handler found for request type '{request_type}'")


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
        length_bytes = serial_port.readline(4)
        message_length = int.from_bytes(length_bytes, byteorder='little')
        encoded_message = serial_port.read(message_length)
        decode_message(encoded_message)


if __name__ == "__main__":
    typer.run(main)
