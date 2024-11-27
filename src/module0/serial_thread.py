#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By  : Matthew Davidson
# Created Date: 2024-01-01
# version ='0.0.1'
# ---------------------------------------------------------------------------
"""a_short_module_description"""
# ---------------------------------------------------------------------------




from abc import abstractmethod

import sys

import threading

import serial

import logging

from collections import deque

import time

import traceback

 

logger = logging.getLogger(__name__)

 

class SerialThread(threading.Thread):

    def __init__(self, port=None, baudrate=9600, timeout=1, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, max_reconnect_attempts=0):

        super().__init__(name="SerialThread")

        self.port = port

        self.baudrate = baudrate

        self.bytesize=bytesize

        self.parity=parity

        self.stopbits=stopbits

        self.running = True

        self.input_buffer = deque()

        self.output_buffer = deque()

        self.output_lock = threading.Lock()

        self.input_lock = threading.Lock()

        self.max_reconnect_attempts = max_reconnect_attempts

        self.timeout = timeout

 

        self.serial = None

        self.reconnect_attempts = 0

        # self.packet_size = 80

        self._specified_port = port # Store the specified port for reconnection

 

        self.connect()

 

        self.packet_reader = WindowedPacketReader(self.serial, window_size=86, timeout=0.2)

 

    def detect_devices(

        self,

        search: str = r"ACM",

        pattern: str = "/dev/tty*",

    ):

       

        def find_devices(search, pattern):

            import glob

            import re

 

            # List all files matching the pattern /dev/tty*

            files = glob.glob(pattern)

 

            # Filter files that contain 'usb'

            devices = [f for f in files if re.search(search, f)]

 

            return devices

 

        if devices := find_devices(search, pattern):

            for device in devices:

                try:

                    serial_device = serial.Serial(port=device, baudrate=self.baudrate, timeout=self.timeout, bytesize=self.bytesize, parity=self.parity, stopbits=self.stopbits)

                    self.port = device

                    logger.info(f"Found serial device: {device}")

                    return serial_device

                except Exception as e:

                    logger.error(f"Error connecting to {device}: {e}")

                    continue

            logger.error("Unable to connect to any serial devices.", extra={"devices": devices})

            raise serial.SerialException("Unable to connect to any detected devices.")

        logger.error("No serial devices found.", extra={"devices": devices})

        raise serial.SerialException("No serial devices found.")

 

    def connect(self):

        """Attempt to open the serial connection."""

        while True:

            try:

                # Close any existing open serial connection

                if self.serial and self.serial.is_open:

                    self.serial.close()

               

                # Detect or open the serial port

                if self.port is None:

                    self.serial = self.detect_devices()

                else:

                    self.serial = serial.Serial(

                        port=self.port,

                        baudrate=self.baudrate,

                        timeout=self.timeout,

                        bytesize=self.bytesize,

                        parity=self.parity,

                        stopbits=self.stopbits

                    )

               

                # Check if the serial port is open

                if self.serial.is_open:

                    logger.info(f"Connected to serial port {self.port} at {self.baudrate} baud.")

                    self.flush_buffers()

                    self.reconnect_attempts = 0  # Reset reconnect attempts on successful connection

                    return  # Exit loop on successful connection

               

            except serial.SerialException as e:

                logger.error(f"Failed to connect to serial port {self.port}: {e}")

 

                # Increment reconnect attempts if max attempts are specified

                if self.max_reconnect_attempts > 0:

                    self.reconnect_attempts += 1

                    if self.reconnect_attempts > self.max_reconnect_attempts:

                        logger.error("Max reconnect attempts reached. Giving up.")

                        self.running = False  # Stop thread if max attempts reached

                        sys.exit(0)

               

                logger.info(

                    f"Attempting to reconnect... "

                    f"(Attempt: {self.reconnect_attempts if self.max_reconnect_attempts > 0 else '∞'})"

                )

                if self._specified_port is None:

                    # No port was specified on initialization

                    # Set self.port = None to attempt to detect devices

                    self.port = None  

                time.sleep(self.timeout)  # Wait before retrying

 

    def run(self):

        packet = b""

        self.running = True

        while self.running:

            try:

                if self.serial.is_open:

                    with self.input_lock:

                        if self.serial.in_waiting:

                            if packet := self.packet_reader.read_packet():

                                logger.debug(f"Received {len(packet)} bytes", extra={"data": packet})

                                self.input_buffer.append(packet)

 

                    with self.output_lock:

                        if self.output_buffer:

                            data_to_send = self.output_buffer.popleft()

                            if isinstance(data_to_send, str):

                                data_to_send = data_to_send.encode("utf-8")

                            if data_to_send:

                                self.serial.write(data_to_send)

                                logger.debug(f"Sent {len(data_to_send)} bytes", extra={"data": data_to_send})

                else:

                    logger.warning(

                        f"Serial port {self.port} is not open - Attempting to reconnect."

                    )

                    self.connect()  # Try to reconnect if the serial port is not open

            except serial.SerialException as e:

                logger.error(f"Serial error: {e} - Attempting to reconnect...")

                self.connect()  # Reconnect on general serial error

            except OSError as e:

                logger.error(f"OS error: {e} - Attempting to reconnect...")

                self.connect()

            except Exception as e:

                print(traceback.format_exc())

                logger.error(

                    f"Unexpected error in serial thread: {e} - Stopping thread."

                )

                self.running = False  # Stop thread on unexpected error

 

    def write(self, data):

        try:

            # with self.output_lock:

            self.output_buffer.append(data)

        except Exception as e:

            logger.error(f"Failed to add data to output buffer: {e}")

 

    def read_input(self):

        try:

            if self.input_buffer:

                latest_data = self.input_buffer.pop()

                self.input_buffer.clear()

                return latest_data

            return None

        except Exception as e:

            logger.error(f"Failed to read from input buffer: {e}")

            return None

 

    def readline(self):

        return self.read_input()

 

    def stop(self):

        self.running = False

        if self.serial and self.serial.is_open:

            self.serial.close()

 

    @property

    def in_waiting(self):

        return len(self.input_buffer)

   

    @property

    def is_open(self):

        try:

            return self.serial.is_open

        except AttributeError:

            return False

   

    def flush_buffers(self):

        # self.serial.flush()

        self.serial.reset_input_buffer()

        self.serial.reset_output_buffer()

        self.input_buffer.clear()

        self.output_buffer.clear()

 

class SerialPacketReader:

 

    def __init__(self, serial_port):

        self.serial_port = serial_port

 

    @abstractmethod

    def read_packet(self): ...

 

import time

 

class WindowedPacketReader(SerialPacketReader):

 

    def __init__(self, serial_port, window_size=10, start_byte=0xA5, end_byte=0x5A, timeout=1.0):

        super().__init__(serial_port)

        self.window_size = window_size

        self.buffer = bytearray()

        self.start_byte = start_byte

        self.end_byte = end_byte

        self.timeout = timeout

 

    def read_packet(self):

        start_time = time.time()  # Start the timeout clock

        window_size = self.window_size

 

        while time.time() - start_time <= self.timeout:

            try:

                # If buffer is empty, read a chunk to fill it up

                if not self.buffer:

                    chunk = self.serial_port.read(window_size)

                    if not chunk:  # Handle None or empty read

                        continue

                    self.buffer = chunk

                else:

                    # Read one byte and slide the window

                    byte = self.serial_port.read(1)

                    if not byte:  # Handle None or empty read

                        continue

                    self.buffer = self.buffer[1:] + byte

 

                # Check for valid packet by verifying start and end bytes

                if (

                    self.buffer

                    and self.buffer[0] == self.start_byte

                    and self.buffer[-1] == self.end_byte

                ):

                    return self.buffer[1:-1]  # Return packet without start and end bytes

            except TypeError as e:

                print(traceback.format_exc())

                self.buffer = bytearray()

 

        # If timeout is reached, return None

        return None

 

    # def read_packet(self, timeout=1.0):

    #     start_time = time.time()  # Start the timeout clock

    #     window_size = self.window_size

 

    #     while time.time() - start_time <= timeout:

    #         try:

    #             # If buffer is empty, read a chunk to fill it up

    #             if not self.buffer:

    #                 self.buffer = self.serial_port.read(window_size)

    #             else:

    #                 # Read one byte and slide the window

    #                 self.buffer = self.buffer[1:] + self.serial_port.read(1)

 

    #             # Check for valid packet by verifying start and end bytes

    #             if (

    #                 self.buffer

    #                 and self.buffer[0] == self.start_byte

    #                 and self.buffer[-1] == self.end_byte

    #             ):

    #                 return self.buffer[1:-1]  # Return packet without start and end bytes

    #         except Exception as e:

    #             print(traceback.format_exc())

    #             raise e

               

    #     # If timeout is reached, return None

    #     return None

 

class EscapedPacketDecoder(SerialPacketReader):

 

    def __init__(self, serial_port, start_byte=0x7E, escape_byte=0x7D, xor_mask=0x20):

        super().__init__(serial_port)

        self.start_byte = start_byte

        self.escape_byte = escape_byte

        self.xor_mask = xor_mask

 

    def read_packet(self):

        packet = self.read_packet_encoded()

        if packet:

            return self.decode_packet(packet)

 

    def read_packet_encoded(self):

        while True:

            byte = self.serial_port.read(1)

            if not byte:

                continue  # No data, wait for the next byte

            byte = byte[0]  # Convert from byte to int

 

            # Look for the start of a new packet

            if byte == self.start_byte:

                length_byte = self.serial_port.read(1)

                if not length_byte:

                    continue  # No length byte, wait for the next packet

 

                data_length = length_byte[0]

                packet_data = self.serial_port.read(

                    data_length + 1

                )  # Read data and end byte

 

                # Verify packet is complete

                if (

                    len(packet_data) == data_length + 1

                    and packet_data[-1] == self.start_byte

                ):

                    return packet_data[:-1]  # Return data portion without the end byte

 

    def decode_packet(self, packet_data):

        data = bytearray()

        escaping = False

 

        for byte in packet_data:

            if byte == self.escape_byte:

                escaping = True  # Next byte is escaped

                continue

            if escaping:

                byte ^= self.xor_mask  # Unescape byte

                escaping = False

            data.append(byte)

 

        # Convert the bytearray into a list of int32 values

        int32_values = []

        for i in range(0, len(data), 4):

            if i + 4 <= len(data):

                int32_value = int.from_bytes(

                    data[i : i + 4], byteorder="little", signed=True

                )

                int32_values.append(int32_value)

 

        return int32_values

 

def main():

    serial_port = "/dev/ttyACM0"

    baudrate = 115200

    serial_thread = SerialThread(serial_port, baudrate)

    serial_thread.start()

 

    try:

        while True:

            command = input("Enter a command to send (or 'exit' to quit): ")

            if command.lower() == "exit":

                break

            serial_thread.write(command)

 

            received_data = serial_thread.read_input()

            if received_data:

                print(f"Processed input: {received_data}")

 

    except KeyboardInterrupt:

        print("Exiting...")

    finally:

        serial_thread.stop()

        serial_thread.join()

 

if __name__ == "__main__":

    main()

