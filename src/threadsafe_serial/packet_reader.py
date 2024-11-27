from abc import abstractmethod
import time


class SerialPacketReader:
    def __init__(self, serial_port):
        self.serial_port = serial_port

    @abstractmethod
    def read_packet(self):
        pass


class WindowedPacketReader(SerialPacketReader):
    def __init__(
        self, serial_port, window_size=10, start_byte=0xA5, end_byte=0x5A, timeout=1.0
    ):
        super().__init__(serial_port)
        self.window_size = window_size
        self.buffer = bytearray()
        self.start_byte = start_byte
        self.end_byte = end_byte
        self.timeout = timeout

    def read_packet(self):
        """Read packets using a sliding window."""
        start_time = time.time()
        while time.time() - start_time <= self.timeout:
            if not self.buffer:
                self.buffer = self.serial_port.read(self.window_size)
            else:
                self.buffer = self.buffer[1:] + self.serial_port.read(1)

            if (
                len(self.buffer) == self.window_size
                and self.buffer[0] == self.start_byte
                and self.buffer[-1] == self.end_byte
            ):
                return self.buffer[1:-1]
        return None


def main():
    serial_port = "/dev/ttyACM0"
    baudrate = 115200
    serial_thread = ThreadSafeSerial(serial_port, baudrate)
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
