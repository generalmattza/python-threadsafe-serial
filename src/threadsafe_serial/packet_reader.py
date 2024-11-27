from abc import abstractmethod
import time


class PacketReader:
    def __init__(self, read_callback):
        self.read_callback = read_callback

    @abstractmethod
    def read_packet(self):
        pass


class WindowedPacketReader(PacketReader):
    def __init__(self, read_callback, window_size=10, start_byte=0xA5, end_byte=0x5A, timeout=1.0):
        super().__init__(read_callback)
        self.window_size = window_size
        self.start_byte = start_byte
        self.end_byte = end_byte
        self.timeout = timeout

    def read_packet(self):
        """Read packets using a sliding window."""
        start_time = time.time()
        buffer = bytearray()

        while time.time() - start_time <= self.timeout:
            # Use the callback to fetch data
            data = self.read_callback()
            if data:
                buffer.extend(data)
                if len(buffer) > self.window_size:
                    buffer.pop(0)  # Keep the buffer size within the window

                # Check if the buffer matches the packet criteria
                if (
                    len(buffer) == self.window_size
                    and buffer[0] == self.start_byte
                    and buffer[-1] == self.end_byte
                ):
                    return buffer[1:-1]  # Return packet data without start/end bytes
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
