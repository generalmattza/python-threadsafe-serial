
# Serial Port Manager

Serial Port Manager is a Python library that enables concurrent connections to a single serial port instance. It provides a thread-safe interface for multiple threads to read from and write to the same serial port without data corruption or race conditions.

## Features

- **Thread-Safe Access**: Allows multiple threads to safely interact with a single serial port instance.
- **Concurrent Connections**: Supports simultaneous read and write operations from different threads.
- **Ease of Use**: Simple API for integrating into existing Python applications.

## Installation

To install Serial Port Manager, clone the repository and install the dependencies:

```bash
git clone https://github.com/davidson-engineering/serial-port-manager.git
cd serial-port-manager
pip install .
```

## Usage

Here's an example of how to use Serial Port Manager:

```python
from threadsafe_serial import SerialManager

# Initialize the serial manager
serial_manager = SerialManager(port='/dev/ttyUSB0', baudrate=9600)

# Start the serial manager
serial_manager.start()

# Function to read data
def read_data():
    while True:
        data = serial_manager.read()
        if data:
            print(f"Received: {data}")

# Function to write data
def write_data():
    while True:
        serial_manager.write(b'Hello, Serial Port!')

# Create threads for reading and writing
read_thread = threading.Thread(target=read_data)
write_thread = threading.Thread(target=write_data)

# Start the threads
read_thread.start()
write_thread.start()

# Join the threads
read_thread.join()
write_thread.join()
```

## Configuration

You can configure the serial port settings in the `config` directory. Modify the `config.yaml` file to set parameters like port, baud rate, timeout, etc.

## Testing

To run the tests, use the following command:

```bash
pytest tests/
```

## Contributing

Contributions are welcome! Please fork the repository and create a pull request with your changes. Ensure that your code adheres to the project's coding standards and includes appropriate tests.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Acknowledgements

This project is developed and maintained by Davidson Engineering. We appreciate the contributions from the open-source community.

For more information, visit the [GitHub repository](https://github.com/davidson-engineering/serial-port-manager). 
