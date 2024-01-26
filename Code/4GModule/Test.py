import machine
import time


uart = machine.UART(0, baudrate=115200, tx=0, rx=1, timeout=1000)

def send_at_command(command):
    # Send AT command
    uart.write(command + b'\r\n')
      # Wait for the module to respond
    time.sleep(0.5)
    # Read and print the response
    response = uart.read()
    print(response.decode('utf-8'))


send_at_command(b'AT')


 