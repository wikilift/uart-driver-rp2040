import serial
import time

#Quick and dirty script to test the rp2040 with RS232 com
#replace with your port and desired baudrate
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


last_message_id = 0

def send_data_via_serial(ser, data, message_id):
    message_with_id = f"{data}ID:{message_id}\n"
    ser.write(message_with_id.encode())  
    ser.flush() 
    print(f"Sent: {message_with_id}")

def read_data(ser):
    
    received_data = ser.read(ser.in_waiting)  
    if received_data:
        decoded_data = received_data.decode(errors='ignore')
        print(f"Received: {decoded_data.strip()}")

if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        exit(1)

    print("Serial communication started. Press Ctrl+C to exit.")

    while True:
        last_message_id += 1
        text_message = "Hello from Python via RS232!"
        ser.reset_input_buffer()  
        send_data_via_serial(ser, text_message, last_message_id)
        time.sleep(0.5)  
        read_data(ser)  
        time.sleep(2)  
