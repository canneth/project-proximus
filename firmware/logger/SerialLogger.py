
import serial
import serial.tools.list_ports

from pathlib import Path

if __name__ == "__main__":

    save_path = Path(".") / "data" / "logged_data.csv"

    # Establishing connection automatically
    comport_name_list = []
    for comport in serial.tools.list_ports.comports():
        comport_name_list.append(comport.device)
    print("COM ports available: {}".format(comport_name_list))

    teensy_port_name = "COM5"
    teensy_port = serial.Serial(port = teensy_port_name, baudrate = 115200, timeout = 1)
    print("Selected COM port: {}".format(teensy_port_name))

    while True:
        received_data = teensy_port.readline()
        decoded_data = received_data[0:len(received_data) - 2].decode("UTF-8") # Remove newline
        print(decoded_data)
        with open(save_path, "a") as f:
            f.write(decoded_data + "\n")