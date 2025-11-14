import serial
import serial.tools.list_ports
import struct

ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device)

ser = serial.Serial("COM0", 9600)
start_marker = bytes([0xAA])
end_marker = bytes([0x55])
buffer = bytearray()

def decode_data(start, end):
    global buffer
    # get the buffer that represents the stuff and update the old buffer            
    packet = buffer[start: end+1]
    buffer = buffer[end+1:]

    #compute checksum for checking
    float_bytes = packet[1:13]
    compute_checksum = 0
    for b in float_bytes:
        compute_checksum ^= b

    #print(len(json_str))
    decoded_struct = struct.unpack("<B3f2B", packet)
     
    #print(f"{checksum} | {compute_checksum}")
    if(compute_checksum == decoded_struct[4]):
        return decoded_struct
    else:
        return -1

while(True):
    if ser.in_waiting > 0:
        #data = ser.read(ser.in_waiting).decode("utf-8", errors = "ignore")
        data = ser.read(ser.in_waiting)
        buffer += data

        while start_marker in buffer and end_marker in buffer:
            start = buffer.find(start_marker)
            end = buffer.find(end_marker, start)


            if start != -1 and end != -1 and end > start:
                print(decode_data(start,end))