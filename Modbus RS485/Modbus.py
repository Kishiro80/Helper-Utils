
###
#Modbus Class
#Make sure to have the pyserial library installed (pip install pyserial) to work with the serial port.
#Adjust the COM port and baud rate according to your specific setup.
#This code is ised with USB to RS485 converter. It works well with 9.6k to 38.4k baud.
#This code is never tested with 2Million Baud, but since it is python, i Assume it might have speed issue there.
###

from time import sleep
import struct
import crcmod
import serial





class ModbusController:
    def __init__(self, port):
        self.port = port

    def generate_modbus_request(self, slave, function_code, starting_address, quantity):
        request = struct.pack('>2B2H', slave, function_code, starting_address, quantity)
        return request

    def calculate_crc(self, data):
        crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        crc = crc16(data)
        return crc

    def append_crc(self, data):
        crc = self.calculate_crc(data)
        data += struct.pack('<H', crc)
        return data

    def generate_modbus_response(self, address, value, data_type):
        data_formats = {
            'int16': ('>h', 2),
            'int': ('>i', 4),
            'float': ('>f', 4),
            'double': ('>d', 8),
            'int64': ('>q', 8)
        }
        if data_type not in data_formats:
            raise ValueError(f"Unsupported data type: {data_type}")
        
        format_str, byte_count = data_formats[data_type]
        
        packed_value = struct.pack(format_str, value)
        
        response = struct.pack(f'>{byte_count + 2}B', address, byte_count, *packed_value)
        
        response = self.append_crc(response)
        
        return response

    def send_request(self, motor, code, address, word_count=2, troubleshoot=0):
        data = [motor, code, (address >> 8) & 0xFF, address & 0xFF, 0x00, word_count]
        checksum = 0xFFFF
        for j in range(6):
            checksum ^= data[j]
            for i in range(8):
                if checksum & 0x0001 == 1:
                    checksum = (checksum >> 1) ^ 0xA001
                else:
                    checksum = checksum >> 1
        data.extend([checksum & 0xFF, (checksum >> 8) & 0xFF])
        if troubleshoot == 1:
            message = ' '.join([f'{i:02x}' for i in data])
            print(message)
        self.port.write(bytes(bytearray(data)))
        if troubleshoot:
            print("done write")

    def unpack_response(self, data, data_type='int', endian=1234, word_count=2, troubleshoot=0, multiplier=1):
        result = -999
        checksum = 0xFFFF
        for j in range(len(data) - 2):
            checksum ^= ord(data[j])
            for i in range(8):
                if checksum & 0x0001 == 1:
                    checksum = (checksum >> 1) ^ 0xA001
                else:
                    checksum = checksum >> 1
        if troubleshoot == 1:
            message = ' '.join([f'{ord(d):02x}' for d in data])
            print("\t\t", message)
        if hex(checksum & 0xFF) == hex(ord(data[len(data) - 2])) and hex((checksum >> 8) & 0xFF) == hex(ord(data[len(data) - 1])):
            value = 0
            endian_digits = [int(d) for d in str(endian)]
        for i in range(word_count * 2):
            value = (value << 8) | ord(data[endian_digits[i] + 2])
        try:
            if data_type == 'int':
                result = value
            elif data_type == 'float':
                result = struct.unpack('f', struct.pack('I', value))[0]
            elif data_type == 'sint':
                result = struct.unpack("<i", struct.pack('I', value))[0]
            else:
                result = value
            result *= multiplier
        except Exception as e:
            print("res err", e)
            result = -999
        return result
    
    def receive_response(self, limit=3, count=0, data_type='int', endian=1234, word_count=2, troubleshoot=0, multiplier=1):
        result = -999
        if self.port.in_waiting > 0:
            data = []
            while self.port.in_waiting > 0:
                data.append(self.port.read())
            if troubleshoot == 1:
                print("data", data)
            return self.unpack_response(data, data_type, endian, word_count, troubleshoot, multiplier)
        else:
            if count < limit:
                sleep(0.01)
                print("nothing received", count)
                return self.receive_response(limit, count + 1, troubleshoot=troubleshoot)
            else:
                if troubleshoot:
                    print("no data")
        return result
if __name__ == "__main__":
   
if __name__ == "__main__":
    # Set up the serial port
    ser = serial.Serial('COM4', 9600)

    # Create an instance of ModbusController
    controller = ModbusController(ser)

    # Testing generate_modbus_request
    request = controller.generate_modbus_request(8, 4, 3110, 2)
    print("Modbus Request:", request)

    # Testing calculate_crc
    crc = controller.calculate_crc(request)
    print("CRC:", crc)

    # Testing append_crc
    request_with_crc = controller.append_crc(request)
    print("Request with CRC:", request_with_crc)

    # Testing generate_modbus_response
    response = controller.generate_modbus_response(1, 500, 'int16')
    print("Modbus Response:", response)

    # Testing send_request
    controller.send_request(1, 2, 1000, word_count=4, troubleshoot=1)

    # Testing unpack_response
    data = [b'\x10', b'\x03', b'\x04', b'\xff', b'\xfd', b'H', b'i', b'\xac', b'\xf8']
    unpacked_response = controller.unpack_response(data, 'sint', 1234, 2, 0, 1)
    print("Unpacked Response:", unpacked_response)

    # Testing receive_response
    received_response = controller.receive_response(troubleshoot=1)
    print("Received Response:", received_response)