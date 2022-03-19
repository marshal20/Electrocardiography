import socket
import serializer

def recv_all(s):
    data = bytearray()
    
    while True:
        packet = s.recv(8192)
        if not packet:
            break
        data.extend(packet)
        
    return data
    


class Client:
    def __init__(self, server_addr, server_port):
        self.server_addr = server_addr
        self.server_port = server_port
    
    
    def get_probes_values(self):
        data = bytearray()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.server_addr, self.server_port))
            
            # form request
            ser = serializer.Serializer()
            ser.push_u32(1) # request REQUEST_GET_VALUES
            
            # send request
            s.sendall(ser.get_data());
            
            # receive response
            data = recv_all(s)
            
            # close socket
            s.close()
            
        # handle response
        des = serializer.Deserializer(data)
        
        rows_count = des.parse_u32()
        cols_count = des.parse_u32()
        
        # parse 2D array
        arr = []
        # parse name rows
        names = [];
        for i in range(cols_count):
            names.extend([des.parse_string()])
        arr.extend([names])
        # parse values rows
        for i in range(1, rows_count):
            new_row = [0]*cols_count
            for j in range(cols_count):
                new_row[j] = des.parse_double()
            arr.extend([new_row])
        
        return arr
    
    
    def set_dipole_vector(self, x, y, z):
        data = bytearray()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.server_addr, self.server_port))
            
            # form request
            ser = serializer.Serializer()
            ser.push_u32(2) # request REQUEST_SET_DIPOLE_VECTOR
            # dipole vector
            ser.push_double(x)
            ser.push_double(y)
            ser.push_double(z)
            
            # send request
            s.sendall(ser.get_data());
            
            # receive response
            data = recv_all(s)
            
            # close socket
            s.close()
        
    

