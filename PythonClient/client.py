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
        # parse name rows
        arr = [[""]*cols_count]
        for i in range(cols_count):
            arr[0][i] = des.parse_string()
        # parse values rows
        arr.extend([[0]*cols_count]*(rows_count-1))
        for i in range(1, rows_count):
            for j in range(cols_count):
                arr[i][j] = des.parse_double()
        
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
        
    

