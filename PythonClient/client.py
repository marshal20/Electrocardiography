import socket
import serializer

def socket_recvall(s):
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
    
    
    def send_request(self, request_bytes):
        data = bytearray()
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.server_addr, self.server_port))
            
            # send request
            s.sendall(request_bytes);
            
            # receive response
            data = socket_recvall(s)
            
            # close socket
            s.close()
        
        return data
    
    
    def get_probes_values(self):
        # form request
        ser = serializer.Serializer()
        ser.push_u32(1) # request REQUEST_GET_VALUES
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        rows_count = des.parse_u32()
        cols_count = des.parse_u32()
        
        # parse 2D array (rows of [sample_number time dipole_pos_x&y&z dipole_vec_x&y&z probes_values])
        arr = []
        # parse name rows
        names = [];
        for i in range(cols_count):
            names.append(des.parse_string())
        arr.append(names)
        # parse values rows
        for i in range(1, rows_count):
            new_row = [0]*cols_count
            for j in range(cols_count):
                new_row[j] = des.parse_double()
            arr.append(new_row)
        
        return arr
    
    
    def set_dipole_vector(self, x, y, z):
        # form request
        ser = serializer.Serializer()
        ser.push_u32(2) # request REQUEST_SET_DIPOLE_VECTOR
        # dipole vector
        ser.push_double(x)
        ser.push_double(y)
        ser.push_double(z)
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        # check for acknowledgement byte (1)
        if des.parse_u8() != 1:
            print("Warning: set_dipole_vector request no acknowledgement")
    
    
    def get_probes_names(self):
        # form request
        ser = serializer.Serializer()
        ser.push_u32(3) # request REQUEST_GET_PROBES_NAMES
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        probes_count = des.parse_u32()
        probes_names = []
        for i in range(probes_count):
            probes_names.append(des.parse_string())
        
        return probes_names
    
    
    def calculate_values_for_vector(self, x, y, z):
        # form request
        ser = serializer.Serializer()
        ser.push_u32(4) # request REQUEST_CALCULATE_VALUES_FOR_VECTOR
        # dipole vector value
        ser.push_double(x)
        ser.push_double(y)
        ser.push_double(z)
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        probes_count = des.parse_u32()-3 # -3 for dipole vector x, y and z
        values = []
        # dipole vector x, y and z
        values.append(des.parse_double())
        values.append(des.parse_double())
        values.append(des.parse_double())
        # probes_values
        for i in range(probes_count):
            values.append(des.parse_double())
        
        return values
    
    
    def calculate_values_for_random_vectors(self, random_samples_count, maximum_radius=1):
        # form request
        ser = serializer.Serializer()
        ser.push_u32(5) # request REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS
        ser.push_u32(random_samples_count) # random samples count
        ser.push_double(maximum_radius) # maximum radius
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        probes_count = des.parse_u32()-3 # -3 for dipole vector x, y and z
        
        # parse row by row (each row is a sample containing dipole vector x, y, z and probes values)
        values = []
        for j in range(random_samples_count):
            row = []
            # dipole vector x, y and z
            row.append(des.parse_double())
            row.append(des.parse_double())
            row.append(des.parse_double())
            # probes_values
            for i in range(probes_count):
                row.append(des.parse_double())
            # append row to matrix
            values.append(row)
        
        return values

    def set_dipole_vector_values(self, vec_values):
        # vec_values: n rows, 3 columns
        
        # form request
        ser = serializer.Serializer()
        ser.push_u32(6) # request REQUEST_SET_DIPOLE_VECTOR_VALUES
        
        # size of vectors (n)
        ser.push_u32(len(vec_values))
        # dipole vector values
        for vec in vec_values:
            ser.push_double(vec[0])
            ser.push_double(vec[1])
            ser.push_double(vec[2])
        
        response_bytes = self.send_request(ser.get_data())
        
        # handle response
        des = serializer.Deserializer(response_bytes)
        
        # check for acknowledgement byte (1)
        if des.parse_u8() != 1:
            print("Warning: set_dipole_vector request no acknowledgement")
    
    