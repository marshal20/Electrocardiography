import socket
import serializer


class Client:
    def __init__(self, server_addr, server_port):
        self.server_addr = server_addr
        self.server_port = server_port
    
    
    def send_request(self, request_bytes):
        data = bytearray()
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.server_addr, self.server_port))
            
            # send request size
            s.send(len(request_bytes).to_bytes(4, 'big', signed=False));
            # send request
            s.sendall(request_bytes);
            
            # receive response size
            response_size = int.from_bytes(s.recv(4), 'big', signed=False)
            # receive response
            #print("Receiving {} bytes".format(response_size))
            data = s.recv(response_size);
            
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
    
    
    def get_tmp_bsp_values(self):
        # returns two matrices: 
        #   * TMP_values: SAMPLE_COUNTxTMP_POINTS_COUNT
        #   * BSP_values: SAMPLE_COUNTxBSP_POINTS_COUNT
    
        # form request
        ser = serializer.Serializer()
        ser.push_u32(7) # request REQUEST_GET_TMP_BSP_VALUES
        
        response_bytes = self.send_request(ser.get_data())
        
        # parse response
        des = serializer.Deserializer(response_bytes)
        
        sample_count = des.parse_u32()
        tmp_points_count = des.parse_u32()
        bsp_count = des.parse_u32()
        
        # parse row by row
        tmp_values = []
        bsp_values = []
        for j in range(sample_count):
            # TMP values
            tmp_row = []
            # tmp_values
            for i in range(tmp_points_count):
                tmp_row.append(des.parse_double())
            # append row to matrix
            tmp_values.append(tmp_row)
            
            # probes values
            bsp_row = []
            # bsp_values
            for i in range(bsp_count):
                bsp_row.append(des.parse_double())
            # append row to matrix
            bsp_values.append(bsp_row)
        
        return tmp_values, bsp_values
    
    
    def set_tmp_values(self, tmp_values):
        # sends tmp_values matrix: SAMPLE_COUNTxHEART_PROBES_COUNT
    
        # form request
        ser = serializer.Serializer()
        ser.push_u32(8) # request REQUEST_SET_TMP_VALUES
        
        # send matrix dimensions
        ser.push_u32(len(tmp_values))    # rows count
        ser.push_u32(len(tmp_values[0])) # cols count
        
        # send matrix
        for i in range(len(tmp_values)):
            for j in range(len(tmp_values[0])):
                ser.push_double(tmp_values[i][j])
        
        response_bytes = self.send_request(ser.get_data())
        
        # parse response
        des = serializer.Deserializer(response_bytes)
        
        # check for acknowledgement byte (1)
        if des.parse_u8() != 1:
            print("Warning: set_tmp_values request no acknowledgement")
        
        
    def get_tmp_bsp_values_probes(self):
        # returns two matrices: 
        #   * TMP_values:    SAMPLE_COUNTxTMP_POINTS_COUNT
        #   * probes_values: SAMPLE_COUNTxPROBES_COUNT
    
        # form request
        ser = serializer.Serializer()
        ser.push_u32(9) # request REQUEST_GET_TMP_BSP_VALUES_PROBES
        
        response_bytes = self.send_request(ser.get_data())
        
        # parse response
        des = serializer.Deserializer(response_bytes)
        
        sample_count = des.parse_u32()
        tmp_points_count = des.parse_u32()
        probes_count = des.parse_u32()
        
        # parse row by row
        tmp_values = []
        probes_values = []
        for j in range(sample_count):
            # TMP values
            tmp_row = []
            # probes_values
            for i in range(tmp_points_count):
                tmp_row.append(des.parse_double())
            # append row to matrix
            tmp_values.append(tmp_row)
            
            # probes values
            probes_row = []
            # probes_values
            for i in range(probes_count):
                probes_row.append(des.parse_double())
            # append row to matrix
            probes_values.append(probes_row)
        
        return tmp_values, probes_values
    

    def get_tmp_bsp_values_probes_2(self):
        # returns two matrices: 
        #   * TMP_values:    SAMPLE_COUNTxTMP_POINTS_COUNT
        #   * probes_values: SAMPLE_COUNTxPROBES_COUNT
    
        # form request
        ser = serializer.Serializer()
        ser.push_u32(10) # request REQUEST_GET_TMP_BSP_VALUES_PROBES_2
        
        response_bytes = self.send_request(ser.get_data())
        
        # parse response
        des = serializer.Deserializer(response_bytes)
        
        sample_count = des.parse_u32()
        tmp_points_count = des.parse_u32()
        probes_count = des.parse_u32()
        
        # parse row by row
        tmp_values = []
        probes_values = []
        for j in range(sample_count):
            # TMP values
            tmp_row = []
            # probes_values
            for i in range(tmp_points_count):
                tmp_row.append(des.parse_double())
            # append row to matrix
            tmp_values.append(tmp_row)
            
            # probes values
            probes_row = []
            # probes_values
            for i in range(probes_count):
                probes_row.append(des.parse_double())
            # append row to matrix
            probes_values.append(probes_row)
        
        return tmp_values, probes_values

        