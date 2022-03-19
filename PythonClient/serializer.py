import struct

class Serializer:
    def __init__(self):
        self.data = bytearray()
    
    def push_u8(self, val:int):
        self.data.extend(val.to_bytes(1, 'big', signed=False))
    
    def push_u16(self, val:int):
        self.data.extend(val.to_bytes(2, 'big', signed=False))
        
    def push_u32(self, val:int):
        self.data.extend(val.to_bytes(4, 'big', signed=False))
        
    def push_u64(self, val:int):
        self.data.extend(val.to_bytes(8, 'big', signed=False))
    
    def push_i8(self, val:int):
        self.data.extend(val.to_bytes(1, 'big', signed=True))
    
    def push_i16(self, val:int):
        self.data.extend(val.to_bytes(2, 'big', signed=True))
        
    def push_i32(self, val:int):
        self.data.extend(val.to_bytes(4, 'big', signed=True))
        
    def push_i64(self, val:int):
        self.data.extend(val.to_bytes(8, 'big', signed=True))
    
    def push_float(self, val):
        self.data.extend(bytearray(struct.pack(">f", val)))

    def push_double(self, val):
        self.data.extend(bytearray(struct.pack(">d", val)))

    
    def get_data(self):
        return self.data
        
        
class Deserializer:
    def __init__(self, data):
        self.data = data
        self.pointer = 0
    
    def parse_u8(self):
        return int.from_bytes(self.parse_bytes(1), 'big', signed=False)
    
    def parse_u16(self):
        return int.from_bytes(self.parse_bytes(2), 'big', signed=False)
        
    def parse_u32(self):
        return int.from_bytes(self.parse_bytes(4), 'big', signed=False)
        
    def parse_u64(self):
        return int.from_bytes(self.parse_bytes(8), 'big', signed=False)
    
    def parse_i8(self):
        return int.from_bytes(self.parse_bytes(1), 'big', signed=True)
    
    def parse_i16(self):
        return int.from_bytes(self.parse_bytes(2), 'big', signed=True)
        
    def parse_i32(self):
        return int.from_bytes(self.parse_bytes(4), 'big', signed=True)
        
    def parse_i64(self):
        return int.from_bytes(self.parse_bytes(8), 'big', signed=True)
    
    def parse_float(self):
        return struct.unpack('>f', self.parse_bytes(4))[0]

    def parse_double(self):
        return struct.unpack('>d', self.parse_bytes(8))[0]
    
    def parse_string(self):
        size = self.parse_u16() # string size
        s = self.parse_bytes(size).decode("utf-8") # string content
        self.parse_u8() # null-termination
        return s
    
    def parse_bytes(self, size):
        arr = bytearray(size);
        for i in range(size):
            arr[i] = self.data[self.pointer]
            self.pointer = self.pointer+1
        return arr;
    
    
    def get_data(self):
        return self.data


