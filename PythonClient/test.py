import client
from time import sleep

# define server address
SERVER_ADDR = "127.0.0.1"
SERVER_PORT = 1234

# initialize client class
c = client.Client(SERVER_ADDR, SERVER_PORT)


# test get_probes_values

arr = c.get_probes_values()
print("Received data")
print("Rows count {}, Columns count {}\n".format(len(arr), len(arr[0])));
print("Data: ")
print(arr);


# test set_dipole_vector

while True:
    sleep(1)
    c.set_dipole_vector(1, 1, 1)
    sleep(1)
    c.set_dipole_vector(1, 0, 0)
    sleep(1)
    c.set_dipole_vector(0, 1, 0)
    sleep(1)
    c.set_dipole_vector(0, 0, 1)
    sleep(1)
    c.set_dipole_vector(0, 0, -1)
    sleep(1)
    c.set_dipole_vector(0, -1, 0)
    sleep(1)
    c.set_dipole_vector(-1, 0, 0)

