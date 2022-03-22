import client
from time import sleep

# define server address
SERVER_ADDR = "127.0.0.1"
SERVER_PORT = 1234

# initialize client class
c = client.Client(SERVER_ADDR, SERVER_PORT)


# test get_probes_values
arr = c.get_probes_values()
print("Test get_probes_values")
print("Rows count {}, Columns count {}\n".format(len(arr), len(arr[0])));
print("Data: ")
print(arr)
print("")


# test get_probes_names
print("Test get_probes_names")
print(c.get_probes_names())
print("")


# test calculate_values_for_random_vectors (10 samples)
print("Test calculate_values_for_random_vectors (10 samples)")
print(c.calculate_values_for_random_vectors(10))
print("")


# test set_dipole_vector and calculate_values_for_vector
print("Test set_dipole_vector and calculate_values_for_vector")
test_vectors = [[1, 1, 1], [-1, -1, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
for i in range(len(test_vectors)):
    x = test_vectors[i][0]
    y = test_vectors[i][1]
    z = test_vectors[i][2]
    print("Testing ({}, {}, {}) vector:".format(x, y, z))
    print(c.calculate_values_for_vector(x, y, z))
    c.set_dipole_vector(x, y, z)
    sleep(1)


# test calculate_values_for_random_vectors (10000 samples)
print("Test calculate_values_for_random_vectors (10000 samples)")
print(c.calculate_values_for_random_vectors(10000))
print("")
