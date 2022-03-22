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

# pause
input("Press enter to continue...")

# test get_probes_names
print("Test get_probes_names")
print(c.get_probes_names())
print("")

# pause
input("Press enter to continue...")

# test calculate_values_for_random_vectors (10 samples)
print("Test calculate_values_for_random_vectors (10 samples)")
print(c.calculate_values_for_random_vectors(10))
print("")

# pause
input("Press enter to continue...")

# test set_dipole_vector and calculate_values_for_vector
print("Test set_dipole_vector and calculate_values_for_vector")
test_vectors = [[1, 1, 1], [-1, -1, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
for vec in test_vectors:
    print("Testing ({}, {}, {}) vector:".format(vec[0], vec[1], vec[2]))
    print(c.calculate_values_for_vector(vec[0], vec[1], vec[2]))
    c.set_dipole_vector(vec[0], vec[1], vec[2])
    sleep(1)

# pause
input("Press enter to continue...")

# test calculate_values_for_random_vectors (10000 samples)
print("Test calculate_values_for_random_vectors (10000 samples)")
print(c.calculate_values_for_random_vectors(10000))
print("")
