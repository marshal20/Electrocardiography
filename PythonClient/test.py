import client
from time import sleep

# define server address
SERVER_ADDR = "127.0.0.1"
SERVER_PORT = 1234

# initialize client class
c = client.Client(SERVER_ADDR, SERVER_PORT)



# test get_tmp_bsp_values
print("Test get_tmp_bsp_values_probes_2")
tmp_values, bsp_values = c.get_tmp_bsp_values_probes_2()
print("tmp_values: {}x{}, bsp_values: {}x{}".format(len(tmp_values), len(tmp_values[0]), len(bsp_values), len(bsp_values[0])))
print("")

# pause
input("Press enter to continue...")



# test set_tmp_values with edited tmp values
print("Test set_tmp_values")
c.set_tmp_values(tmp_values)
print("")

# pause
input("Press enter to continue...")



# test set_tmp_values with edited tmp values
print("Test set_tmp_values with edited tmp values")
# zero the 2nd half of tmp_values
for i in range(int(len(tmp_values)/2), len(tmp_values)):
    for j in range(len(tmp_values[0])):
        tmp_values[i][j] = 0
c.set_tmp_values(tmp_values)
print("")



