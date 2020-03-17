# Takes a .vrp instance file and outputs the desired data in the needed format.
# Needs to be adapted to write directly into program.

print('Hello')


input_file = open('C:\\Users\\jc321076\\Documents\\PhD\\github\\Autonomous_DSVRP\\Instances\\BOPMid_full_matrix.txt', mode='r')

# text = input_file.read()

# Convert a space-seperated distance matrix into a comma-seperated matrix.
line_num = 0
start_switch = False

coordinates = []
for line in input_file:
    line_num += 1

    # Convert space-seperated coordinates into braket-comma pairs.
    if start_switch:
        if line.split(None,1)[0] == "647": #last location number
            start_switch = False
        split_line = line.split()
        coordinates.append('(' + split_line[1] + ',' + split_line[2] + ')')

    if line.split(None,1)[0] == "NODE_COORD_SECTION":
        start_switch = True

    #  Extract the first nxn distance matrix etc
    if 1304 > line_num > 656:
        float_line = list(map(float, line.split()))  # read as float
        float_line = float_line[0:647]
        scaled_line = []
        int_line = []
        for i in float_line:
            scaled_line.append(i * 100)  # scale up by 100
            int_line = list(map(int, scaled_line))  # convert to int
        print('[' + ', '.join(str(x) for x in int_line) + '],')

print(', '.join(coordinates))
input_file.close()
