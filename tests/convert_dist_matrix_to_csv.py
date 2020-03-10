# Convert a space-seperated distance matrix into a comma-seperated matrix.
# Takes a .vrp instance file and outputs a copiable, formatted distance matrix.
# Needs to be adapted to write directly into program.

print('Hello')


input_file = open('C:\\Users\\jc321076\\Documents\\PhD\\github\\Autonomous_DSVRP\\Instances\\BOPMid_full_matrix.txt', mode='r')

# text = input_file.read()

line_num = 0

for line in input_file:
    line_num += 1
    # Only process distance matrix section
    # if 1304 > line_num > 656:
    #     float_line = list(map(float, line.split()))  # read as float
    #     scaled_line = []
    #     int_line = []
    #     for i in float_line:
    #         scaled_line.append(i * 100)  # scale up by 100
    #         int_line = list(map(int, scaled_line))  # convert to int
    #     print('{' + ', '.join(str(x) for x in int_line) + '},')

    #  Extract the first 18x18 distance matrix etc
    if 1157 > line_num > 656:
        float_line = list(map(float, line.split()))  # read as float
        float_line = float_line[0:500]
        scaled_line = []
        int_line = []
        for i in float_line:
            scaled_line.append(i * 100)  # scale up by 100
            int_line = list(map(int, scaled_line))  # convert to int
        print('{' + ', '.join(str(x) for x in int_line) + '},')

input_file.close()
