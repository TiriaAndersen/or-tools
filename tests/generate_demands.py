# Generate a list of demands and print out in appropriate format

demands = [1]*647
demands[0] = 0
print('{' + ', '.join(str(x) for x in demands) + '};')
