
map = []
with open('../util/map.txt', 'r') as f:
    for line in f:
        map.append(line)
n = len(map)
m = len(map[0]) - 1
count = 0
print("agents:")
for i in range(n):
    for j in range(m):
        if (map[i][j] == "r"):
            print("-   name: agent{}".format(count))
            print("    start: [{}, {}]".format(i, j))
            count += 1
print("")
print("map:")
print("    dimension: [{}, {}]".format(n, m))
print("    obstacles:")
for i in range(n):
    for j in range(m):
        if (map[i][j] == "@"):
            print("    - [{}, {}]".format(i, j))

print("-------------------")
print("-------------------")
for i in range(n):
    for j in range(m):
        if (map[i][j] == "e"):
            print("    - [{}, {}]".format(i, j))