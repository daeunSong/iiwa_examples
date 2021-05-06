import numpy as np
import matplotlib.pyplot as plt

x = list()
y = list()
x_line = list()
y_line = list()
# file_name = 'Bear_Coordinates'
file_name = 'anne'

with open(file_name+'.txt', 'r') as file:
    for line in file:
        c = 0
        for i in line.split():
            if i == 'End':
                continue
            else:
                if c%2 ==0:
                    x.append(float(i))
                    c+=1
                else:
                    y.append(float(i))
                    c += 1
interval= 10
for i in range(0, len(x), 2):
    x_expand = np.linspace(x[i], x[i+1], interval)
    y_expand = np.linspace(y[i], y[i+1], interval)
    x_line.extend(x_expand)
    y_line.extend(y_expand)
fig, ax = plt.subplots(figsize=(20, 20))
ax.scatter(x_line, y_line, s=0.1, color="hotpink")
# ax.plot(x,y)

# save line points to file
h = ""
for i in range(len(x_line)):
    h += str(x_line[i]) + " " + str(y_line[i]) + '\n'
with open(file_name+'_to_line'+'.txt', 'w') as the_file:
    the_file.write(h)


# ax.autoscale()
ax.set_title(file_name)
plt.show()