import matplotlib.pyplot as plt
x = list()
y = list()
# file_name = 'Bear_Coordinates'
file_name = 'anne_to_line'
with open(file_name+'.txt', 'r') as file:
    # reading each line
    for line in file:
        c = 0
        # reading each word
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
fig, ax = plt.subplots(figsize=(20, 20))
ax.scatter(x, y, s=0.1, color="green")
# ax.plot(x,y)
ax.set_title(file_name)
plt.show()