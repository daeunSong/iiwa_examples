def readFromFile (fileName):
    data = []
    stroke = []
    try:
        f = open(fileName, 'r')
        while True:
            line = f.readline()
            if line != 'End\n':
                coord = list(map(float,line.split()))
                stroke.append(coord)
            elif line == 'End\n':
                data.append(stroke)
                stroke = []
            if not line:
                break
    except:
        print("Error opening the file")
        return None
    return data

fileName = '../input/Bear_Coordinates2.txt'
drawing = readFromFile(fileName)

import matplotlib.pylab as plt
import mpl_toolkits.mplot3d as a3
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.grid(False)

## plot drawing line by line
for stroke in drawing:
    xs = list(np.array(stroke).T[0])
    zs = list(np.array(stroke).T[1])
    ys = list(np.zeros(len(xs)))
    ax.plot(xs,ys,zs,color='black',linewidth='0.5')
    plt.ion()
    plt.show()

# plt.axis('square')    // For the right scale
