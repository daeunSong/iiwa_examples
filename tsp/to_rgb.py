from PIL import Image

image_name = 'anne'
img = Image.open(image_name+'.jpeg')
data = img.getdata()

# Suppress specific bands (e.g. (255, 120, 65) -> (0, 120, 0) for g)
r = [(d[0], 0, 0) for d in data]
g = [(0, d[1], 0) for d in data]
b = [(0, 0, d[2]) for d in data]

img.putdata(r)
img.save('r_'+image_name+'.png')
img.putdata(g)
img.save('g_'+image_name+'.png')
img.putdata(b)
img.save('b_'+image_name+'.png')
rgb = [(r[i][0], g[i][1], b[i][2]) for i in range(len(data))]
img.putdata(rgb)
img.save('rgb_'+image_name+'.png')
# r_list = list()
# g_list = list()
# b_list = list()
#
# for (r, g, b) in data:
#     r_list.append((r, 0, 0))
#     g_list.append((0, g, 0))
#     b_list.append((0, 0, b))