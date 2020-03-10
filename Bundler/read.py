import numpy as np

# read file
with open('bundle.out', 'r') as f:
    data = f.readlines()

length = len(data)
# read camera and point number
num_image_points = data[1].split(" ")
num_image = int(num_image_points[0])
num_pts = int(num_image_points[1])

# save # of pts in each image
start = (num_image * 5) + 3
img_dic = {}
for i in range (num_image):
    img_dic[i] = 0

for i in range((start+1), length, 3):
    view_list = data[i].split(" ")
    for j in range(1, len(view_list), 4):
        img_dic[int(view_list[j])] += 1
    
# sort image by number of points in descending ordering
sorted_img = sorted(img_dic.items(), key= lambda x:x[1], reverse=True)

# print(sorted_img)

# record corresponding perspective or information related to camera
view_direction = {} 
R_dict = {}
k = 0
# store R
for i in range(3, (start -1), 5):
    row1 = list(map(float, (data[i].split(" "))))
    row2 = list(map(float, (data[i+1].split(" "))))
    row3 = list(map(float, (data[i+2].split(" "))))
    R = np.concatenate((row1, row2, row3))
    R_dict[k] = R.reshape(3,3)
    k += 1

# store viewing direction
for key, value in R_dict.items():
    vec = np.array(([[0, 0, -1]])).T
    view_direction[key] = np.dot(value.T ,vec).reshape(1,3)

# record viewing direction


