import numpy  as np
'''length = int(input("Length in cms: "))
breadth = int(input("Breadth in cms: "))
height = int(input("Height in cms: "))
path_width = int(input("Path Width in cms: "))
print ("Length: ", length , "cms ; ", "Breadth: ", breadth , "cms ; ", "Height: ", height , "cms ; ", "Path Width: ", path_width , "cms")
'''
convert_to_mts = 100.0
path_width = float(5.0/convert_to_mts)
print ("path_width:", path_width)
length = 240/convert_to_mts
height = 30/convert_to_mts
breadth = 500/convert_to_mts
dist_tip_mold = 20/convert_to_mts
#offsets
tbl_height = 20/convert_to_mts #--->100/convert_to_mts
rbase_height = 0 #--->90/convert_to_mts
#in length
dist_rbase_table_x = 40/convert_to_mts
dist_table_obj_x = 0#2--->0.05#1--->0#3-->0.1 #--->5/convert_to_mts
#in breadth
dist_rbase_table_y = 0 #--->-20/convert_to_mts
dist_table_obj_y = 0.3#2--->-0.1#1--->0.3#3--->-0.3 #as object is on the edge of the table #--->10/convert_to_mts

#no. of points on each path upon addition of waypoints
points_per_path = 2

#dividing the length into no. of bins
path_bins = (length/path_width)

#Total no. of points to cover
num_points = int(path_bins*points_per_path + points_per_path)

#dist to move per point upon addition of waypoints on single path
new_breadth = breadth/(points_per_path - 1)

#List to store the points
points = np.zeros((num_points, 3))
centre_x = dist_rbase_table_x + dist_table_obj_x + length/2
centre_y = dist_rbase_table_y + dist_table_obj_y + breadth/2
centre_z = height + tbl_height - rbase_height + dist_tip_mold

x = centre_x -  length/2
y = centre_y - breadth/2
z = centre_z

print ("Number of points: ", num_points, "Start points: ", x, y, z)

for i in range(0, num_points):
	#For the very first point, no changes. Add as is
	if i == 0 :
		points[i] = [x, y, z]
		continue

	#Moves along breadth, according to no. of points per path
	if (i%points_per_path != 0):
		y = y + new_breadth

	#Moves along length, by path width
	else:
		x = x + path_width
		new_breadth = -new_breadth

	points[i] = [x, y, z]

print (points)

np.savetxt('list1.txt', points, delimiter=',')
'''with open('listfile.txt', 'w') as filehandle:
    for listitem in points:
        filehandle.write('%s\n' % listitem)'''
