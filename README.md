# poly-detect-transform

Computes the linear transform of basis polygons into polygons on image if there is any similar polygons


# launch
The command to start your program:
shape_finer.py -s input.txt -i image.png 

Output printed to the stdout

# Input
The input receives the number N, and then on the N subsequent lines the coordinates X, Y - the segments defining the polygon are given. It is guaranteed that the polygons are non-self-intersecting and also closed. As well as an image with a resolution of 300 by 200 pixels.

# Output
The program should output the number M - the number of detected primitives, and on the next M lines - the number of the figure, offset x, offset y, scale and angle of rotation.

# Algorithm


- Input image preprocessed by a filter that blacks each pixel that doesn't have at least two white neighbors (include diagonally neighbors).
- The result image processed by canny operator to get list of contours.
- Each contour with area more that 1 is approximated by polygon.
- For each basis polygons (polygons from text file) and for approximated polygon arrays of angles computed
- by shifting and reversing arrays of angles, a decision of polygon similarity is taken
- if polygons are similar, then two corresponding segments are taken from each of them and linear transform is computed
