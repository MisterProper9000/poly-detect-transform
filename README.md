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


1. Input image preprocessed by a filter that blacks each pixel that doesn't have at least two white neighbors (include diagonally neighbors):
  1. Go through all the pixels of the image.
  2. As soon as a pixel that has less than two white neighbors is found, blacked it and go to the white neighbor (if possible), remembering the original position. 
  3. Continue to shade the pixels and move on until the neighbor's count will become zero or more or equal to two.
  4. Return to the position remembered at the very beginning and continue to iterate.
2. The result image processed by canny operator to get list of contours.
3. Each contour with area more that 1 is approximated by polygon.
4. For each basis polygons (polygons from text file) and for approximated polygon arrays of angles computed
5. by shifting and reversing arrays of angles, a decision of polygon similarity is taken
6. if polygons are similar, then a scale, angle, and axis shifts are computed:
  1. Compute area as square root of the area ratio of polygons.
  2. By combining the centroids of the polygons is the angle of rotation.
  3. After finding the angle, the base polygon is rotated by the found angle relative to the point (0,0).
  4. Transfer is calculated as a vector between the centroids.