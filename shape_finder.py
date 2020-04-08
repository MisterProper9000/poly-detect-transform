##################################################################
#                                                                #
#        Python code to detect an polygon from an image.         # 
#                                                                #
##################################################################
# Algotithm                                                      #
# 1) clear (make black) all white pixels which have less than    #
#    two white neighbours (cleared pixels don't counted in next  #
#    iterations)                                                 #
# 2) detect edges and use approxPolyDP to recognize polygons     #
# 3) by shifting arrays of basis and detected polygons angles    #
#    correlate vertices and segments of polygons                 #
# 4) after correlation compute scale, rotation and shift         #
##################################################################
# Output format:                                                 #
# M // number of detected primitives                             #
# number_of_basis_primitive shift_x shift_y scale angle          #
##################################################################
# Launch cmd command example:                                    #
# python shape_finder.py -s 001_noise_in.txt -i 001_line_src.png #
##################################################################
# Example files:                                                 #
#   001_noise_in.txt                                             #
#   001_pure_src.png                                             #
#   001_line_src.png                                             #
#   001_noise_src.png                                            #
#   test300_200.png                                              #
##################################################################
import cv2
import numpy as np
import argparse
import sys

class Transform:
   def __init__(self, obj):
       self.i = obj['i'] if 'i' in obj else 0
       self.scale = obj['scale'] if 'scale' in obj else 1
       self.angle = obj['angle'] * np.pi / 180.0 if 'angle' in obj else 0
       self.dx = obj['dx'] if 'dx' in obj else 0
       self.dy = obj['dy'] if 'dy' in obj else 0


transforms = []

def draw(inpt, gt, shape, transform: Transform, color=255):
    
  shape = np.array(shape)
  assert (inpt.shape == gt.shape)

  new_shape = shape.copy().astype(np.float)
  
  # Scale
  new_shape *= transform.scale
  
  # Rotation
  tmp = new_shape.copy()
  for i in [0, 1]:
     new_shape[:, i] = np.cos(transform.angle) * tmp[:, i] \
                       - ((-1)** i) * np.sin(transform.angle) * tmp[:, 1 - i]
  
  #Shift
  new_shape[:, 0] += transform.dx
  new_shape[:, 1] += transform.dy
  
  cv2.fillPoly(gt, [new_shape.astype(np.int32)], color)
  cv2.polylines(inpt, [new_shape.astype(np.int32)], True, color)
  cv2.imshow("sas", inpt)
  cv2.imshow("sas", gt)
  if cv2.waitKey(0) & 0xFF == ord('q'):  
    cv2.destroyAllWindows() 



# read and parse input file
def read_file(filename):
    with open(filename) as f:
      res = []
      fig_num = int(f.readline())
      for lines in range(0,fig_num):
        rres = []
        nums = f.readline().split(",")
        for num in nums:
          rres.append(int(num.strip()))
        res.append(rres)
      return res


# if segments have common point
def getAngle1(a, b, c):
  ba = a - b
  bc = c - b
  cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
  angle = np.arccos(cosine_angle)
  return np.degrees(angle)


# if segments do not have common point
def getAngle2(a, b, c, d):
  ab = b - a
  cd = d - c
  
  cosine_angle = np.dot(ab, cd) / (np.linalg.norm(ab) * np.linalg.norm(cd))
  angle = np.arccos(cosine_angle)
  return np.degrees(angle)


# i can use np.roll, but who cares
def shift(seq, n=0):
    a = n % len(seq)
    return seq[-a:] + seq[:-a]


# one-degree accuracy here
def isclose(a, b, eps=1):
    return abs(a-b) <= eps


# find a shift between two arrays: [shift, second array was reversed(-1), or not (1)]
def correlateAngles(angles1, angles2):
  if len(angles1) != len(angles2):
    return [-1000,-1000]

  for n in range(0, len(angles1)):
    shift_angles2 = shift(angles2, n)
    for i in range(0, len(angles1)):
      if not isclose(angles1[i], shift_angles2[i]):
        break
      if i == len(angles1)-1:
        return [n,1]
  rev_anges2 = list(reversed(angles2))

  for n in range(0, len(angles1)):
    shift_rev_angles2 = shift(rev_anges2, n)
    for i in range(0, len(angles1)):
      if not isclose(angles1[i], shift_rev_angles2[i]):
        break
      if i == len(angles1)-1:
        return[n,-1]
  return [-1000,-1000]


def segmentLength(segment):
  return np.sqrt((segment[0][0]-segment[1][0])*(segment[0][0]-segment[1][0]) + (segment[0][1]-segment[1][1])*(segment[0][1]-segment[1][1]))

def checkPoly(angle, poly1, poly2):
  pass


# finds shifts, scale and angle to make two segments equal
def findTrasform(i, segment1, segment2):
  res = [i]
  length1 = segmentLength(segment1)
  length2 = segmentLength(segment2)
  
  scale = length2/length1
  segment1[1][0] *= scale
  segment1[1][1] *= scale
  # print("_____trans______")
  # print(segment1)
  # print(segment2)

  # angle between segments
  angle = getAngle2(segment2[0], segment2[1], segment1[0], segment1[1])
  # print(angle)
  #second_try_angle = 180 - angle

  # rotate segment1 by angle

  c, s = np.cos( angle*np.pi/180. ), np.sin(angle*np.pi/180.)
  # print(c,s)
  rot_segment1 = [[segment1[0][0]*c+segment1[0][1]*s,-segment1[0][0]*s+segment1[0][1]*c], [segment1[1][0]*c+segment1[1][1]*s,-segment1[1][0]*s+segment1[1][1]*c]]
  # print(segment2)
  # print(rot_segment1)

  shift_x = segment2[0][0] - rot_segment1[0][0]
  shift_y = segment2[0][1] - rot_segment1[0][1]
  # print(shift_x,shift_y)

  #bruteforce
  if(not isclose(rot_segment1[1][0]+shift_x, segment2[1][0], 5) or not isclose(rot_segment1[1][1] + shift_y, segment2[1][1], 5)):
    # print("bruteforce1")
    angle = -angle
    c, s = np.cos( angle*np.pi/180. ), np.sin(angle*np.pi/180.)
    # print(c,s)
    rot_segment1 = [[segment1[0][0]*c+segment1[0][1]*s,-segment1[0][0]*s+segment1[0][1]*c], [segment1[1][0]*c+segment1[1][1]*s,-segment1[1][0]*s+segment1[1][1]*c]]
    shift_x = segment2[0][0] - rot_segment1[0][0]
    shift_y = segment2[0][1] - rot_segment1[0][1]


  if(not isclose(rot_segment1[1][0]+shift_x, segment2[1][0], 5) or not isclose(rot_segment1[1][1] + shift_y, segment2[1][1], 5)):
    # print("bruteforce2")
    angle = 180-angle
    c, s = np.cos( angle*np.pi/180. ), np.sin(angle*np.pi/180.)
    # print(c,s)
    rot_segment1 = [[segment1[0][0]*c+segment1[0][1]*s,-segment1[0][0]*s+segment1[0][1]*c], [segment1[1][0]*c+segment1[1][1]*s,-segment1[1][0]*s+segment1[1][1]*c]]
    shift_x = segment2[0][0] - rot_segment1[0][0]
    shift_y = segment2[0][1] - rot_segment1[0][1]


  angle = -angle
  # print("_____trans_end______")

  res_dict = {'dx' : shift_x, 'dy' : shift_y, 'scale' : scale, 'angle' : angle, 'i' : i}
  transforms.append(Transform(res_dict))
  return [i, shift_x, shift_y, scale, angle]


# 001_pure_src.png
# 001_line_src.png
# 001_noise_src.png
# test300_200.png


# Reading image 
ap = argparse.ArgumentParser()
ap.add_argument("-s", "--file", required=True,
  help="path to the input text file")
ap.add_argument("-i", "--image", required=True,
  help="path to the input image")
args = vars(ap.parse_args())
img2 = cv2.imread(args["image"], cv2.IMREAD_COLOR) 

basis_polygons = read_file(args["file"])

#forming the array of polygon's angles
basis_angles = []
for p in range (0, len(basis_polygons)):
  length = len(basis_polygons[p])
  bbasis_angles = []
  for i in range (0, int(length/2)):
    bbasis_angles.append(getAngle1(np.array([basis_polygons[p][(2*(i-1))%length], basis_polygons[p][(2*(i-1)+1)%length]]),
                                 np.array([basis_polygons[p][2*i], basis_polygons[p][2*i+1]]), 
                                 np.array([basis_polygons[p][(2*(i+1))%length], basis_polygons[p][(2*(i+1)+1)%length]])))
  basis_angles.append(bbasis_angles)

# print(basis_angles)

# Reading same image in another variable and  
# converting to gray scale. 
img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
img = cv2.copyMakeBorder(img, 1,1,1,1, cv2.BORDER_CONSTANT, value=0)   

# clear all pixel with less then 2 white neighbours
h = img.shape[0]
w = img.shape[1]

for y in range(0, h):
  for x in range(0, w):
    non_zero_arr = [1]
    if img[y, x] != 0:
      save_x = x
      save_y = y
      while len(non_zero_arr) == 1:
        arr_p = [img[y, x+1], img[y, x-1], img[y+1, x], img[y-1, x],
               img[y+1, x+1], img[y+1, x-1], img[y-1, x-1], img[y-1, x+1]]
        arr_i = [[y, x+1], [y, x-1], [y+1, x], [y-1, x],
                 [y+1, x+1], [y+1, x-1], [y-1, x-1], [y-1, x+1]]
        non_zero_arr = np.transpose([np.nonzero(arr_p)])
        if len(non_zero_arr) == 1:
          img[y, x] = 0
          x = arr_i[non_zero_arr[0,0,0]][1]
          y = arr_i[non_zero_arr[0,0,0]][0]
        if len(non_zero_arr) == 0:
          img[y, x] = 0
      
      x = save_x
      y = save_y

#cv2.imshow("cleared", img)
#cv2.imwrite("cleared.png", img)


# Converting image to a binary image  
# (black and white only image). 
_,threshold = cv2.threshold(img, 110, 255,  
                            cv2.THRESH_BINARY) 

# Detecting shapes in image by selecting region  
# with same colors or intensity. 
contours,_=cv2.findContours(threshold, cv2.RETR_EXTERNAL, 
                            cv2.CHAIN_APPROX_SIMPLE)

f= open("out.txt","w")

M=0;
results = []
# print(contours)
# Searching through every region selected to  
# find the required polygon. 
#print("____________")
for cnt in contours : 
    area = cv2.contourArea(cnt) 
    # print(area)
    # Shortlisting the regions based on there area. 
    # >1 to avoid noise of 4 white pixels forming a square
    if area >= 1:  
        approx = cv2.approxPolyDP(cnt,  
                                  0.009 * cv2.arcLength(cnt, True), True) 
        # print("_________")
        # print(area)
        for poly in approx:
          for vert in poly:
            # since approxPolyDP use external contour of shape:
            vert[0] -= 1
            vert[1] -= 1
        # print(approx)
        #cv2.drawContours(img2, [approx], 0, (0, 0, 255), 1)
        approx_angles = []

        length = len(approx)
        # forming the array of polygon's angles
        for i in range (0, length):
          approx_angles.append(getAngle1(np.array([approx[(i-1)%length][0][0], approx[(i-1)%length][0][1]]),
                                       np.array([approx[i][0][0], approx[i][0][1]]), 
                                       np.array([approx[(i+1)%length][0][0], approx[(i+1)%length][0][1]])))

        for i in range(0, len(basis_angles)):
          res = correlateAngles(basis_angles[i], approx_angles)
          if res[0] != -1000 and res[1] != -1000:
            # print(basis_angles[i])
            # print(approx_angles)
            # print(res)
            # print(basis_polygons)
            # print(approx)
            approx = np.roll(approx,res[0], axis=0)
            
            # print(approx)

            #now we are ready to find two corresponding segments in each polygon
            segment1 = np.array([[basis_polygons[i][0], basis_polygons[i][1]], [basis_polygons[i][2], basis_polygons[i][3]]])

            #approx[(0 + res[1]) -- consider whether the array of angles of image-polygon was reversed (res[1])
            segment2 = np.array([[approx[0][0][0], approx[0][0][1]], [approx[(0 + res[1])%length][0][0], approx[(0 + res[1])%length][0][1]]])
            # print(segment1)
            # print(segment2)

            results.append(findTrasform(i, segment1, segment2))
            M += 1

"""       
f.write("%i\n"%M)
for i in range (0,M):
  f.write("%i, %f, %f, %f, %f\n" % (results[i][0], results[i][1], results[i][2], results[i][3], results[i][4]))
f.close()
"""


# print(len(transforms))
# print("basis_polygons")
# print(basis_polygons)
for trans in transforms:
  b_p = []
  for i in range(len(basis_polygons[trans.i])-1):
    # print(i)
    b_p.append([basis_polygons[trans.i][i], basis_polygons[trans.i][i+1]])
    i += 1
  # print("B_P")
  # print(b_p)
  #draw(inpt = img, gt = img, shape = b_p, transform = trans)


sys.stdout.write("%i\n"%M)
for i in range (0,M):
  sys.stdout.write("%i, %f, %f, %f, %f\n" % (results[i][0], results[i][1], results[i][2], results[i][3], results[i][4]))

"""
# Showing the image along with outlined arrow. 
#cv2.imshow('image2', img2)
#cv2.imwrite("res.png", img2)
   
# Exiting the window if 'q' is pressed on the keyboard. 
#if cv2.waitKey(0) & 0xFF == ord('q'):  
#   cv2.destroyAllWindows() 


#print(getAngle2(np.array([0,0]),np.array([0,1]),np.array([30,50]),np.array([73,75])))
"""

