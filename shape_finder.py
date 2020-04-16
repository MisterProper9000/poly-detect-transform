##################################################################
#                                                                #
#        Python code to detect an polygon from an image.         #
#                                                                #
##################################################################
# Algotithm                                                      #
# 1) clear (make black) all white pixels which have less than    #
#    two white neighbours (cleared pixels don't counted in next  #
#    iterations)                                                 #
# 2) Select countours by hierarchy                               #
# 3) Fill contours on image and then detect edges and            #
#    approxPolyDP                                                #
# 5) find centroids of polygons and shift basis poly to detected #
# 6) by rotation bruteforce find an angle                        #
# 7) after that rotate basis poly by angle regarding (0,0) and   #
#    find shift                                                  #
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
#     001_pure_src.png                                           #
#     001_line_src.png                                           #
#     001_noise_src.png                                          #
#     test300_200.png                                            #
#   test1.txt                                                    #
#     test1_1.png                                                #
#     test1_2.png                                                #
#     test1_3.png                                                #
#   test1_3.txt                                                  #
#     test1_3.png                                                #
##################################################################

import cv2
import numpy as np
import argparse
import sys
from math import sin, cos, radians
from shapely.geometry import Polygon
import shapely

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
  #print("_______draw_________", file = log)
  new_shape = shape.copy().astype(np.float)
  #print("new_shape", new_shape, file = log)
  # Scale
  new_shape *= transform.scale
  #print("scale new shape", new_shape, file = log)
  # Rotation
  tmp = new_shape.copy()
  for i in [0, 1]:
     new_shape[:, i] = np.cos(transform.angle) * tmp[:, i] \
                       - ((-1)** i) * np.sin(transform.angle) * tmp[:, 1 - i]
  #print("rotate new shape", new_shape, file = log)
  #Shift
  new_shape[:, 0] += transform.dx
  new_shape[:, 1] += transform.dy
  #print("shift new shape", new_shape, file = log)
  
  cv2.fillPoly(gt, [new_shape.astype(np.int32)], color)
  cv2.polylines(inpt, [new_shape.astype(np.int32)], True, color)
  #cv2.imshow("final_result", inpt)
  #cv2.imshow("final_result", gt)
  if cv2.waitKey(0) & 0xFF == ord('q'):  
    cv2.destroyAllWindows() 
  #print("_____end_draw_______", file = log)


# read and parse input file
def read_file(filename):
    with open(filename) as f:
      res = []
      fig_num = int(f.readline())
      for lines in range(0,fig_num):
        rres = []
        nums = f.readline().split(",")
        for num in nums:
          rres.append(float(num.strip()))
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
def isclose(a, b, eps=10):
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



def draw_polygon(img, poly: Polygon, show=True):
  x, y = poly.exterior.coords.xy
  x = np.array(x)
  y = np.array(y)
  contours = []
  cnt = []
  for i in range(len(x)):
    cnt.append([x[i],y[i]])
  cnt = np.array(cnt).astype(int)
  contours.append(cnt)

  img_ = np.zeros_like(img)

  

  for cnt in contours:
    img_ = cv2.drawContours(img_, [cnt], 0, 255, -1)
  if show:
    #cv2.imshow("draw polygon", img_)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
  
  return img_


def find_angle(detected_polygon: Polygon, basis_polygon: Polygon, target_image):

  angle_res = None
  max_iou = 0

  shift_x = detected_polygon.centroid.x - basis_polygon.centroid.x
  shift_y = detected_polygon.centroid.y - basis_polygon.centroid.y
  x, y = basis_polygon.exterior.coords.xy
  x = np.array(x) 
  x += shift_x
  y = np.array(y) 
  y += shift_y
  shifted_scaled_basis_polygon = Polygon(zip(x,y))
  #print("shifted_scaled_basis_polygon", shifted_scaled_basis_polygon, file=log)
  for angle in range(0,360):
    rotated_shifted_scaled_basis_polygon = shapely.affinity.rotate(shifted_scaled_basis_polygon, angle, origin='centroid')
    x, y = rotated_shifted_scaled_basis_polygon.exterior.coords.xy
    x = np.array(x)
    y = np.array(y)
    contours = []
    cnt = []
    for i in range(len(x)):
      cnt.append([x[i],y[i]])
    cnt = np.array(cnt).astype(int)
    contours.append(cnt)

    img_rotated = np.zeros_like(target_image)
    #cv2.imshow("trget image", target_image)

    
    for cnt in contours:
      cv2.drawContours(img_rotated, [cnt], 0, 255, -1)
    #cv2.imshow("rotated", img_rotated)
    iou = np.count_nonzero(np.logical_and(img_rotated, target_image)) / np.count_nonzero(np.logical_or(img_rotated, target_image))

    # uncomment to watch a movie :)
    # draw_polygon(target_image,rotated_shifted_scaled_basis_polygon)

    if iou > max_iou:
      max_iou = iou
      angle_res = angle

  # -Add more detailed search??
  # -Yes!
  for i in range(1, 201):
    angle = angle - 1 + i*0.01
    rotated_shifted_scaled_basis_polygon = shapely.affinity.rotate(shifted_scaled_basis_polygon, angle, origin='centroid')
    x, y = rotated_shifted_scaled_basis_polygon.exterior.coords.xy
    x = np.array(x)
    y = np.array(y)
    contours = []
    cnt = []
    for i in range(len(x)):
      cnt.append([x[i],y[i]])
    cnt = np.array(cnt).astype(int)
    contours.append(cnt)

    img_rotated = np.zeros_like(target_image)
    #cv2.imshow("trget image", target_image)

    
    for cnt in contours:
      cv2.drawContours(img_rotated, [cnt], 0, 255, -1)
    #cv2.imshow("rotated", img_rotated)
    iou = np.count_nonzero(np.logical_and(img_rotated, target_image)) / np.count_nonzero(np.logical_or(img_rotated, target_image))
    # uncomment to watch a movie :)
    # draw_polygon(target_image,rotated_shifted_scaled_basis_polygon)

    if iou > max_iou:
      max_iou = iou
      angle_res = angle
    




  #print("founded angle and max_iou", angle_res, max_iou, file=log)

  return angle_res


# finds shifts, scale and angle to make two segments equal
def findTrasform(i, detected_polygon: Polygon, basis_polygon: Polygon, target_image):
  #print("_____trans______", file = log)
  #print("detected_polygon", detected_polygon, file=log)
  #print("basis_polygon", basis_polygon, file=log)

  res = [i]
  scale = np.sqrt(detected_polygon.area / basis_polygon.area)
  #apply scale
  x, y = basis_polygon.exterior.coords.xy
  #print()
  x = np.array(x)
  x *= scale
  y = np.array(y)
  y *= scale
  scaled_basis_polygon = Polygon(zip(x,y))
  
  draw_polygon(target_image, scaled_basis_polygon)

  #print("scaled_basis_polygon", scaled_basis_polygon, file=log)

  sub_target_image = draw_polygon(target_image, detected_polygon, False)
  #cv2.imshow("sub_target_image", sub_target_image)

  angle = find_angle(detected_polygon, scaled_basis_polygon, sub_target_image)
  if angle is None:
    return angle

  rotated_scaled_basis_polygon = shapely.affinity.rotate(scaled_basis_polygon, angle, origin=(0, 0))

  shift_x = detected_polygon.centroid.x - rotated_scaled_basis_polygon.centroid.x
  shift_y = detected_polygon.centroid.y - rotated_scaled_basis_polygon.centroid.y

  x, y = rotated_scaled_basis_polygon.exterior.coords.xy
  x = np.array(x) 
  x += shift_x
  y = np.array(y) 
  y += shift_y
  shifted_rotated_scaled_basis_polygon = Polygon(zip(x,y))

  #print("detected_polygon", detected_polygon, file=log)
  #print("shifted_rotated_scaled_basis_polygon", shifted_rotated_scaled_basis_polygon, file=log)


  #print("_____trans_end______", file = log)

  res_dict = {'dx' : shift_x, 'dy' : shift_y, 'scale' : scale, 'angle' : angle, 'i' : i}
  transforms.append(Transform(res_dict))
  return [i, shift_x, shift_y, scale, angle]




# Reading image 
ap = argparse.ArgumentParser()
ap.add_argument("-s", "--file", required=True,
  help="path to the input text file")
ap.add_argument("-i", "--image", required=True,
  help="path to the input image")
args = vars(ap.parse_args())
img2 = cv2.imread(args["image"], cv2.IMREAD_GRAYSCALE) 

log = open('log.txt', 'w') 

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

#print("basis angles", basis_angles, file = log)

# Reading same image in another variable and  
# converting to gray scale. 


img = cv2.copyMakeBorder(img2, 1,1,1,1, cv2.BORDER_CONSTANT, value=0)   

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



img = img[1:img.shape[0]-1, 1:img.shape[1]-1]

# Converting image to a binary image  
# (black and white only image). 
_,threshold = cv2.threshold(img, 110, 255,  
                            cv2.THRESH_BINARY) 

# Detecting shapes in image by selecting region  
# with same colors or intensity. 
contours, hierarchy =cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
contours = np.array(contours)[hierarchy [0][:, 2] != -1]

#print("contours count", len(contours), file = log)

target_image = np.zeros_like(img)

for c in contours:
  cv2.drawContours(target_image, [c], 0, 255, -1)
#cv2.imshow("kek", target_image)
#cv2.imwrite("kek.png", target_image)
if cv2.waitKey(0) & 0xFF == ord('q'):  
  cv2.destroyAllWindows()

contours, hierarchy =cv2.findContours(target_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
test_image = np.zeros_like(img)
for c in contours:
  #print(c)
  cv2.drawContours(test_image, [c], 0, 255, -1)
#cv2.imshow("cleared", test_image)
#cv2.imwrite("cleared.png", test_image)
if cv2.waitKey(0) & 0xFF == ord('q'):  
  cv2.destroyAllWindows()

f= open("out.txt","w")

M=0;
results = []

# #print("contours", contours, file = log)
# Searching through every region selected to  
# find the required polygon. 
##print("____________", file = log)
for cnt in contours:
    area = cv2.contourArea(cnt) 
    #print("area", area, file = log)
    # Shortlisting the regions based on there area. 
    # >1 to avoid noise of 4 white pixels forming a square
    if area >= 1:  
        approx = cv2.approxPolyDP(cnt,  
                                  0.009 * cv2.arcLength(cnt, True), True)

        #print("_________", file = log)
        
        
      
        #print("approx", approx, file = log)
        

        length = len(approx)
       
        #print("len approx", len(approx), file = log)
        # forming the array of polygon's angles
        approx_angles = []
        for i in range (0, length):
          approx_angles.append(getAngle1(np.array([approx[(i-1)%length][0][0], approx[(i-1)%length][0][1]]),
                                       np.array([approx[i][0][0], approx[i][0][1]]), 
                                       np.array([approx[(i+1)%length][0][0], approx[(i+1)%length][0][1]])))
        
        for i in range(0, len(basis_angles)):
          res = correlateAngles(basis_angles[i], approx_angles)
          
          #print("res", res, file = log)
         
          if res[0] != -1000 and res[1] != -1000:
            
            #print("basis angles", basis_angles[i], file = log)
            #print("approx angles", approx_angles, file = log)
            #print("res", res, file = log)
            #print("basis ploygons", basis_polygons, file = log)
            #print("approx", approx, file = log)

            approx = np.roll(approx,res[0], axis=0)
            if res[1] == -1:
              approx = list(reversed(approx))
            #print("rolled approx", approx, file = log)

            b_p = []
            for j in range(0,len(basis_polygons[i])-1, 2):
              b_p.append((basis_polygons[i][j], basis_polygons[i][j+1]))

            basis_polygon = Polygon(b_p)

            a_p = []
            for j in range(0, len(approx)):
              a_p.append((approx[j][0][0],approx[j][0][1]))
            detected_polygon = Polygon(a_p)

            #print("basis polygon", basis_polygon, file = log)
            #print("detected polygon", detected_polygon, file = log)
            #print("centroids", basis_polygon.centroid, detected_polygon.centroid, file = log)

            found_trasform = findTrasform(i, detected_polygon, basis_polygon, target_image)

            if found_trasform is not None:
              results.append(found_trasform)

           
            M += 1
#print("detected shapes count:", M, file = log)

for trans in transforms:
  b_p = []
  for i in range(0,len(basis_polygons[trans.i])-1, 2):
    b_p.append([basis_polygons[trans.i][i], basis_polygons[trans.i][i+1]])
  #print("B_P", b_p, file = log)
  draw(inpt = img, gt = img, shape = b_p, transform = trans)

#print(results, file = log)
sys.stdout.write("%i\n"%M)
for i in range (0,M):
  sys.stdout.write("%i, %f, %f, %f, %f\n" % (results[i][0], results[i][1], results[i][2], results[i][3], results[i][4]))