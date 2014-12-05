#!/usr/bin/env python
import cv,cv2,time,sys
import numpy as np
from math import ceil
from munkres import Munkres
from numpy.linalg import *

class ObjectRecognizer:
  def __init__(self):
    # Names of the captured objects entered by user
    self.names = []
    # List of the contours for each object. Each element is a list of
    # points belonging to each object
    self.contours = []
    # Create an instance of the Munkres class used to compute the 
    # bipartite matching algorithm
    self.munkres_algo = Munkres()

  # Callback for processing streaming images
  def contour_received(self, message):
    contour = self.contour_msg_to_np(message)
    
    if self.state is 'Wait':
      obj_name = raw_input('Enter next object name, or \"done\" to finish:')
      
      # If the user entered a name, capture a contour and add it to the library
      # of stored contours
      if obj_name != 'done':
        self.names.append(obj_name)
        self.capture_time = time.time() + 0.1
        self.state = 'Capture'
      
      # If the user entered 'done', process all of the stored contours to
      # prepare for comparing subsequent contours to the library
      else:
        self.process_library()
        self.state = 'Identify'
    
    # This just captures the next contour after waiting a little bit
    elif self.state is 'Capture':
      if time.time() > self.capture_time:
        self.contours.append(contour)
        self.state = 'Wait'
    
    # When we are in 'Identify' state, publish the name of the identified contour
    else:
      object_name = self.identify_contour(contour)
      self.pub.publish(String(object_name))

    # Allow OpenCV to process
    cv2.waitKey(1)

  # Takes in a NumPy array of contour points and computes the point histograms
  # at each point
  # Returns a NumPy array of histogram vectors indexed by [point, sector_index]
  #
  # Thus histograms[12,:] is a vector that has the normalized counts of points
  # in each of the sector bins with contour[12] as the center point,
  # and histograms[10,7] is the single value of ratio of points in sector 7 
  # when contour[10] is the center point
  def compute_histograms(self, contour):
    n_r_bins = 4
    n_theta_bins = 5
    histograms = np.zeros((len(contour), n_r_bins*n_theta_bins))
    max_dist = max([np.linalg.norm(i-j) for i in contour for j in contour])
    for i in range(len(contour)):
      point_a = contour[i]
      for j in range(len(contour)):
        if i == j:
          continue
        point_b = contour[j]
        dx, dy = point_b - point_a
        theta = np.arctan2(dy,dx)
        r = np.sqrt(dx*dx + dy*dy)
        theta_index = np.round((theta+np.pi/2)/(2*np.pi) * (n_theta_bins-1))
        r_index = np.round(r/(max_dist) * (n_r_bins-1))
        histograms[i][r_index*n_theta_bins + theta_index] += 1
    return histograms

  # Takes in an Array of histograms for two different objects and
  # creates the weighting matrix to be used in the bipartite matching
  # M[i,j] is the histogram distance between obj1[i,:] and obj2[j,:]
  def get_cost_matrix(self, obj1, obj2):
    M = np.zeros((len(obj1), len(obj2)))
    for i in range(len(obj1)):
      for j in range(len(obj2)):
        M[i][j] = np.linalg.norm(obj1[i]-obj2[j])
    return M
  
  # Convert Polygon message to NumPy array indexed by [point, coordinate]
  def contour_msg_to_np(self, message):
    return np.array([[p.x, p.y] for p in message.points])

  # Prepare the collected sample contours to be compared against subsequent
  # contours
  def process_library(self):
    # Precompute the histograms of known object contours
    self.histograms = [self.compute_histograms(c) for c in self.contours]
   
    # Display names and contours in library
    n_cont = len(self.contours)
    scale = 0.5
    img_w = 640
    img_h = 472
    sub_img_w = int(img_w*scale)
    sub_img_h = int(img_h*scale)

    lib_img = np.zeros((sub_img_h, n_cont*sub_img_w, 3), dtype=np.uint8)

    # Draw white line between images
    lib_img[:,sub_img_w::sub_img_w,:] = np.array([255,255,255])

    for i in range(n_cont):
      sub_img = lib_img[:,(i*sub_img_w):((i+1)*sub_img_w),:]
      
      for point in self.contours[i]:
        cv2.circle(sub_img, tuple((point*scale).astype('int')), 3, 100, -1)

      self.label_image(sub_img, self.names[i])

    cv2.imshow('Object Library', lib_img)

  # Put the scaled label text in the upper left corner of the image
  def label_image(self, img, label):
    org = (3,18)
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1.0
    color = (255,255,255)
    cv2.putText(img, label, org, fontFace, fontScale, color)

  # Given a contour, determine which of the stored contours it most closely
  # resembles, and return the name of that contour
  def identify_contour(self, contour):
    # Compute the histogram of the new contour
    new_histogram = self.compute_histograms(contour)
    
    # Calculate the cost matrix M comparing the new histogram to each object
    # histogram
    cost_matrices = []
    for obj_histogram in self.histograms:
      obj_cost_M = self.get_cost_matrix(obj_histogram, new_histogram)
      cost_matrices.append(obj_cost_M)
    
    # Use the Munkres algorithm to determine the minimal total matching 
    # cost for each object cost matrix
    best_match_costs = [self.bipartite_matching(M) for M in cost_matrices]
    print best_match_costs

    # Find the smallest cost amongst all objects
    min_idx = best_match_costs.index(min(best_match_costs))
    
    # Return the name of the object with the lowest cost
    return self.names[min_idx]
    
  # Return total minimal cost found by optimal matching of cost matrix M
  def bipartite_matching(self, M):
    # Create a copy of M (since m.compute() modifies its input during calculation)
    # compute optimal matching
    match = self.munkres_algo.compute(M.copy())
    
    # Compute optimal cost
    cost = 0
    for i in range(len(match)):
      #print mat[assignments[i]]
      xx = match[i][0]
      yy = match[i][1]
      cost = cost + M[xx,yy]
    return cost

  def run(self):
    try:
      rospy.spin()
    except KeyboardInterrupt:
      cv2.destroyAllWindows()

#Python's syntax for a main() method
if __name__ == '__main__':
  recognizer_node = ObjectRecognizer()
  recognizer_node.run()

