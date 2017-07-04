import train_data
import os
import utils
import math
import xml_out
import csv
import numpy as np
from sets import Set
from threading import Thread, Lock

import rospy
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
from lidar.msg import lidar_pose

MS_IN_SEC = 1.0e9
lidar_c = False
eps = 1.0e-6

class ObjectDetector:
    
    def __init__(self):
        self.objects = []
        self.object_positions = []
        self.object_velocity = []
        self.object_acceleration = []
        self.last_position = [0.0,  0.0,  0.0]
        self.last_realtime = 0
        self.last_velocity = [0.0,  0.0,  0.0]
        self.last_acceleration = [0.0,  0.0,  0.0]
        
        self.car_position = []
        self.car_velocity = []
        self.car_times = []

        self.camera_times = []
        
        self.radar_times = []
        self.radar_points = []
        
        self.lidar_times = []
        self.lidar_data = []
        
        self.camera_coordinates_predicted = []
        self.camera_angles_predicted = []
        self.camera_distances_predicted = []
        
        self.mutexRadar = Lock()
        self.mutexCamera= Lock()
        self.mutexLidar = Lock()
        self.mutexSolve = Lock()
        
    def loadRadarData(self,  path):
        
        self.radar_path = path
        file = open(path)
        
        lines = file.readlines()

        i = 0
        self.radar_times = []
        self.radar_points = []
        while i < len(lines):
            t = int(lines[i][:-1])
            i += 1
            n = int(lines[i][:-1])
            i += 1
            v = []
            for j in range(n):
                w = []
                line = lines[i][:-1].split(' ')
                for k in range(3):
                    w.append(float(line[k]))
                i += 1
                v.append(w)
            self.radar_times.append(t)
            self.radar_points.append(v)
        
        # times need to be sorted to search efficiently
        for i in range(len(self.radar_times)):
            for j in range(len(self.radar_times)):
                if j > i and self.radar_times[j] < self.radar_times[i]:
                    self.radar_times[i],  self.radar_times[j] = self.radar_times[j],  self.radar_times[i]
                    self.radar_points[i],  self.radar_points[j] = self.radar_points[j],  self.radar_points[i]
    
    def loadCameraData(self,  path,  boxes_path,  test_path):
        
        self.camera_path = path
        self.camera_boxes_path = boxes_path
        self.camera_test_path = test_path
        
        self.camera_coordinates_predicted = train_data.get_data(path,  boxes_path,  test_path, self.camera_times)
        self.camera_angles_predicted = train_data.get_data_angles(path,  boxes_path,  test_path, self.camera_times)
        self.camera_distances_predicted = train_data.get_data_distances(path,  boxes_path,  test_path, self.camera_times)
        
        #The angles are the opposite, y > 0 on the left
        #for i in range(len(self.camera_angles_predicted)):
        #    self.camera_angles_predicted[i] = -self.camera_angles_predicted[i]
        print self.camera_angles_predicted
        
    def loadCameraTimes(self,  camera_path):
        self.camera_times_path = camera_path
        
        file = open(self.camera_times_path)

        vis = Set()		

        for line in file:
        	t = int(line)
        	if not t in vis:
        		self.camera_times.append(int(line))
        		vis.add(t)

        self.camera_times.sort()

    def calc_camera_angle(self,  time):
        
        if len(self.camera_times) == 0:
            return 100.0
        
        idx_camera = utils.binary_search(time,  self.camera_times)
        nxt_idx_camera = idx_camera + 1
        
        if idx_camera == len(self.camera_times)-1:
            nxt_idx_camera = idx_camera
            if idx_camera > 0:
                idx_camera = idx_camera - 1

        angle_value = self.camera_angles_predicted[idx_camera]
        angle_value_nxt = self.camera_angles_predicted[nxt_idx_camera]
        
        if abs(angle_value) > 10.0:
            return angle_value_nxt
        if abs(angle_value_nxt) > 10.0:
            return angle_value
    
        angle = utils.interpolate_aprox(time,    self.camera_times[idx_camera],  \
                                                        self.camera_angles_predicted[idx_camera],  \
                                                        self.camera_times[nxt_idx_camera],   \
                                                        self.camera_angles_predicted[nxt_idx_camera])
        
        return angle
    
    def calc_camera_distance(self,  time):
        
        #print "--------> Dist_",  len(self.camera_times)
        
        if len(self.camera_times) == 0:
            return 0.0
            
        idx_camera = utils.binary_search(time,  self.camera_times)
        nxt_idx_camera = idx_camera + 1
        
        if idx_camera == len(self.camera_times)-1:
            nxt_idx_camera = idx_camera
            if idx_camera > 0:
                idx_camera = idx_camera - 1
                
        #print idx_camera,  nxt_idx_camera
            
        distance_val = self.camera_distances_predicted[idx_camera]
        distance_val_nxt = self.camera_distances_predicted[nxt_idx_camera]
        
#        print time,  distance_val,  distance_val_nxt
#        print idx_camera
#        print self.camera_times
#        print self.camera_distances_predicted
#        print self.camera_angles_predicted
        
        if abs(distance_val) < eps:
            return distance_val_nxt
        if abs(distance_val_nxt) < eps:
            return distance_val

        distance = utils.interpolate_aprox(time,    self.camera_times[idx_camera],  \
                                                        self.camera_distances_predicted[idx_camera],  \
                                                        self.camera_times[nxt_idx_camera],   \
                                                        self.camera_distances_predicted[nxt_idx_camera])
        
        return distance
        
    def calc_camera_coordinates(self,  time):
        
        if len(self.camera_times) == 0:
            return [0.0,  0.0,  0.0]
            
        idx_camera = utils.binary_search(time,  self.camera_times)
        nxt_idx_camera = idx_camera + 1
        
        if idx_camera == len(self.camera_times)-1:
            nxt_idx_camera = idx_camera
            if idx_camera > 0:
                idx_camera = idx_camera - 1
        
        #print len(self.camera_times),  len(self.camera_coordinates_predicted),  idx_camera
        coordinates = [utils.interpolate_aprox(time,    self.camera_times[idx_camera],  \
                                                         self.camera_coordinates_predicted[idx_camera][i],  \
                                                        self.camera_times[nxt_idx_camera],   \
                                                         self.camera_coordinates_predicted[nxt_idx_camera][i]) for i in range(3)]
        return coordinates
    
    def solve_from_time(self,  time):
        
        if len(self.radar_times) == 0:
            return 
            
        self.mutexSolve.acquire()
        
        try:
            
            real_time = time - int(0.25*1000000000)
            
            #print real_time,  self.last_realtime
            
            if real_time > self.last_realtime:
                
                self.solve_time(time)
                
#                self.mutexRadar.acquire()
#                try:
#                    nxt_radar_idx = self.get_next_radar_index(self.last_realtime)
#                    last_radar_idx = self.get_next_radar_index(real_time)
#                finally:
#                    self.mutexRadar.release()
#                
#                print nxt_radar_idx,  last_radar_idx
#                
#                while nxt_radar_idx <= last_radar_idx:
#                    
#                    if self.radar_times[nxt_radar_idx] - int(0.25*1000000000) != self.last_realtime:
#                        self.solve_time(self.radar_times[nxt_radar_idx])
#                    
#                    nxt_radar_idx += 1
            else:
                if real_time < self.last_realtime:
                    
                    xml_out.make(detector.ntimes(),  detector.object_positions,  "test",  "Car")
                    
                    print "Init"
                    
                    self.last_position = [0.0,  0.0,  0.0]
                    self.last_realtime = 0
                    self.last_velocity = [0.0,  0.0,  0.0]
                    self.last_acceleration = [0.0,  0.0,  0.0]
                    
                    self.mutexLidar.acquire()
                    try:
                        self.lidar_times = self.lidar_times[-1:]
                        self.lidar_data = self.lidar_data[-1:]
                    finally:
                        self.mutexLidar.release()
                    
                    self.mutexRadar.acquire()
                    try:
                        self.radar_times = self.lidar_times[-1:]
                        self.radar_data = self.lidar_data[-1:]
                    finally:
                        self.mutexRadar.release()
                        
                    self.mutexCamera.acquire()
                    try:
                        self.camera_angles_predicted = []
                        self.camera_coordinates_predicted = []
                        self.camera_distances_predicted = []
                        self.camera_times = []
                    finally:
                        self.mutexCamera.release()
                
            
        finally:
            self.mutexSolve.release()
        
    def solve_time(self,  time,  idx=-1):
        
        real_time = time - int(0.25*1000000000)
        real_lidar_time = time #+ int(0.2*1000000000)
        
        #print real_time
        
        # Estimate position
        estimated_position = [0.0,  0.0,  0.0]
        if not utils.isempty(self.last_position):
            estimated_position = self.last_position
        # KARMAN-FILTER
        if False and not utils.isempty(self.last_velocity):
            estimated_position = [estimated_position[i] + self.last_velocity[i]* \
                                                    (real_time/MS_IN_SEC - self.last_realtime/MS_IN_SEC) \
                                                    for i in range(len(estimated_position))]
                                                    
        #if idx == 350:
        #    print "------------->",  idx
        
        #print estimated_position
#        if not utils.isempty(self.last_acceleration):
#            estimated_position = [estimated_position[i] + 0.5*self.last_acceleration[i]* \
#                                                math.pow(real_time/MS_IN_SEC - self.last_realtime/MS_IN_SEC,  2) \
#                                                for i in range(len(estimated_position))]
        
        self.mutexCamera.acquire()
        try:
            camera_angle = self.calc_camera_angle(real_time)
            camera_distance = self.calc_camera_distance(real_time)
            camera_coordinates = self.calc_camera_coordinates(real_time)
        finally:
            self.mutexCamera.release()
        
        #print camera_angle,  camera_distance
        
        if abs(camera_distance) > eps:
            camera_coordinates = [camera_distance*math.cos(camera_angle),  camera_distance*math.sin(camera_angle),  0.0]
        else:
            camera_coordinates = [0.0,  0.0,  0.0]
        
        self.mutexRadar.acquire()
        try:
            prev_radar = self.get_previous_radar(real_time)
            nxt_radar = self.get_next_radar(real_time)
            prev_radar_time = self.get_previous_radar_time(real_time)
            nxt_radar_timer = self.get_next_radar_time(real_time)
        finally:
            self.mutexRadar.release()
        
        #print real_time
        #print nxt_radar
        #print prev_radar
        
        radar_coordinates_prev = self.get_closest_radar(prev_radar,  estimated_position)
        radar_coordinates_next = self.get_closest_radar(nxt_radar,  estimated_position)
        radar_cam_coordinates_prev = self.get_closest_radar(prev_radar,  camera_coordinates)
        radar_cam_coordinates_next = self.get_closest_radar(nxt_radar,  camera_coordinates)
        
        #print radar_coordinates_prev,  radar_coordinates_next
        #print radar_cam_coordinates_prev,  radar_cam_coordinates_next
        
        radar_coordinates = [0.0,  0.0,  0.0]
        radar_cam_coordinates = [0.0,  0.0,  0.0]
        
        if utils.isempty(radar_coordinates_prev):
            radar_coordinates = radar_coordinates_next
        elif utils.isempty(radar_coordinates_next):
            radar_coordinates = radar_coordinates_prev
        else:
            radar_coordinates = [utils.interpolate_aprox(real_time,    prev_radar_time,  \
                                                        radar_coordinates_prev[i],  \
                                                        nxt_radar_timer,   \
                                                        radar_coordinates_next[i]) for i in range(3)]
                                
        if utils.isempty(radar_cam_coordinates_prev):
            radar_cam_coordinates = radar_cam_coordinates_next
        elif utils.isempty(radar_cam_coordinates_next):
            radar_cam_coordinates = radar_cam_coordinates_prev
        else:
            radar_cam_coordinates = [utils.interpolate_aprox(real_time,    prev_radar_time,  \
                                                        radar_cam_coordinates_prev[i],  \
                                                        nxt_radar_timer,   \
                                                        radar_cam_coordinates_next[i]) for i in range(3)]
        
        #print "------->",  camera_angle,  math.atan2(radar_cam_coordinates[1],  radar_cam_coordinates[0])
        d = utils.distance_2d(radar_cam_coordinates,  [0.0, 0.0, 0.0])
        radar_cam_coordinates = [d*math.cos(camera_angle),  d*math.sin(camera_angle),  0.0]
        #print radar_cam_coordinates
        
        #radar_position_prev = self.get_best_radar_position(prev_radar_time,  estimated_position)
        #radar_position_next = self.get_best_radar_position(nxt_radar_time,  estimated_position)
        
        #if len(self.object_positions) == 486:
        #    print radar_position_prev,  radar_position_next,  real_time
        #    print prev_radar_time,  nxt_radar_time
            
        #radar_position = []
        #if utils.distance_2d(radar_position_prev,  radar_position_next) < 10.0:
        #    radar_position = [utils.interpolate_aprox(real_time,    prev_radar_time,  \
        #                                                 radar_position_prev[i],  \
        #                                                 nxt_radar_time,   \
        #                                                 radar_position_next[i]) for i in range(3)]
        #else:
        #    radar_position = self.get_best_radar_position_full([radar_position_prev, radar_position_next],  camera_angle,  camera_distance,  estimated_position)
        
        lidar_position_prev = [0.0,  0.0,  0.0]
        lidar_position_next = [0.0,  0.0,  0.0]
        
        self.mutexLidar.acquire()
        try:
            if len(self.lidar_times) > 0:
                prev_lidar_time = self.get_previous_lidar_time(real_lidar_time)
                next_lidar_time = self.get_next_lidar_time(real_lidar_time)

                guess = estimated_position
                if utils.isempty(guess):
                    guess = camera_coordinates
            
                #if not utils.isempty(estimated_position) and len(self.lidar_times) > 0:
                lidar_position_prev = self.get_closest_lidar(prev_lidar_time,  guess)
                lidar_position_next = self.get_closest_lidar(next_lidar_time,  guess)
                
                #elif not utils.isempty(radar_position) and len(self.lidar_times) > 0:
                #    lidar_position_prev = self.get_closest_lidar(prev_lidar_time,  radar_position)
                #    lidar_position_next = self.get_closest_lidar(next_lidar_time,  radar_position)
        finally:
            self.mutexLidar.release()
            
        if len(self.lidar_times) > 0:
            lidar_position = [utils.interpolate_aprox(real_lidar_time,    prev_lidar_time,  \
                                                             lidar_position_prev[i],  \
                                                             next_lidar_time,   \
                                                             lidar_position_next[i]) for i in range(3)]
        else:
            lidar_position = [0.0,  0.0,  0.0]
            
        #if len(self.object_positions) == 201:
        #   print "------------->"
        #   print camera_angle,  radar_position
        
#        object_position = [0.0,  0.0,  0.0]
#        if abs(camera_angle) <1.0e-6 or utils.isempty(radar_position):
#            if not utils.isempty(estimated_position) and \
#                utils.distance_2d(lidar_position,  [0, 0, 0]) < 25.0 and \
#                utils.distance_2d(lidar_position, estimated_position) < 5.0:
#                    object_position = lidar_position[:]
#                    object_position[1]  += 0.4
#                    object_position[0]  -= 0.4
#            else:
#                radar_position = [0.0,  0.0,  0.0]
#                object_position = radar_position[:]
#        else:
#            if not utils.isempty(lidar_position) or not utils.isempty(radar_position):
#                if not utils.isempty(lidar_position) and utils.distance_2d(radar_position,  [0, 0, 0]) < 15.0:
#                    print "Elegir lidar: ",  lidar_position
#                    object_position = lidar_position[:]
#                else:
#                    object_position = radar_position[:]
#                object_position[2] = camera_coordinates[2]
#                object_position[1]  += 0.4
        
#        print real_time
#        print radar_position,  camera_angle
#        print object_position,  lidar_position,  estimated_position
#        print prev_lidar_time,  next_lidar_time
#        print lidar_position_prev,  lidar_position_next

        hasLidar = not utils.isempty(lidar_position)
        hasRadar = not utils.isempty(radar_coordinates)
        hasRadarCam = not utils.isempty(radar_cam_coordinates)
        
        #print "Lidar: ",  hasLidar,  lidar_position
        #print "Radar: ",  hasRadar,  radar_coordinates
        #print "RadarCam: ",  hasRadarCam,  radar_cam_coordinates
        #print "Camera",  camera_coordinates
        #print estimated_position
        
        if utils.isempty(estimated_position) or not hasLidar:
            distanceRadar = utils.distance_ang(estimated_position,  radar_coordinates)
            distanceRadarCam = utils.distance_ang(estimated_position,  radar_cam_coordinates)
            if abs(distanceRadar - distanceRadarCam) > 15:
                object_position = radar_cam_coordinates
            else:
                if distanceRadar < distanceRadarCam:
                    object_position = radar_coordinates
                else:
                    object_position = radar_cam_coordinates
        else:
            distanceLidar = utils.distance_ang(camera_coordinates,  lidar_position)
            distanceRadar = utils.distance_ang(estimated_position,  radar_coordinates)
            distanceRadarCam = utils.distance_ang(estimated_position,  radar_cam_coordinates)
            
            if (abs(distanceLidar) < 10 or not hasRadarCam) \
                    and (utils.distance_ang([0.0, 0.0, 0.0],  lidar_position) < 20.0 or utils.distance_ang(radar_cam_coordinates,  lidar_position) < 5.0):
                #print distanceLidar
                if abs(distanceLidar) > 5:
                    object_position = radar_coordinates
                    object_position = radar_cam_coordinates
                else:
                    object_position = lidar_position
                object_position = lidar_position
            else:
                if distanceRadar < distanceRadarCam:
                    object_position = radar_coordinates
                else:
                    object_position = radar_cam_coordinates
                object_position = radar_cam_coordinates
        
        #if utils.isempty(object_position) and not utils.isempty(camera_coordinates):
        #    object_position = camera_coordinates
        
        #print "Result: ",  object_position
        
        #object_position[1]  = abs(object_position[1]) - 2
        self.object_positions.append(object_position)
        
        #print real_time,  self.last_realtime
        #print estimated_position,  object_position
        #print "-->",  self.get_previous_lidar(real_time)
        
        velocity = [0.0,  0.0,  0.0]
        if not utils.isempty(object_position) and not utils.isempty(self.last_position):
            velocity = [(object_position[i] - self.last_position[i]) / (real_time/MS_IN_SEC - self.last_realtime/MS_IN_SEC) \
                                for i in range(len(object_position))]
              
        acceleration = [0.0,  0.0,  0.0]
        if not utils.isempty(velocity) and not utils.isempty(self.last_velocity):
            acceleration = [(velocity[i] - self.last_velocity[i]) / (real_time/MS_IN_SEC - self.last_realtime/MS_IN_SEC) \
                                for i in range(len(velocity))]
        
        self.object_velocity.append(velocity)
        self.object_acceleration.append(acceleration)
            
        self.last_position = object_position
        self.last_velocity = velocity
        self.last_acceleration = acceleration
        self.last_realtime = real_time
        
    def get_previous_radar_time(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 == len(self.radar_times) and idx > 0:
            return self.radar_times[idx-1]
        return self.radar_times[idx]
    
    def get_previous_radar_index(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 == len(self.radar_times) and idx > 0:
            return idx-1
        return idx
        
    def get_next_radar_time(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 < len(self.radar_times):
            return self.radar_times[idx+1]
        return self.radar_times[idx]
        
    def get_next_radar_index(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 < len(self.radar_times):
            return idx+1
        return idx
        
    def get_previous_radar(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 == len(self.radar_times) and idx > 0:
            return list(self.radar_points[idx-1])
        return list(self.radar_points[idx])
        
    def get_next_radar(self,  time):
        idx = utils.binary_search(time,  self.radar_times)
        if idx+1 < len(self.radar_times):
            return list(self.radar_points[idx+1])
        return list(self.radar_points[idx])
        
    def get_previous_lidar_time(self,  time):
        idx = utils.binary_search(time,  self.lidar_times)
        return self.lidar_times[idx]
        
    def get_next_lidar_time(self,  time):
        idx = utils.binary_search(time,  self.lidar_times)
        if idx+1 < len(self.lidar_times):
            return self.lidar_times[idx+1]
        return self.lidar_times[idx]
        
    def get_best_radar_position(self,  time,  guess = [0.0,  0.0,  0.0]):
        
        camera_angle = self.calc_camera_angle(time)
        camera_distance = self.calc_camera_distance(time)

        #if abs(camera_distance) > 0.1:
        #	print time, camera_angle, camera_distance
        
        idx = utils.binary_search(time,  self.radar_times)
        points = self.radar_points[idx]
        
        debug = False
        
        res = self.get_best_radar_position_full(points,  camera_angle,  camera_distance, guess,  debug)
        
        #print camera_distance,  utils.distance_2d(res,  [0, 0, 0])
        
        if False and camera_distance > 0.1:
            print "------------------------"
            print time
            print points
            print camera_distance,  camera_angle
            print res
        
        return res
    
    def get_best_radar_position_full(self,  points,  camera_angle,  camera_distance, guess = [0.0,  0.0,  0.0],  debug=False):
        
        best = -1.0
        res = [0.0, 0.0,  0.0]
        
        if len(points) == 0:
            return [camera_distance*math.cos(camera_angle),  -camera_distance*math.sin(camera_angle),  0.0]
        
        #if len(self.object_positions) == 439:
        #    print points
        #    print camera_angle,  camera_distance
        #    print guess
        #    debug = True
        
        #if debug:
        #    print "--------------------------"
        #    print points
        #    print camera_angle,  camera_distance
        
        if abs(camera_angle) > 1.0e-6:
        
#            dif1 = 1000000
#            dif2 = 1000000
#            
#            for j in range(len(points)):
#                try:
#                    ang2 = math.atan2(points[j][1],  points[j][0])
#                    dist2 = abs(math.sqrt(points[j][0]*points[j][0] + points[j][1]*points[j][1]) - camera_distance)
#                    
#                    if debug:
#                        print ang2,  dist2,  dif1,  dif2
#                    
#                    if utils.isempty(guess) or utils.distance_2d(guess,  points[j]) < 5.0:
#                        if dif1 - abs(ang2 - camera_angle) > 0.1:
#                            res =points[j]
#                            dif1 = abs(ang2 - camera_angle)
#                            dif2 = dist2
#                        elif abs(dif1 - abs(ang2 - camera_angle)) < 0.1 and dif2 > dist2:
#                            res =points[j]
#                            dif2 = dist2
#                except TypeError:
#                    print "TypeError"

            dif1 = 1000000
            dif2 = 1000000
            
            for j in range(len(points)):
                try:
                    ang2 = math.atan2(points[j][1],  points[j][0])
                    dist2 = abs(math.sqrt(points[j][0]*points[j][0] + points[j][1]*points[j][1]) - camera_distance)
                    
                    #if debug:
                    #    print points[j]
                    #    print ang2,  dist2,  dif1,  dif2
                    
                    #print ang2,  dist2

                    if abs(ang2 - camera_angle) < 0.15 and abs(dist2) < 10:
                        #if debug:
                        #    print "pasa ",  guess
                        if not utils.isempty(guess):
                            if utils.distance_2d(guess,  points[j]) < dif2:
                                res =points[j]
                                dif2 = utils.distance_2d(guess,  points[j]) 
                        else:
                            if dif2 > dist2:
                                res =points[j]
                                dif2 = dist2
                except TypeError:
                    print "TypeError"
        
        #print camera_angle,  math.atan2(res[1],  res[0])
        
        if res[0] == 0.0 and res[1]==0.0:
            dif1 = 1000000
            dif2 = 1000000
            
            for j in range(len(points)):
                try:
                    ang2 = math.atan2(points[j][1],  points[j][0])
                    dist2 = abs(math.sqrt(points[j][0]*points[j][0] + points[j][1]*points[j][1]) - camera_distance)
                    
                    #if debug:
                    #    print utils.distance_2d(guess,  points[j])
                    #    print "-->"
                    
                    if not utils.isempty(guess):
                        if utils.distance_2d(guess,  points[j]) < 10.0 and utils.distance_2d(guess,  points[j]) <dif2:
                            res =points[j]
                            dif2 = utils.distance_2d(guess,  points[j]) 

                except TypeError:
                    print "TypeError"
            
        return res
        
    def get_closest_lidar(self,  time,  guess):
        
        if utils.isempty(guess):
            return [0.0,  0.0,  0.0]
        
        idx = utils.binary_search(time,  self.lidar_times)
        
        #print "Lidar---> ",  self.lidar_data[idx]
        
        points = self.lidar_data[idx]
        
        #print "lidar: ",  points
        #print idx
        
        best = 10.0
        res = [0.0,  0.0,  0.0]
        
        #print points
        
        for elem in points:
            
            d = utils.distance_ang(elem,  guess)
            #print d
            
            if d < best:
                best = d
                res = elem
        
        return list(res)
        
    def solve(self):
        
        n = len(self.camera_times)
        
        for i in range(n):
            t = int(self.camera_times[i])
           
            self.solve_time(t,  i)
            
        #print self.object_positions
        
    def loadCarPosition(self,  path_front,  path_rear):
        
        front_coord = []
        rear_coord = []
        times = []
        
        with open(path_front) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                time = int(row['timestamp'])
                tx = float(row['tx'])
                ty = float(row['ty'])
                tz = float(row['tz'])
                front_coord.append([tx,  ty,  tz])
                times.append(time)
                    
        with open(path_rear) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                time = int(row['timestamp'])
                tx = float(row['tx'])
                ty = float(row['ty'])
                tz = float(row['tz'])
                rear_coord.append([tx,  ty,  tz])
        
        self.car_times = times
        yaws = []
        for f,  r in zip(front_coord,  rear_coord):
            yaw = utils.get_yaw(f, r)
            yaws.append(yaw)
            self.car_position.append([(f[i] + r[i])/2.0 for i in range(3)])
        
        self.car_velocity.append([0.0,  0.0,  0.0])
        for i in range(len(self.car_position) - 1):
            vel = [(self.car_position[i+1][j] -self.car_position[i][j])/(times[i+1]/MS_IN_SEC - times[i]/MS_IN_SEC) \
            for j in range(3)]
            
            rot_z = utils.rotMatZ(-yaws[i+1])
            vel = np.dot(rot_z, vel)
            
            self.car_velocity.append(vel)
            
    def loadLidarData(self,  path):
        
        file = open(path)
        lines = file.readlines()
        
        self.lidar_times = []
        self.lidar_data = []
        
        IMAGE_HEIGHT = 701
        IMAGE_WIDTH	= 801
        BIN	= 0.1
        
        i = 0
        while i < len(lines):
            
            t = int(lines[i][:-1])
            
            self.lidar_times.append(t)
            i+= 1
            n = int(lines[i][:-1])
            i += 1
            
            w = []
            for j in range(n):
                v = lines[i][:-1].split(" ")
                
                if lidar_c:
                    x1 = int(v[0])
                    y1 = int(v[1])
                    x2 = int(v[2])
                    y2 = int(v[3])
                else:
                    x1 = int(v[0])
                    y1 = int(v[2])
                    x2 = int(v[1])
                    y2 = int(v[3])
                
                #if t == 1490993852401494000:
                #    print v
                
                x1 = ((IMAGE_HEIGHT*BIN)/2.0) - (1.0*x1/IMAGE_HEIGHT) * (IMAGE_HEIGHT*BIN)
                y1 = ((IMAGE_WIDTH*BIN)/2.0) - (1.0*y1/IMAGE_WIDTH) * (IMAGE_WIDTH*BIN)
                x2 = ((IMAGE_HEIGHT*BIN)/2.0) - (1.0*x2/IMAGE_HEIGHT) * (IMAGE_HEIGHT*BIN)
                y2 = ((IMAGE_WIDTH*BIN)/2.0) - (1.0*y2/IMAGE_WIDTH) * (IMAGE_WIDTH*BIN)
                
                w.append([(x1+x2)/2.0,  (y1+y2)/2.0,  0.0])
                
                if False and t == 1492886993411118000:
                    print v
                    print x1,  y1,  x2,  y2
                    print [(x1+x2)/2.0,  (y1+y2)/2.0]
                
                i += 1
                
            self.lidar_data.append(w)
        
        for i in range(len(self.lidar_times)):
            for j in range(i+1,  len(self.lidar_times),  1):
                
                if self.lidar_times[i] > self.lidar_times[j]:
                    self.lidar_times[i],  self.lidar_times[j] = self.lidar_times[j],  self.lidar_times[i]
                    self.lidar_data[i],  self.lidar_data[j] = self.lidar_data[j],  self.lidar_data[i]
                    
                
    def ntimes(self):
        return len(self.camera_times) 
        
    def get_closest_radar(self,  radar_points,  coord):
        
        #print coord,  utils.isempty(coord)
        if utils.isempty(coord):
            return [0.0,  0.0,  0.0]
            
        res = [0.0,  0.0,  0.0]
        best = 10.0
        for elem in radar_points:
            #print elem,  coord,  utils.distance_ang(elem,  coord) ,  best,  res
            if utils.distance_ang(elem,  coord) < best:
                best = utils.distance_ang(elem,  coord)
                res = elem
        #print res
        return res
    
    def showResults(self,  path):
        
        from PIL import Image, ImageFont, ImageDraw
        import os
        
       # print self.object_positions
        
        for filename in sorted(os.listdir(path)):
            if filename.endswith(".png"): 
                print filename[:-4]
                t = int(filename[:-4])
                print t
                idx = utils.binary_search(t,  self.camera_times)
                
                if idx+1 == len(self.camera_times):
                    nxtidx = idx
                    idx = idx - 1
                else:
                    nxtidx = idx + 1
                
                print idx
                
                pos = [utils.interpolate_aprox(t,    self.camera_times[idx],  \
                                                             self.object_positions[idx][i],  \
                                                             self.camera_times[nxtidx],   \
                                                             self.object_positions[nxtidx][i]) for i in range(3)]
                                                            
                IMAGE_HEIGHT	= 701
                IMAGE_WIDTH	= 801
                BIN	= 0.1
                row = int(round(math.floor(((((IMAGE_HEIGHT*BIN)/2.0) - pos[0])/(IMAGE_HEIGHT*BIN)) * IMAGE_HEIGHT)))
                column = int(round(math.floor(((((IMAGE_WIDTH*BIN)/2.0) - pos[1])/(IMAGE_WIDTH*BIN)) * IMAGE_WIDTH)))
                print pos,  row,  column
                
                source_img = Image.open(path + filename)
                draw = ImageDraw.Draw(source_img)
                draw.rectangle(((column-10, row-10), (column+10, row+10)))
                
                newimgpath = path + "res/" + filename 
                
                source_img.save(newimgpath, "PNG")
                
    def lidarCallback(self,  lidarMsg):
        
        print "LidarCallback: ",  lidarMsg.timestamp
        
        t = lidarMsg.timestamp
        
        IMAGE_HEIGHT  = 701
        IMAGE_WIDTH   = 801
        BIN   = 0.1
        
        self.mutexLidar.acquire()
        
        try:
            if len(self.lidar_times) == 0 or t != self.lidar_times[-1]:
                self.lidar_times.append(t)
                self.lidar_data.append([])
                for x1,  x2,  y1,  y2 in zip(lidarMsg.xl,  lidarMsg.xh,  lidarMsg.yl,  lidarMsg.yh):

                    x1 = ((IMAGE_HEIGHT*BIN)/2.0) - (1.0*x1/IMAGE_HEIGHT) * (IMAGE_HEIGHT*BIN)
                    y1 = ((IMAGE_WIDTH*BIN)/2.0) - (1.0*y1/IMAGE_WIDTH) * (IMAGE_WIDTH*BIN)
                    x2 = ((IMAGE_HEIGHT*BIN)/2.0) - (1.0*x2/IMAGE_HEIGHT) * (IMAGE_HEIGHT*BIN)
                    y2 = ((IMAGE_WIDTH*BIN)/2.0) - (1.0*y2/IMAGE_WIDTH) * (IMAGE_WIDTH*BIN)

                    self.lidar_data[-1].append([(x1+x2)/2.0,  (y1+y2)/2.0,  0.0])
                    
                    if len(self.lidar_data) > 500:
                        self.lidar_data = self.lidar_data[1:]
                        self.lidar_times = self.lidar_times[1:]
        finally:
            self.mutexLidar.release()
            
            self.solve_from_time(t)
        
        #print self.object_positions[-1]
        pass
        
    def radarCallback(self,  msg):

        arr = utils.msg_to_arr(msg)
        
        s = ('%010d%09d'%(
            msg.header.stamp.secs, msg.header.stamp.nsecs
        ))
        
        #print "RadarCallback: ",  s
        
        t = int(s)
        
        self.mutexRadar.acquire()
        try:
            self.radar_times.append(t)
            self.radar_points.append(arr)
            
            if len(self.radar_times) > 500:
                self.radar_times = self.radar_times[1:]
                self.radar_points = self.radar_points[1:]
        finally:
            self.mutexRadar.release()
            
        pass
        
    def cameraCallback(self,  lidarMsg):
        
        print "CameraCallback: ",  lidarMsg.timestamp
        
        t = lidarMsg.timestamp
        
        x1 = lidarMsg.xl[0]
        y1 = lidarMsg.yl[0]
        x2 = lidarMsg.xh[0]
        y2 = lidarMsg.yh[0]
        h = y2-y1
        w = x2-x1
        
        if abs(h) < eps:
            D = 0.0
            self.camera_distances_predicted.append(0.0)
        else:
            P = h
            F = 1460.62
            F = 1580.46
            W = (1.0*w / h)*1.57
            W = 1.524
            D = F*W/P
            self.camera_distances_predicted.append(D)
        
        WIDTH = 1368
        HEIGHT = 1096
                
        tmp = [x1 +w/2,  y1+ h/2]
        #print tmp
        tmp = [HEIGHT - tmp[1], -tmp[0] + WIDTH/2]
        val = math.atan2(tmp[1],  tmp[0])
        val = val*0.2/0.5
        
        if abs(D) < eps:
            val = 100.0
        
        self.mutexCamera.acquire()
        try:
            self.camera_angles_predicted.append(val)
            self.camera_coordinates_predicted.append([D*math.cos(val),  D*math.sin(val),  0.0])
            self.camera_times.append(t)
            
            #print lidarMsg
            #print D,  val

            if len(self.camera_angles_predicted) > 500:
                self.camera_angles_predicted = self.camera_angles_predicted[1:]
                self.camera_coordinates_predicted = self.camera_coordinates_predicted[1:]
                self.camera_distances_predicted = self.camera_distances_predicted[1:]
                self.camera_times = self.camera_times[1:]
        finally:
            self.mutexCamera.release()
            
        pass
    
if __name__ == "__main__":
    
    rospy.init_node('detector')
    
    detector = ObjectDetector()
    
    lidar_subscriber = rospy.Subscriber('/lidar/lidar_pose', lidar_pose, detector.lidarCallback)
    radar_subscriber = rospy.Subscriber('/radar/points', PointCloud2, detector.radarCallback)
    cam_subscriber = rospy.Subscriber('/camera_pose', lidar_pose, detector.cameraCallback)
    
    rospy.spin()
    
#    files = ["ford02_v2"]
#    
#    for file in files:
#        
#        path = "./Out/" + file
#    
#        detector = ObjectDetector()
#        
#        detector.loadRadarData("./Out/" + "radar_data_" + file + ".txt")
#        
#        if lidar_c:
#            detector.loadLidarData("./Out/" + "lidar_c_" + file + "_v2.txt")
#        else:
#            detector.loadLidarData("./Out/" + "lidar_" + file[:-1] + "4.txt")
#        
#        times = "./Out/" + "times_" + file + ".txt"
#        detector.loadCameraTimes(times)
#        
#        camera = "./Out/" + "boxes_" + file + ".txt"
#        
#        detector.loadCameraData("./data_2_cut.txt",  "./output2.txt", camera)
#        
#        #detector.loadCarPosition("/home/alex/Downloads/didi-competition-master/tracklets/output_test/19_f2/capture_vehicle_front_rtk_interp.csv",  \
#        #    "/home/alex/Downloads/didi-competition-master/tracklets/output_test/19_f2/capture_vehicle_rear_rtk_interp.csv")
#        
#        detector.solve()
#        
#        detector.showResults("/home/alex/lidar/tmp2/")
#        
#        name = file.split('_')[0]
#        type = "Car"
#        if name == "ped":
#            name = "ped_test"
#            type = "Pedestrian"
#        
#        xml_out.make(detector.ntimes(),  detector.object_positions,  name,  type)
#        
#        import zipfile
#                
#        with zipfile.ZipFile('xml.zip', 'w') as myzip:
#            for file in files:
#                name = file.split('_')[0]
#                if name == "ped":
#                    name = "ped_test"
#                myzip.write(name + '.xml')
