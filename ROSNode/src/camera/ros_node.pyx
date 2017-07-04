import sys
##sys.path.insert(0,'/opt/ros/kinetic/lib/python2.7/dist-packages')
##sys.path.insert(0,'/usr/lib/python2.7/dist-packages')  ## rospkg

import os
#os.system('source /opt/ros/kinetic/setup.bash')

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
from geometry_msgs.msg import TwistStamped
from PIL import Image as PILImage
import math
from cv_bridge import CvBridge, CvBridgeError
import cv2
import test

from lidar.msg import lidar_pose

#---------------------------------------------------------------------------------------------------------

# https://github.com/eric-wieser/ros_numpy #############################################################################################

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}



def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list




def msg_to_arr(msg):

    dtype_list = fields_to_dtype(msg.fields, msg.point_step)
    arr = np.fromstring(msg.data, dtype_list)

    # remove the dummy fields that were added
    arr = arr[[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    
    if msg.height == 1:
        return np.reshape(arr, (msg.width,))
    else:
        return np.reshape(arr, (msg.height, msg.width))

##################################################################################################################################

lidar_dir = 'lidar/tmp'
radar_dir = 'lidar/tmp'

name = "ped_test"

file = open("radar_data_" + name + ".txt",  "w")
times = {}

bridge = CvBridge()
detector = test.DetectorHelper(3, name)

def cameraPosePublisherCallback(boxes, stamp):
    
    print('callbackCameraPub: timestamp=%d'%(
        stamp
    ))

    msg = lidar_pose()
    
    v = []
    for b in boxes:
        v.append(b[0])
    msg.xl = v
    
    v = []
    for b in boxes:
        v.append(b[2])
    msg.xh = v
    
    v = []
    for b in boxes:
        v.append(b[1])
    msg.yl = v
    
    v = []
    for b in boxes:
        v.append(b[3])
    msg.yh = v
    
    msg.timestamp = stamp

    camera_pub.publish(msg)

detector.setCallback(cameraPosePublisherCallback)

def callback(msg):
    #print('callback: msg : seq=%d, timestamp=%010d:%09d'%(
    #    msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs
    #))
    arr= msg_to_arr(msg)
    #file=lidar_dir+'/%010d%09d.npy'%(
    #     msg.header.stamp.secs, msg.header.stamp.nsecs
    #)
    dir=lidar_dir+'/%010d%09d.png'%(
         msg.header.stamp.secs, msg.header.stamp.nsecs
    )

    #np.save(file,arr)
    create_image(arr,  dir)
    #dd=0

    detector.detect_lidar(dir)

    pass


def callback2(msg):
    #print('callbackRadar: msg : seq=%d, timestamp=%010d:%09d'%(
    #    msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs
    #))
    arr= msg_to_arr(msg)
    
    s = ('%010d%09d'%(
        msg.header.stamp.secs, msg.header.stamp.nsecs
    ))
    
    if s not in times:
        file.write(s + "\n")
        file.write(str(len(arr)) + "\n")
        for i in range(len(arr)):
            file.write("%f %f %f\n" % (arr[i][0],  arr[i][1],  arr[i][2]))
        file.flush()
        times[s] = True
    
    pass

def callback4(msg):
    print('callbackSonar: msg : seq=%d, timestamp=%010d:%09d'%(
        msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs
    ))
    print msg
    arr= msg_to_arr(msg)
    print arr
    
    s = ('%010d%09d'%(
        msg.header.stamp.secs, msg.header.stamp.nsecs
    ))
    
    pass

def callbackCamera(msg):
    print('callbackCamera: msg : seq=%d, timestamp=%010d:%09d'%(
        msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs
    ))

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    s = ('%010d%09d'%(
        msg.header.stamp.secs, msg.header.stamp.nsecs
    ))

    filepath = "./camera/" + s + ".jpg"

    #print filepath
    height, width, channels = cv_image.shape
    cv_image = cv_image[400:height-200, 0:width]
    cv2.imwrite(filepath, cv_image)

    detector.detect(filepath)
    
    pass
    
def callback3(msg):
    print('callbackVel: msg : seq=%d, timestamp=%010d:%09d'%(
        msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs
    ))
    print msg
    
    pass

from libc.math cimport round 
from libc.math cimport floor 

def create_image(arr,  dir):
  data = arr

  w, h = 801, 701

  IMAGE_HEIGHT  = 701
  IMAGE_WIDTH   = 801
  BIN   = 0.1

  data2 = np.zeros((h, w, 3), dtype=np.uint8)

  cdef int n = len(data)

  cdef double x
  cdef double y
  cdef double z
  cdef int row
  cdef int column
  cdef int f

  for i in range(n):
    p = data[i]
    
    x = p[0]
    y = p[1]
    z = p[2]
    
    row = int(round(math.floor(((((IMAGE_HEIGHT*BIN)/2.0) - x)/(IMAGE_HEIGHT*BIN)) * IMAGE_HEIGHT)))
    column = int(round(math.floor(((((IMAGE_WIDTH*BIN)/2.0) - y)/(IMAGE_WIDTH*BIN)) * IMAGE_WIDTH)))

    #row = int(round(floor(np.multiply(np.divide(np.subtract(np.divide(np.multiply(IMAGE_HEIGHT,BIN),2.0), x), np.multiply(IMAGE_HEIGHT,BIN)), IMAGE_HEIGHT))))
    #column = int(round(floor(np.multiply(np.divide(np.subtract(np.divide(np.multiply(IMAGE_WIDTH,BIN),2.0), y), np.multiply(IMAGE_WIDTH,BIN)), IMAGE_WIDTH))))
    
    
    f = int(round((z + 3.0)/6.0 * 255));
    
    if f < 0:
      f = 0
        
    if row >= 0 and row < h and column >= 0 and column < w:
      data2[row][column] = [f, 0, 0]
    
  img = PILImage.fromarray(data2, 'RGB')
  img.save(dir)




if __name__=='__main__':
    if not os.path.exists(lidar_dir):
        os.makedirs(lidar_dir)

    rospy.init_node('velodyne_subscriber')
    #rospy.init_node('radar_subscriber')
    #rospy.init_node('camera_subscriber')
    print ("Node init")
    #velodyne_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, callback)
    #radar_subscriber = rospy.Subscriber('/radar/points', PointCloud2, callback2)
    #velodyne_subscriber = rospy.Subscriber('/vehicle/sonar_cloud', PointCloud2, callback4)
    camera_subscriber = rospy.Subscriber('/image_raw', Image, callbackCamera)
    camera_pub = rospy.Publisher('/camera_pose', lidar_pose)
    rospy.spin()
