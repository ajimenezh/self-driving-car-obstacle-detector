import math
import numpy as np

def binary_search(x,  v):
    lo = 0
    hi = len(v) - 1
    
    while lo < hi:
        mi = int((lo+hi+1)/2)
        
        if v[mi] <= x:
            lo = mi
        else:
            hi = mi-1
            
    return lo
    
def interpolate_aprox(x,  x0,  y0,  x1,  y1):
    
    if x1 == x0:
        return y0
        
    return np.interp(x,  [x0,  x1],  [y0,  y1])
        
    x0 = (x0/1000000)%100000000
    x1 = (x1/1000000)%100000000
    x = (x/1000000)%100000000
        
    m = (1.0*y0 - y1) / (1.0*x0 - x1)
    
    return m*(x - x0) + y0
    
def interpolate(x,  x0,  y0,  x1,  y1):
    
    if x1 == x0:
        return y0
        
    m = (1.0*y0 - y1) / (1.0*x0 - x1)

    print x0,  x1,  m,  m*(x-x0)
    
    return m*(x - x0) + y0
    
def distance_2d(x,  y):
    d = 0
    for i in range(2):
        d += (x[i] - y[i])*(x[i] - y[i])
    
    return math.sqrt(d)
    
def distance_ang(x,  y):
    d = distance_2d(x,  y)
    ang1 = math.atan2(x[1],  x[0])
    ang2 = math.atan2(y[1],  y[0])
    return d + abs(ang1 - ang2)*0
    
def isempty(x):
    if len(x) < 1:
        return True
    for i in x:
        if abs(i) > 1.0e-6:
            return False
    return True
    
def get_yaw(p1, p2):
    return np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
    
def rotMatZ(a):
    cos = np.cos(a)
    sin = np.sin(a)
    return np.array([
        [cos, -sin, 0.],
        [sin, cos,  0.],
        [0,    0,   1.]
    ])

from sensor_msgs.msg import PointField

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
