from sklearn import linear_model
import math
import csv
import numpy as np
import math
from sklearn.neural_network import MLPRegressor 
from sklearn import neighbors

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

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
    
class Elem:
    def __init__(self):
        self.box = []
        self.d = []

def get_data(path,  boxes_path,  test_path, times):
    
    onlyOne = True
    train_data = open(path)

    all = []

    for line in train_data:
        words = line[:-1].split(' ')
        
        tx = float(words[1])
        ty = float(words[2])
        tz = float(words[3])
        
        obj = Elem()
        obj.d = [tx,  ty,  tz]
        
        all.append(obj)
        
    train_data.close()

    key_data = open(boxes_path) 

    lines = key_data.readlines()

    key_data.close()

    i = 0
    m = 0

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        
        while i+1 < len(lines) and len(lines[i+1]) > 7:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            i = i+1
            if len(all[id].box) == 0 or all[id].box[2] < box[2]:
                all[id].box = box
        i = i+1

    i = 0
    x  = []
    y = []
    while i < m:
        tmpx = []
        tmpy = []
        if len(all[i].box) > 0:
            tmpx = all[i].box
            tmpx[1] = tmpx[1] + 500
            tmpy = all[i].d
        
        i = i + 1
        if not onlyOne:
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpy = all[i].d
        
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 855
                tmpy = all[i].d
                
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 456
                tmpy = all[i].d
                
            i = i+1
        
        if len(tmpx) > 0:
            default = True
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2,  (tmpx[2]+tmpx[3])/2]
            else:
                xx = tmpx[0] + tmpx[2]/2 - 1368/2
                yy = tmpx[1] + tmpx[3]/2
                yaw2 = math.atan2(yy,  xx)
                tmpx = [tmpx[3],  yaw2]
            x.append(tmpx)
            y.append(tmpy)

    m = len(x)

    n = m

    X = []
    Y = []
    i = 0
    for ex,  ey in zip(x,  y):
        X.append(ex)
        Y.append(ey)
        i = i+1

    n_neighbors = 80
    n = len(X)
    knn = neighbors.KNeighborsRegressor(n_neighbors, weights='distance')
    reg = knn.fit(X[:n], Y[:n])

    #reg = MLPRegressor(hidden_layer_sizes=(20,),solver="lbfgs")
    #reg.fit(X[:n], Y[:n])

    #reg = linear_model.LinearRegression()
    #reg.fit(X[:n], Y[:n])

#    j = n
#    for i in X[n:]:
#        print reg.predict(i)[0],  Y[j]
#        j = j+1

    test_data = open(test_path) 

    lines = test_data.readlines()

    test_data.close()

    i = 0
    m = 0

    all = {}

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        i = i+1
        
        while i+1 < len(lines) and len(lines[i+1][:-1].split(' ')) > 2:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            box = [box[0], box[1], box[2]-box[0], box[3]-box[1]]  
            i = i+1
            all[id] = box
        i = i+1

    data = []
    for t in times:
        if t in all:
            data.append(all[t])
        else:
            data.append([])

    all = data
    i = 0
    j = 0
    x_test  = []
    m = len(all)
    y_pred = []

    while i < m:
        tmpx = []
        tmpy = []
        if len(all[i]) > 0:
            tmpx = all[i]
            tmpx[1] = tmpx[1] + 500
        
        i = i + 1
        
        if len(tmpx) > 0:
            default = True
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2,   (tmpx[2]+tmpx[3])/2]
            else:
                xx = tmpx[0] + tmpx[2]/2 - 1368/2
                yy = tmpx[1] + tmpx[3]/2
                yaw2 = math.atan2(yy,  xx)
                tmpx = [tmpx[3],  yaw2]
            y_pred.append(reg.predict(tmpx)[0])
        else:
            y_pred.append([0.0,  0.0,  0.0])
            
        j = j+1
        
    return y_pred
    
def get_data_angles(path,  boxes_path,  test_path, times):
    
    onlyOne = True
    train_data = open(path)

    all = []

    for line in train_data:
        words = line[:-1].split(' ')
        
        tx = float(words[1])
        ty = float(words[2])
        tz = float(words[3])
        
        obj = Elem()
        obj.d = [tx,  ty,  tz]
        
        all.append(obj)
        
    train_data.close()

    key_data = open(boxes_path) 

    lines = key_data.readlines()

    key_data.close()

    i = 0
    m = 0

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        
        while i+1 < len(lines) and len(lines[i+1]) > 7:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            i = i+1
            if len(all[id].box) == 0 or all[id].box[2] < box[2]:
                all[id].box = box
        i = i+1

    i = 0
    x  = []
    y = []
    while i < m:
        tmpx = []
        tmpy = []
        if len(all[i].box) > 0:
            tmpx = all[i].box
            tmpx[1] = tmpx[1] + 500
            tmpy = all[i].d
        
        i = i + 1
        if not onlyOne:
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpy = all[i].d
        
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 855
                tmpy = all[i].d
                
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 456
                tmpy = all[i].d
                
            i = i+1
        
        if len(tmpx) > 0:
            default = False
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
            else:
                HEIGHT = 1368
                WIDTH = 1096
                tmp = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
                tmp = [WIDTH - tmp[1], -tmp[0] + HEIGHT/2]
                val1 = math.atan2(tmp[1],  tmp[0])
                tmpx = [val1]
            val = math.atan2(tmpy[1],  tmpy[0])
            x.append(tmpx)
            y.append(val)

    m = len(x)
    n = m
    
    X = []
    Y = []
    i = 0
    for ex,  ey in zip(x,  y):
        X.append(ex)
        Y.append(ey)
        i = i+1
        
    #import matplotlib.pyplot as plt
    #plt.plot(x, y)
    #plt.show()

    from sklearn import neighbors
    n_neighbors = 20
    n = len(X)
    #knn = neighbors.KNeighborsRegressor(n_neighbors, weights='distance')
    #reg = knn.fit(X[:n], Y[:n])
    
#    from sklearn.pipeline import Pipeline
#    from sklearn.preprocessing import PolynomialFeatures
#    from sklearn.model_selection import cross_val_score
#    from sklearn.linear_model import LinearRegression
#    polynomial_features = PolynomialFeatures(degree=1,
#                                             include_bias=False)
#    linear_regression = LinearRegression()
#    pipeline = Pipeline([("polynomial_features", polynomial_features),
#                         ("linear_regression", linear_regression)])
#    pipeline.fit(X[:n], Y[:n])
#    scores = cross_val_score(pipeline, X[:n], Y[:n],
#                             scoring="neg_mean_squared_error", cv=10)
    
    #reg = MLPRegressor(hidden_layer_sizes=(100,),solver="lbfgs", max_iter=500)
    #reg.fit(X[:n], Y[:n])

    reg = linear_model.LinearRegression()
    reg.fit(X[:n], Y[:n])

#    j = n
#    for i in X[n:]:
#        print reg.predict(i)[0],  Y[j]
#        j = j+1

    test_data = open(test_path) 

    lines = test_data.readlines()

    test_data.close()

    i = 0
    m = 0

    all = {}

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        i = i+1
        
        while i+1 < len(lines) and len(lines[i+1][:-1].split(' ')) > 2:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            box = [box[0], box[1], box[2]-box[0], box[3]-box[1]]        
            i = i+1
            all[id] = box
        i = i+1

    data = []
    for t in times:
        if t in all:
            data.append(all[t])
        else:
            data.append([])

    all = data
    i = 0
    j = 0
    x_test  = []
    m = len(all)
    y_pred = []

    while i < m:
        tmpx = []
        tmpy = []
        if len(all[i]) > 0:
            tmpx = all[i]
            tmpx[1] = tmpx[1]
        
        i = i + 1
        
        if len(tmpx) > 0:
            default = False
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
            else:
                HEIGHT = 1368
                WIDTH = 1096
                tmp = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
                #print tmp
                tmp = [WIDTH - tmp[1], -tmp[0] + HEIGHT/2]
                #print tmp
                val = math.atan2(tmp[1],  tmp[0])
            
            #print val,  reg.predict([val])
            #y_pred.append(reg.predict([val])[0])
            result = reg.predict([abs(val)])[0]
            result = result * val / abs(val)
            print val,  result,  val*0.2/0.5
            result = val*0.2/0.5
            y_pred.append(result)
        else:
            y_pred.append(1000.0)
            
        j = j+1
        
    return y_pred
    
def get_data_distances(path,  boxes_path,  test_path, times):
    
    onlyOne = True
    train_data = open(path)

    all = []

    for line in train_data:
        words = line[:-1].split(' ')
        
        tx = float(words[1])
        ty = float(words[2])
        tz = float(words[3])
        
        obj = Elem()
        obj.d = [tx,  ty,  tz]
        
        all.append(obj)
        
    train_data.close()

    key_data = open(boxes_path) 

    lines = key_data.readlines()

    key_data.close()

    i = 0
    m = 0

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        
        while i+1 < len(lines) and len(lines[i+1]) > 7:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            i = i+1
            if len(all[id].box) == 0 or all[id].box[2] < box[2]:
                all[id].box = box
        i = i+1

    i = 0
    x  = []
    y = []
    while i < m:
        tmpx = []
        tmpy = []
        if len(all[i].box) > 0:
            tmpx = all[i].box
            tmpx[1] = tmpx[1] + 500
            tmpy = all[i].d
        
        i = i + 1
        if not onlyOne:
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpy = all[i].d
        
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 855
                tmpy = all[i].d
                
            i = i + 1
            if len(all[i].box) > 0 and (len(tmpx) == 0 or tmpx[2] < all[i].box[2]):
                tmpx = all[i].box
                tmpx[1] = tmpx[1] + 500
                tmpx[0] = tmpx[0] + 456
                tmpy = all[i].d
                
            i = i+1
        
        if len(tmpx) > 0:
            default = False
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
            else:
                tmpx = [tmpx[3]]
            val = math.sqrt(tmpy[0]*tmpy[0] + tmpy[1]*tmpy[1])
            #print "-->"
            #print val
            #print tmpx,  tmpy
            x.append(tmpx)
            y.append(val)

    m = len(x)
    n = m

    X = []
    Y = []
    i = 0
    for ex,  ey in zip(x,  y):
        X.append(ex)
        Y.append(ey)
        i = i+1

    from sklearn import neighbors
    n_neighbors = 8
    n = len(X)
    #knn = neighbors.KNeighborsRegressor(n_neighbors, weights='distance')
    #reg = knn.fit(X[:n], Y[:n])

    reg = MLPRegressor(hidden_layer_sizes=(20,),solver="lbfgs")
    reg.fit(X[:n], Y[:n])

    #reg = linear_model.LinearRegression()
    #reg.fit(X[:n], Y[:n])

    test_data = open(test_path) 

    lines = test_data.readlines()

    test_data.close()

    i = 0
    m = 0

    all = {}

    while i < len(lines):
        line = lines[i]
        id = int(line[:-1])
        m = id+1
        i = i+1
        
        while i+1 < len(lines) and len(lines[i+1][:-1].split(' ')) > 2:
            words = lines[i+1][:-1].split(' ')
            box = [float(words[0]),  float(words[1]), float(words[2]), float(words[3])]
            box = [box[0], box[1], box[2]-box[0], box[3]-box[1]]                        
            i = i+1
            all[id] = box
        i = i+1

    data = []
    for t in times:
        if t in all:
            data.append(all[t])
        else:
            data.append([])

    all = data
    i = 0
    j = 0
    x_test  = []
    m = len(all)
    y_pred = []

    while i < m:
        tmpx = []
        tmpy = []
        
        height = 0
        n = 0
        
        if len(all[i]) > 0:
            tmpx = all[i]
            tmpx[1] = tmpx[1]
            height += tmpx[3]
            n += 1
        
        i = i + 1
        
        if n>0:
            tmpx[3] = height / n
        
        if len(tmpx) > 0:
            default = False
            tmp = tmpx
            if default:
                tmpx = [tmpx[0] + tmpx[2]/2,  tmpx[1] + tmpx[3]/2]
            else:
                 tmpx = [tmpx[3]]
            #res = reg.predict(tmpx)[0]
            
            P = tmp[3]
            F = 1460.62
            F = 1580.46
            W = (1.0*tmp[2] / tmp[3])*1.57
            W = 1.524
            D = F*W/P
            #print tmp
            #print P,  W, D
            res = D
            
            y_pred.append(res)
            #print tmpx,  y_pred[-1]
        else:
            y_pred.append(0.0)
            
        j = j+1
        
    return y_pred


