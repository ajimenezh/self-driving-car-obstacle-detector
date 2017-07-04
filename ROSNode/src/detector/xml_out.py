import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
import os
import xml.dom.minidom

test = True

def make(n,  y,  add="",  type="Car"):
    
    x = []
    idx = []
    i = 0
    while i < len(y):
        j = i
        w = []
        while j < len(y) and (abs(y[j][0]) > 1.0e-6 or abs(y[j][1]) > 1.0e-6):
            w.append(y[j])
            j = j+1
        if len(w) > 0:
            x.append(w)
            idx.append(i)
        i = j + 1
    
    path = "./" + add +".xml"

    top = Element('boost_serialization',  { "signature" : "serialization::archive",  "version":'9'})

    childTop = SubElement(top, 'tracklets',  {"class_id":'0',  "tracking_level":"0",  "version":"0"})

    tmp = SubElement(childTop, 'count')
    tmp.text = str(len(x))
    tmp = SubElement(childTop, 'item_version')
    
    for elem,  k in  zip(x,  idx):
        tmp.text = "1"
        child = SubElement(childTop, 'item',  {"class_id":"1",  "tracking_level":"0",  "version":"1"})

        subchild = SubElement(child, 'objectType')
        subchild.text = type
    
        if type == "Car":
            subchild = SubElement(child, 'h')
            subchild.text = "1.824000"

            subchild = SubElement(child, 'w')
            subchild.text = "2.074800"

            subchild = SubElement(child, 'l')
            subchild.text = "4.191000"
        else:
            subchild = SubElement(child, 'h')
            subchild.text = "1.824000"

            subchild = SubElement(child, 'w')
            subchild.text = "1.04800"

            subchild = SubElement(child, 'l')
            subchild.text = "3.191000"

        subchild = SubElement(child, 'first_frame')
        subchild.text = str(k)
        
        child2 = child
        child = SubElement(child, 'poses',  {"class_id":"2",  "tracking_level":"0",  "version":"0"})
        subchild = SubElement(child, 'count')
        subchild.text = str(len(elem))
        
        subchild = SubElement(child, 'item_version')
        subchild.text = "2"
        
        for i in range(0,  len(elem)):
            if (i == 0):
                subchild = SubElement(child, 'item',  {"class_id":"3",  "tracking_level":"0",  "version":"2"})
            else:
                subchild = SubElement(child, 'item')
            
            if i < len(y):
                tmp = SubElement(subchild,  "tx")
                tmp.text = str(elem[i][0])
                
                tmp = SubElement(subchild,  "ty")
                tmp.text = str(elem[i][1])
                
                tmp = SubElement(subchild,  "tz")
                tmp.text = str(elem[i][2])
            else:
                tmp = SubElement(subchild,  "tx")
                tmp.text = "0.0"
                
                tmp = SubElement(subchild,  "ty")
                tmp.text = "0.0"
                
                tmp = SubElement(subchild,  "tz")
                tmp.text = "0.0"
                
            tmp = SubElement(subchild,  "rx")
            tmp.text = "0.0"
            
            tmp = SubElement(subchild,  "ry")
            tmp.text = "0.0"
            
            tmp = SubElement(subchild,  "rz")
            tmp.text = "3.14"
            
            tmp = SubElement(subchild,  "state")
            tmp.text = "1"
            
            tmp = SubElement(subchild,  "occlusion")
            tmp.text = "-1"
            
            tmp = SubElement(subchild,  "occlusion_kf")
            tmp.text = "-1"
            
            tmp = SubElement(subchild,  "truncation")
            tmp.text = "-1"
            
            tmp = SubElement(subchild,  "amt_occlusion")
            tmp.text = "0.0"
            
            tmp = SubElement(subchild,  "amt_occlusion_kf")
            tmp.text = "-1"
            
            tmp = SubElement(subchild,  "amt_border_l")
            tmp.text = "0.0"
            
            tmp = SubElement(subchild,  "amt_border_r")
            tmp.text = "0.0"
            
            tmp = SubElement(subchild,  "amt_border_kf")
            tmp.text = "-1"
    
        child = SubElement(child2, 'finished')
        child.text = "1"
    
    xml2 = xml.dom.minidom.parseString(tostring(top))
    pretty_xml_as_string = xml2.toprettyxml()
    
    file = open(path,  "w")
    #file .write(tostring(top))
    file .write(pretty_xml_as_string)
    file.close()
    
    


