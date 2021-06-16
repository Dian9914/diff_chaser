#!/usr/bin/env python
# nodo encargado de obtener los datos de imagen y extraer informacion
# acerca de los objetos encontrados

import rospy
from sensor_msgs.msg import Image

class image_processing():
    def __init__(self):
        self.img_sub=rospy.Subscriber('/robot/imagen',Image,self.read_img)

    def read_img(self, data):
        frame = data
        