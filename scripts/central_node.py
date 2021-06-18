#!/usr/bin/env python
# nodo encargado de tomar decisiones en base a los datos de la imagen,
# actuar acorde sobre los actuadores y ofrecer un servicio al usuario
# para alterar el comportamiento del sistema durante el funcionamiento

import rospy

#importamos los mensajes
from diff_chaser.msg import camera_data, velocity_cmd

#importamos los servicio
from diff_chaser.srv import chase_conf


class central_node():
    def __init__(self):
        # parametros de seguimiento
        self.max_lin = 1    #velocidad lineal maxima a la que se persiguen objetos
        self.max_ang = 1    #velocidad angular maxima a la que se persiguen objetos
        self.target_color = 'r'  #color que se persigue, de serie es rojo, todos los demas se evitan

    def process_data(self, data):
        #primero leemos la informacion referente al color que deseamos seguir
        if self.target_color == 'r':
            self.target_pos = data.red.center
            self.target_dist = 4000-data.red.area
            self.obs1_pos = data.green.center
            self.obs1_dist = 3000-data.green.area
            self.obs2_pos = data.blue.center
            self.obs2_dist = 3000-data.blue.area
        elif self.target_color == 'g':
            self.target_pos = data.green.center
            self.target_dist = 3000-data.green.area
            self.obs1_pos = data.red.center
            self.obs1_dist = 3000-data.red.area
            self.obs2_pos = data.blue.center
            self.obs2_dist = 3000-data.blue.area
        elif self.target_color == 'b':
            self.target_pos = data.blue.center
            self.target_dist = 3000-data.blue.area
            self.obs1_pos = data.green.center
            self.obs1_dist = 3000-data.green.area
            self.obs2_pos = data.red.center
            self.obs2_dist = 3000-data.red.area

        # una vez tenemos la informacion, comenzamos el control
        if self.target_pos==-1:
        # si el objeto de color deseado no esta en el encuadre de la camara, comenzamos a dar vueltas sobre nosotros mismos buscando el objeto
            ang_vel=1
            lin_vel=1
        else:
        # si el objeto de color deseado esta en el encuadre de la camara, debemos comprobar ahora si hay algun obstaculo en nuestra trayectoria inmediata
        # se considera obstaculo si esta en nuestra trayectoria inmediata, si esta mas cerca que el objetivo y si el otro obstaculo no esta mas cerca
        # o no existe
            if (self.obs1_pos>30 and self.obs1_pos<100 and self.obs1_dist<self.target_dist) and (self.obs1_dist<self.obs2_dist or self.obs2_pos==-1):
                #si hay un obstaculo, hemos de esquivarlo. Esquivamos primero el mas cercano
                #para esquivar, giramos hacia el objetivo mientras avanzamos a una velocidad proporcional a la distancia al obstaculo
                lin_vel=self.obs1_dist/10000
                ang_vel=(self.obs1_pos-self.target_pos)/10
            elif(self.obs2_pos>30 and self.obs2_pos<100 and self.obs2_dist<self.target_dist) and (self.obs2_dist<self.obs1_dist or self.obs1_pos==-1):
                lin_vel=self.obs2_dist/10000
                ang_vel=(self.obs2_pos-self.target_pos)/10
            else:
                #si no hay obstaculo a la vista, simplemente seguimos el objetivo
                lin_vel=self.target_dist/1000
                ang_vel=(64-self.target_pos)/20

        #finalmente, publicamos la accion
        vel_action=velocity_cmd()
        if lin_vel>self.max_lin:
            vel_action.lineal=self.max_lin
        elif lin_vel<-self.max_lin:
            vel_action.lineal=-self.max_lin
        else:
            vel_action.lineal=lin_vel

        if ang_vel>self.max_ang:
            vel_action.angular=self.max_ang
        elif ang_vel<-self.max_ang:
            vel_action.angular=-self.max_ang
        else:
            vel_action.angular=ang_vel

        self.action_pub.publish(vel_action)

    def change_params(self, request):
        if request.color:
            if request.color == 'r' or request.color == 'g' or request.color == 'b':
                self.target_color = request.color  
            else:
                rospy.logerr('CENTRAL_NODE: Unrecogniced color input.') 
                return False 
        
        if request.limits.lineal > 0.1:
            self.max_lin = request.limits.lineal
        
        if request.limits.angular > 0.1:
            self.max_ang = request.limits.angular

        rospy.loginfo('CENTRAL_NODE: Params changed.\nCurrent params are:\ncolor: %s\tlinvel limit: %.2f\tangvel limit: %.2f',self.target_color, self.max_lin, self.max_ang)

        return True

    def start_node(self):
        # creamos el servicio que permite cambiar los parametros de seguimiento
        self.param_service=rospy.Service('/diff/config',chase_conf, self.change_params)

        # creamos el publisher para publicar la accion de control
        self.action_pub=rospy.Publisher('/diff/cmd_vel', velocity_cmd, queue_size=10)

        # nos suscribimos al topic con los datos extraidos de la camara
        self.img_sub=rospy.Subscriber('/diff/camera_data', camera_data, self.process_data)

if __name__ == "__main__":
    rospy.init_node('central_node')
    rospy.loginfo('CENTRAL_NODE: Node started.')
    obj = central_node()
    obj.start_node()
    rospy.spin()