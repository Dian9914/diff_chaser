#!/usr/bin/env python
# nodo encargado de tomar decisiones en base a los datos de la imagen,
# actuar acorde sobre los actuadores y ofrecer un servicio al usuario
# para alterar el comportamiento del sistema durante el funcionamiento

import rospy

#importamos los mensajes
from diff_chaser.msg import camera_data, velocity_cmd

#importamos los servicio
from diff_chaser.srv import chase_conf



class Node():
        """A node class for A* Pathfinding"""

        def __init__(self, parent=None, position=None):
            self.parent = parent
            self.position = position

            self.g = 0
            self.h = 0
            self.f = 0

        def __eq__(self, other):
            return self.position == other.position
class central_node():
    def __init__(self):
        # parametros de seguimiento
        self.max_lin = 1    #velocidad lineal maxima a la que se persiguen objetos
        self.max_ang = 1    #velocidad angular maxima a la que se persiguen objetos
        self.target_color = 'r'  #color que se persigue, de serie es rojo, todos los demas se evitan

    def astar(self, maze, start, end):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

    def process_data(self, data):
        #primero leemos la informacion referente al color que deseamos seguir
        if self.target_color == 'r':
            self.target_pos = data.red.center
            self.target_dist = 3000-data.red.area
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

        print self.obs1_dist
        print self.obs2_dist
        print self.target_dist

        # una vez tenemos la informacion, comenzamos el control
        if self.target_pos==-1:
        # si el objeto de color deseado no esta en el encuadre de la camara, comenzamos a dar vueltas sobre nosotros mismos buscando el objeto
            ref_pos=128
            ref_dist=1000
        elif (self.obs1_pos==-1 or self.obs1_dist>self.target_dist) and (self.obs2_pos==-1 or self.obs2_dist>self.target_dist): 
            #si no hay obstaculo a la vista, simplemente seguimos el objetivo
            ref_pos=self.target_pos
            ref_dist=self.target_dist
        else:
            # si hay algun obstaculo en nuestro encuadre debemos tenerlo en cuenta para decidir la trayectoria
            # se considera obstaculo si esta por delante del objetivo. Para esquivarlo, construimos un mapa local
            # el mapa local es una cuadricula 3x3 que separa el espacio que podemos observar en la camara y que nos ayuda a tomar decisiones para esquivar
            map=[[0,0,0,0,0],
                 [0,0,0,0,0],
                 [0,0,0,0,0],
                 [0,0,0,0,0]] 
            # 0 significa vacio, 1 es obstaculo y -1 el objetivo

            #primero registramos la posicion de nuestro objetivo en el mapa local
            if self.target_pos>=0 and self.target_pos<30:
                t_pos=1
            elif self.obs1_pos>=30 and self.obs1_pos<100:
                t_pos=2
            elif self.obs1_pos>=100 and self.obs1_pos<128:
                t_pos=3
            if self.target_dist<2000:
                t_dis=1
            elif self.target_dist>=2000 and self.target_dist<2750:
                t_dis=2
            elif self.target_dist>=2750 and self.target_dist<4000:
                t_dis=3

            if (self.obs1_dist<self.target_dist) and (self.obs1_dist!=-1):
                #si hay un obstaculo, hemos de esquivarlo. Para esquivarlo, primero lo registramos en nuestro mapa local
                if self.obs1_pos>=0 and self.obs1_pos<30:
                    o1_pos=1
                elif self.obs1_pos>=30 and self.obs1_pos<100:
                    o1_pos=2
                elif self.obs1_pos>=100 and self.obs1_pos<128:
                    o1_pos=3

                if self.obs1_dist<2000:
                    o1_dis=1
                elif self.obs1_dist>=2000 and self.obs1_dist<2750:
                    o1_dis=2
                elif self.obs1_dist>=2750 and self.obs1_dist<4000:
                    o1_dis=3
                print o1_dis
                map[o1_dis][o1_pos]=1
            
            if(self.obs2_dist<self.target_dist) and (self.obs2_dist!=-1):
                #si hay un obstaculo, hemos de esquivarlo. Para esquivarlo, primero lo registramos en nuestro mapa local
                if self.obs1_pos>=0 and self.obs1_pos<30:
                    o2_pos=1
                elif self.obs1_pos>=30 and self.obs1_pos<100:
                    o2_pos=2
                elif self.obs1_pos>=100 and self.obs1_pos<128:
                    o2_pos=3

                if self.obs1_dist<1000:
                    o2_dis=1
                elif self.obs2_dist>=1000 and self.obs2_dist<2000:
                    o2_dis=2
                elif self.obs2_dist>=2000 and self.obs2_dist<4000:
                    o2_dis=3

                print o2_dis
                map[o2_dis][o2_pos]=1
            
            #a continuacion hemos de tomar una decision respecto a la trayectoria a tomar segun el mapa 
            #utilizaremos un algoritmo de planificacion Astar, anque solo utilizaremos el primer waypoint
            #como son mapas de 5x4 el tiempo de computo sera tan pequenio que merecera la pena 
            map[t_dis][t_pos]=2
            print(map)
            map[t_dis][t_pos]=0
            start = (0, 2)
            end = (t_dis, t_pos)

            path = self.astar(map, start, end)
            print(path)

            #extraemos de la trayectoria el camino a seguir a continuacion
            for point in path:
                if point[0]==1:
                    tray=point[1]
                    break

            if tray==0:
                ref_pos=-30
            elif tray==1:
                ref_pos=20
            elif tray==2:
                ref_pos=64
            elif tray==3:
                ref_pos=100
            elif tray==4:
                ref_pos=140

            ref_dist=self.target_dist

        
        # a continuacion hemos de calcular la accion del control
        # usamos para ello un controlador PI
        err_pos=64-ref_pos
        err_dist=ref_dist

        lin_vel=err_dist*0.0005
        ang_vel=err_pos*0.01
                

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