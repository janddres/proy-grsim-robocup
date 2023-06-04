#! /usr/bin/env python3

import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador
from pelota import Pelota

ball = Pelota()  # Creamos una instancia del mensaje SSL_DetectionBall
atacante1 = Jugador('atacante1')
#robot0 = SSL_DetectionRobot()  # Creamos una instancia del mensaje SSL_DetectionRobot para el robot 0
#robot1 = SSL_DetectionRobot()  # Creamos una instancia del mensaje SSL_DetectionRobot para el robot 1
#robot2 = SSL_DetectionRobot()  # Creamos una instancia del mensaje SSL_DetectionRobot para el robot 2
#atacante1 = SSL_DetectionRobot()  # Creamos una instancia del mensaje SSL_DetectionRobot para el robot 3
#robot4 = SSL_DetectionRobot()  # Creamos una instancia del mensaje SSL_DetectionRobot para el robot 4




def vision_callback(data):
    global ball, golero, defensa1, defensa2, atacante1, atacante2
    # Recorremos la lista de robots azules detectados en el marco de detección
    for i in range(0, len(data.robots_blue)):
        if data.robots_blue[i].robot_id == 0:
            robot0 = data.robots_blue[i]  # Actualizamos la información del robot 0
        if data.robots_blue[i].robot_id == 1:
            robot1 = data.robots_blue[i]  # Actualizamos la información del robot 1
        if data.robots_blue[i].robot_id == 2:
            robot2 = data.robots_blue[i]  # Actualizamos la información del robot 2
        if data.robots_blue[i].robot_id == 3:
            atacante1 = data.robots_blue[i]  # Actualizamos la información del robot 3
        if data.robots_blue[i].robot_id == 4:
            robot4 = data.robots_blue[i]  # Actualizamos la información del robot 4
    ball = data.balls  # Actualizamos la información del balón
    print('pelota  ', data.balls)
if __name__ == "__main__":
    rospy.init_node("grsim_pria", anonymous=False)  # Inicializamos el nodo ROS con el nombre "grsim_pria"
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)  # Nos suscribimos al tópico "/vision" para recibir los marcos de detección
    pub_robot_3_blue = rospy.Publisher("/robot_blue_3/cmd", SSL, queue_size=10)  # Creamos un publicador para enviar comandos al robot 0 azul
    r = rospy.Rate(1000)  # Establecemos la frecuencia de publicación en 1000 Hz

    #atacante1_x = 0  # Inicializamos la posición x del robot 0 en 0
    #atacante1_y = 0  # Inicializamos la posición y del robot 0 en 0
    atacante1.set_ubicacion(0,0)
    atacante1_orientacion = 0
    ball_x = 0  # Inicializamos la posición x del balón en 0
    ball_y = 0  # Inicializamos la posición y del balón en 0

    msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al robot

    while not rospy.is_shutdown():
        try:
            ball_x = ball[0].x  # Obtenemos la posición x del balón
            ball_y = ball[0].y  # Obtenemos la posición y del balón
            #atacante1_x = atacante1.pixel_x  # Obtenemos la posición x del robot 0
            #atacante1_y = atacante1.pixel_y  # Obtenemos la posición y del robot 0
            atacante1.set_ubicacion(atacante1.x,atacante1.y)
            atacante1_orientacion = atacante1.posicion
        except:
            pass

        goal_angle = math.atan2(ball_y - atacante1.get_ubicacion()['y'], ball_x - atacante1.get_ubicacion()['x'])  # Calculamos el ángulo hacia el objetivo

        heading = abs(goal_angle - atacante1_orientacion)  # Calculamos la diferencia angular entre el objetivo y la orientación del robot

        distance = math.sqrt((ball_x - atacante1.get_ubicacion()['x'])**2 + (ball_y - atacante1.get_ubicacion()['y'])**2)  # Calculamos la distancia al objetivo

        if distance < 0.2:  # Si la distancia al objetivo es menor a 0.2
            msg.cmd_vel.linear.x = 0  # Detenemos el movimiento lineal
            msg.cmd_vel.angular.z = 0  # Detenemos el movimiento angular
        else:
            if heading < 0.2:  # Si la diferencia angular es menor a 0.2
                msg.cmd_vel.linear.x = 0.25  # Movemos hacia adelante con una velocidad lineal de 0.25
                msg.cmd_vel.angular.z = 0  # No hay movimiento angular
            else:
                msg.cmd_vel.linear.x = 0  # No hay movimiento lineal
                msg.cmd_vel.angular.z = 0.5  # Rotamos con una velocidad angular de 0.5

        #msg.dribbler = True  # Activamos el dribbler
        msg.kicker = 1.0 # Activamos el kicker

        #print(heading, distance)  # Imprimimos la diferencia angular y la distancia

        pub_robot_3_blue.publish(msg)  # Publicamos el mensaje de comandos para el robot 0 azul
