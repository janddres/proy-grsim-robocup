#! /usr/bin/env python3

import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionBall  # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador
from pelota import Pelota

ball = SSL_DetectionBall()  # Creamos una instancia del mensaje SSL_DetectionBall
pelota = Pelota()
golero = Jugador('atacante1')
defensa1= Jugador('atacante1')
defensa2 = Jugador('atacante1')
atacante1 = Jugador('atacante1')
atacante2 = Jugador('atacante1')

def vision_callback(data):
    global ball, golero, defensa1, defensa2, atacante1, atacante2
    # Recorremos la lista de robots azules detectados en el marco de detección
    if len(data.robots_blue) > 0:
        for robot in data.robots_blue:
            if robot.robot_id == 0:
                golero.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                golero.set_orientacion(robot.orientation)
            if robot.robot_id == 1:
                defensa1.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                defensa1.set_orientacion(robot.orientation)
            if robot.robot_id == 2:
                defensa2.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                defensa2.set_orientacion(robot.orientation)
            if robot.robot_id == 3:
                atacante1.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                atacante1.set_orientacion(robot.orientation)
            if robot.robot_id == 4:
                atacante2.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                atacante2.set_orientacion(robot.orientation)
        ball = data.balls  # Actualizamos la información del balón
        print('ball  ', ball)
        #pelota.set_ubicacion(data.balls[0].x, data.balls[0].y)
if __name__ == "__main__":
    rospy.init_node("grsim_pria", anonymous=False)  # Inicializamos el nodo ROS con el nombre "grsim_pria"
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)  # Nos suscribimos al tópico "/vision" para recibir los marcos de detección
    atacante1.publisher= rospy.Publisher("/robot_blue_3/cmd", SSL, queue_size=10)  # Creamos un publicador para enviar comandos al robot 0 azul
    r = rospy.Rate(1000)  # Establecemos la frecuencia de publicación en 1000 Hz

    atacante1_x = 0  # Inicializamos la posición x del robot 0 en 0
    atacante1_y = 0  # Inicializamos la posición y del robot 0 en 0
    ball_x = 0  # Inicializamos la posición x del balón en 0
    ball_y = 0  # Inicializamos la posición y del balón en 0

    msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al robot

    while not rospy.is_shutdown():
        try:
            ball_x = ball[0].x  # Obtenemos la posición x del balón #pelota.get_ubicacion()['x'] 
            ball_y = ball[0].y  # Obtenemos la posición y del balón #pelota.get_ubicacion()['y'] 
            atacante1_x = atacante1.get_ubicacion()['x'] # Obtenemos la posición x del robot 0
            atacante1_y = atacante1.get_ubicacion()['y']  # Obtenemos la posición y del robot 0
        except:
            pass

        goal_angle = math.atan2(ball_y - atacante1_y, ball_x - atacante1_x)  # Calculamos el ángulo hacia el objetivo

        heading = abs(goal_angle - atacante1.get_orientacion())  # Calculamos la diferencia angular entre el objetivo y la orientación del robot

        distance = math.sqrt((ball_x - atacante1_x)**2 + (ball_y - atacante1_y)**2)  # Calculamos la distancia al objetivo

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

        atacante1.publisher.publish(msg)  # Publicamos el mensaje de comandos para el robot 0 azul
