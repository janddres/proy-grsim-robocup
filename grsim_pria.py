#! /usr/bin/env python3

import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionBall  # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador
from pelota import Pelota

ball = SSL_DetectionBall()  # Creamos una instancia del mensaje SSL_DetectionBall
#pelota = Pelota()
golero = Jugador('golero')
defensa1= Jugador('defensa1')
defensa2 = Jugador('defensa2')
atacante1 = Jugador('atacante1')
atacante2 = Jugador('atacante2')

def vision_callback(data):
    global ball, golero, defensa1, defensa2, atacante1, atacante2
    ball = data.balls  # Actualizamos la información del balón
    print('ball  ', ball)
    # Recorremos la lista de robots azules detectados en el marco de detección
    if len(data.robots_blue) > 0:
        for robot in data.robots_blue:
            if robot.robot_id == 0:
                golero.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 0
                golero.set_orientacion(robot.orientation)
            if robot.robot_id == 1:
                defensa1.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 1
                defensa1.set_orientacion(robot.orientation)
            if robot.robot_id == 2:
                defensa2.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 2
                defensa2.set_orientacion(robot.orientation)
            if robot.robot_id == 3:
                atacante1.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                atacante1.set_orientacion(robot.orientation)
            if robot.robot_id == 4:
                atacante2.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 4
                atacante2.set_orientacion(robot.orientation)
        #pelota.set_ubicacion(data.balls[0].x, data.balls[0].y)
if __name__ == "__main__":
    rospy.init_node("grsim_pria", anonymous=False)  # Inicializamos el nodo ROS con el nombre "grsim_pria"
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)  # Nos suscribimos al tópico "/vision" para recibir los marcos de detección
    golero.publisher= rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)  # Creamos un publicador para enviar comandos al robot 0 azul
    r = rospy.Rate(1000)  # Establecemos la frecuencia de publicación en 1000 Hz

    golero_x = 0  # Inicializamos la posición x del robot 0 en 0
    golero_y = 0  # Inicializamos la posición y del robot 0 en 0
    ball_x = 0  # Inicializamos la posición x del balón en 0
    ball_y = 0  # Inicializamos la posición y del balón en 0

    golero_msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al robot

    while not rospy.is_shutdown():
        try:
            ball_x = ball[0].x  # Obtenemos la posición x del balón #pelota.get_ubicacion()['x']
            ball_y = ball[0].y  # Obtenemos la posición y del balón #pelota.get_ubicacion()['y']
            golero_x = golero.get_ubicacion()['x'] # Obtenemos la posición x del robot 0
            golero_y = golero.get_ubicacion()['y']  # Obtenemos la posición y del robot 0
        except:
            pass

        ######################################################################################
#Golero

        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA  
        distance_ball_cua= ((ball_x - golero_x)**2 + (ball_y - golero_y )**2)
       
       
        if ball_x > -800 and distance_ball_cua> 2500:
            # Si la pelota esta lejos del golero y del arco el golero va a su posicion inicial
            # pos_x = -1500
            # pos_y = 0
            # posfrente_x=2000
            # posfrente_y=0
            dis_cerca=10000

            #distance_pos = (pos_x - golero_x)**2 +( pos_y - golero_y )**2
            distance_pos = golero.set_posicion_distan()
           
            if distance_pos< dis_cerca:
                golero_msg = golero.mirar_frente(golero_msg)
                # golero_msg.cmd_vel.linear.x = 0
               
                # goal_angle = math.atan2(posfrente_y- golero_y , posfrente_x- golero_x)
                # heading_posfrente = goal_angle - golero.get_orientacion()
                # heading_posfrente= math.atan2(math.sin(heading_posfrente), math.cos(heading_posfrente))
                # #gira hasta mirar al frente
                # if abs(heading_posfrente)<0.2:
                #     golero_msg.cmd_vel.angular.z = 0
                # else:
                #     golero_msg.cmd_vel.linear.x = 0
                #     golero_msg.cmd_vel.angular.z = heading_posfrente*5+0.5 if heading_posfrente > 0 else heading_posfrente*5-0.5
                                    
            else:
                golero_msg = golero.ir_a_posicion(distance_pos,golero_msg)
                # # Si el golero no está en su posición objetivo, moverse hacia allá
                # goal_angle = math.atan2(pos_y- golero_y , pos_x- golero_x)
                # heading_pos = goal_angle - golero.get_orientacion()
                # heading_pos= math.atan2(math.sin(heading_pos), math.cos(heading_pos))

                # if abs(heading_pos)<0.2:
                #    # si la idea de la programacion es la misma, controlar con otro rango la velocidad de los jugadores
                #    golero_msg.cmd_vel.linear.x = (distance_pos /2000000)+ 0.5#1.5
                #    golero_msg.cmd_vel.angular.z = 0
                # else:
                #     golero_msg.cmd_vel.linear.x = 0
                #     golero_msg.cmd_vel.angular.z = heading_pos*5+0.5 if heading_pos > 0 else heading_pos*5-0.5

        else:

            # Si la pelota esta cerca del golero o se acerca al arco seguir la pelota
            if distance_ball_cua< 0.2:
                # Si esta cerca detenerse
                golero_msg.cmd_vel.linear.x = 0
                golero_msg.cmd_vel.angular.z = 0
               
                # programar pase
                #llamar metodo depende del jugador
                golero_msg.dribbler =False
                golero_msg.kicker = 3

            else:

                goal_angle = math.atan2(ball_y - golero_y , ball_x - golero_x)
                heading_ball = goal_angle - golero.get_orientacion()
                heading_ball = math.atan2(math.sin(heading_ball), math.cos(heading_ball))
       
                if abs(heading_ball) < 0.2:
                    # Si la orientación es correcta, avanzar hacia la pelota
                    golero_msg.cmd_vel.linear.x = (distance_ball_cua/4000000)+ 0.5#calculo de velocidad segun la distancia
                    golero_msg.cmd_vel.angular.z = 0
                else:

                    # Si la orientación no es correcta, girar
                    golero_msg.cmd_vel.linear.x = 0
                    golero_msg.cmd_vel.angular.z = heading_ball*5+0.5 if heading_ball > 0 else heading_ball*5-0.5
                                                                #control para que gire mas optimo







        golero.publisher.publish(golero_msg)  # Publicamos el mensaje de comandos para el robot 0 azul

        r.sleep()