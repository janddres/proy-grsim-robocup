#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from grsim_ros_bridge_msgs.msg import SSL
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionRobot, SSL_DetectionBall
import math

# Variables globales para almacenar la información de la pelota y los robots
ball = SSL_DetectionBall()
goalkeeper = SSL_DetectionRobot()
defense_right = SSL_DetectionRobot()
defense_left = SSL_DetectionRobot()
attacker1 = SSL_DetectionRobot()
attacker2 = SSL_DetectionRobot()


# Función de callback para recibir los datos de visión
def vision_callback(data):
    global ball, goalkeeper, defense_right, defense_left, attacker1, attacker2
    for i in range(0, len(data.robots_blue)):
        if (data.robots_blue[i].robot_id == 0):
            goalkeeper = data.robots_blue[i]
            print('prueba', data.robots_blue[0])
        if (data.robots_blue[i].robot_id == 1):
            defense_right = data.robots_blue[i]
        if (data.robots_blue[i].robot_id == 2):
            defense_left = data.robots_blue[i]
        if (data.robots_blue[i].robot_id == 3):
            attacker1 = data.robots_blue[i]
        if (data.robots_blue[i].robot_id == 4):
            attacker2 = data.robots_blue[i]
            
    ball = data.balls
    #print(data.robots_blue[0].x, data.robots_blue[0].y)

if __name__=="__main__":
    # Inicializar el nodo de ROS
    rospy.init_node("andres", anonymous=False)
    
    # Suscribirse al tópico "/vision" para recibir los datos de detección
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)
    
    # Crear un publicador para enviar comandos a los jugadores
    pub_goalkeeper = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)
    pub_defense_right = rospy.Publisher("/robot_blue_1/cmd", SSL, queue_size=10)
    pub_defense_left = rospy.Publisher("/robot_blue_2/cmd", SSL, queue_size=10)
    pub_attacker1 = rospy.Publisher("/robot_blue_3/cmd", SSL, queue_size=10)
    pub_attacker2 = rospy.Publisher("/robot_blue_4/cmd", SSL, queue_size=10)
    
    # Configurar la tasa de publicación del nodo en 1000 Hz
    r = rospy.Rate(1000)
    
    # Variables para almacenar las posiciones de la pelota y los jugadores
    ball_x = 0
    ball_y = 0
    goalkeeper_x= 1.8
    goalkeeper_y= 0
    defense_right_x = 0
    defense_right_y = 0
    defense_left_x = 0
    defense_left_y= 0
    attacker1_x= 0
    attacker1_y= 0
    attacker2_x= 0
    attacker2_y= 0
    
	# creamos instancia de los mensajes de SSL

    msg_goalkeeper = SSL()
    msg_defense_right = SSL()
    msg_defense_left = SSL()
    msg_attacker1 = SSL()
    msg_attacker2 = SSL()



    while not rospy.is_shutdown():
        try:
            # Actualizar las posiciones de la pelota y los jugadores
            ball_x = ball[0].x
            ball_y = ball[0].y
            goalkeeper_x= goalkeeper.x
            goalkeeper_y= goalkeeper.y
            defense_right_x =  defense_right.x
            defense_right_y =  defense_right.y
            defense_left_x =  defense_left.x
            defense_left_y = defense_left.y
            attacker1_x =  attacker1.x
            attacker1_y =  attacker1.y
            attacker2_x =  attacker2.x
            attacker2_y =  attacker2.y
            
            
        except:
            pass
        
		#Golero
           
        #goalkeeper_x = goalkeeper.x
        #goalkeeper_y = goalkeeper.y
        # 
        goal_angle = math.atan2(ball_y - goalkeeper_y, ball_x - goalkeeper_x)
        heading_ball= abs(goal_angle - goalkeeper.orientation)
        distance_ball= math.sqrt((ball_x - goalkeeper_x)**2 + (ball_y - goalkeeper_y)**2)
        #msg_goalkeeper = SSL()
        distance_arco = math.sqrt((-2000- goalkeeper_x)**2 + (0 - goalkeeper_y)**2)
        if ball_x > -1000 and distance_ball> 1000 :
            # Si la pelota esta lejos del golero y del arco el golero va a su posicion inicial
            #goal_x = -4500
            #goal_y = 0
            goal_angle = math.atan2(0- goalkeeper_y, -2000 - goalkeeper_x)
            #heading = abs(goal_angle - goalkeeper.orientation)
            heading = goal_angle - goalkeeper.orientation
            heading_arco = math.atan2( math.sin(heading), math.cos(heading))
            

     
            if distance_arco< 0.2:
                # Si el golero está cerca de su posición objetivo, detenerse
                msg_goalkeeper.cmd_vel.linear.x = 0
                msg_goalkeeper.cmd_vel.angular.z = 0
            else:
                # Si el golero no está en su posición objetivo, moverse hacia allá
                if abs(heading_arco)<0.2:
                   msg_goalkeeper.cmd_vel.linear.x = 0.5
                   msg_goalkeeper.cmd_vel.angular.z = 0
                else:
                    msg_goalkeeper.cmd_vel.linear.x = 0
                    msg_goalkeeper.cmd_vel.angular.z = 0.5 if heading > 0 else -0.5

        else:

            # Si la pelota esta cerca del golero o se acerca al arco seguir la pelota
            if distance_ball< 0.2:
                # Si la orientación es correcta, avanzar hacia la pelota
                msg_goalkeeper.cmd_vel.linear.x = 0
                msg_goalkeeper.cmd_vel.angular.z = 0
                
                # programar pase
            else:
                if heading_ball<0.2:
                   msg_goalkeeper.cmd_vel.linear.x = 0.5
                   msg_goalkeeper.cmd_vel.angular.z = 0
                else:
                    msg_goalkeeper.cmd_vel.linear.x = 0
                    msg_goalkeeper.cmd_vel.angular.z = 0.5 
        
        msg_goalkeeper.kicker = True
        msg_goalkeeper.dribbler = True
       
        """
        # Defensa derecho
        defense_right_x = defense_right.x
        defense_right_y = defense_right.y
        goal_angle_defense_right = math.atan2(ball_y - defense_right_y, ball_x - defense_right_x)
        heading_defense_right = abs(goal_angle_defense_right - defense_right.orientation)
        distance_defense_right = math.sqrt((ball_x - defense_right_x)**2 + (ball_y - defense_right_y)**2)
        msg_defense_right = SSL()
        # Lógica de movimiento y acciones para el defensa derecho
        if defense_right_x > -1.5:
            # Si el defensa derecho está en la mitad izquierda del campo, moverse hacia la mitad de la cancha
            target_x = -1
            target_y = 0
            goal_angle = math.atan2(target_y - defense_right_y, target_x - defense_right_x)
            heading = abs(goal_angle - defense_right.orientation)
            distance = math.sqrt((target_x - defense_right_x)**2 + (target_y - defense_right_y)**2)
            if distance < 0.2:
                # Si el defensa derecho está cerca de su posición objetivo, detenerse
                msg_defense_right.cmd_vel.linear.x = 0
                msg_defense_right.cmd_vel.angular.z = 0
            else:
                # Si el defensa derecho no está en su posición objetivo, moverse hacia allá
                msg_defense_right.cmd_vel.linear.x = 0.25
                msg_defense_right.cmd_vel.angular.z = 0
        else:
            # Si el defensa derecho está en la mitad derecha del campo, seguir la pelota
            if heading_defense_right < 0.2:
                # Si la orientación es correcta, avanzar hacia la pelota
                msg_defense_right.cmd_vel.linear.x = 0.25
                msg_defense_right.cmd_vel.angular.z = 0
            else:
                # Si la orientación no es correcta, girar hacia la pelota
                msg_defense_right.cmd_vel.linear.x = 0.
                msg_defense_right.cmd_vel.angular.z = 0.5
        
        msg_defense_right.kicker = True
        msg_defense_right.dribbler = True
        
        # Defensa izquierdo
        defense_left_x = defense_left.x
        defense_left_y = defense_left.y
        goal_angle_defense_left = math.atan2(ball_y - defense_left_y, ball_x - defense_left_x)
        heading_defense_left = abs(goal_angle_defense_left - defense_left.orientation)
        distance_defense_left = math.sqrt((ball_x - defense_left_x)**2 + (ball_y - defense_left_y)**2)
        msg_defense_left = SSL()
        # Lógica de movimiento y acciones para el defensa izquierdo
        if defense_left_x > -1.5:
            # Si el defensa izquierdo está en la mitad izquierda del campo, moverse hacia la mitad de la cancha
            target_x = -1
            target_y = 0
            goal_angle = math.atan2(target_y - defense_left_y, target_x - defense_left_x)
            heading = abs(goal_angle - defense_left.orientation)
            distance = math.sqrt((target_x - defense_left_x)**2 + (target_y - defense_left_y)**2)
            if distance < 0.2:
                # Si el defensa izquierdo está cerca de su posición objetivo, detenerse
                msg_defense_left.cmd_vel.linear.x = 0
                msg_defense_left.cmd_vel.angular.z = 0
            else:
                # Si el defensa izquierdo no está en su posición objetivo, moverse hacia allá
                msg_defense_left.cmd_vel.linear.x = 0.25
                msg_defense_left.cmd_vel.angular.z = 0
        else:
            # Si el defensa izquierdo está en la mitad derecha del campo, seguir la pelota
            if heading_defense_left < 0.2:
                # Si la orientación es correcta, avanzar hacia la pelota
                msg_defense_left.cmd_vel.linear.x = 0.25
                msg_defense_left.cmd_vel.angular.z = 0
            else:
                # Si la orientación no es correcta, girar hacia la pelota
                msg_defense_left.cmd_vel.linear.x = 0.
                msg_defense_left.cmd_vel.angular.z = 0.5
        
        msg_defense_left.kicker = True
        msg_defense_left.dribbler = True
        
        # Atacante 1
        attacker1_x = attacker1.x
        attacker1_y = attacker1.y
        goal_angle_attacker1 = math.atan2(ball_y - attacker1_y, ball_x - attacker1_x)
        heading_attacker1 = abs(goal_angle_attacker1 - attacker1.orientation)
        distance_attacker1 = math.sqrt((ball_x - attacker1_x)**2 + (ball_y - attacker1_y)**2)
        msg_attacker1 = SSL()
        # Lógica de movimiento y acciones para el atacante 1
        if heading_attacker1 < 0.2:
            # Si la orientación es correcta, avanzar hacia la pelota
            msg_attacker1.cmd_vel.linear.x = 0.25
            msg_attacker1.cmd_vel.angular.z = 0
        else:
            # Si la orientación no es correcta, girar hacia la pelota
            msg_attacker1.cmd_vel.linear.x = 0.
            msg_attacker1.cmd_vel.angular.z = 0.5
        
        msg_attacker1.kicker = True
        msg_attacker1.dribbler = True
        
        # Atacante 2
        attacker2_x = attacker2.x
        attacker2_y = attacker2.y
        goal_angle_attacker2 = math.atan2(ball_y - attacker2_y, ball_x - attacker2_x)
        heading_attacker2 = abs(goal_angle_attacker2 - attacker2.orientation)
        distance_attacker2 = math.sqrt((ball_x - attacker2_x)**2 + (ball_y - attacker2_y)**2)
        msg_attacker2 = SSL()
        # Lógica de movimiento y acciones para el atacante 2
        if heading_attacker2 < 0.2:
            # Si la orientación es correcta, avanzar hacia la pelota
            msg_attacker2.cmd_vel.linear.x = 0.25
            msg_attacker2.cmd_vel.angular.z = 0
        else:
            # Si la orientación no es correcta, girar hacia la pelota
            msg_attacker2.cmd_vel.linear.x = 0.
            msg_attacker2.cmd_vel.angular.z = 0.5
        
        msg_attacker2.kicker = True
        msg_attacker2.dribbler = True
        """

        #print(distance_ball, goalkeeper_y)
        # Publicar los comandos para cada jugador
        pub_goalkeeper.publish(msg_goalkeeper)
        #pub_defense_right.publish(msg_defense_right)
        #pub_defense_left.publish(msg_defense_left)
        #pub_attacker1.publish(msg_attacker1)
        #pub_attacker2.publish(msg_attacker2)
        
        r.sleep()
