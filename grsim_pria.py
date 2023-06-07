#! /usr/bin/env python3
import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionBall  # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador
#from pelota import Pelota

ball = SSL_DetectionBall()  # Creamos una instancia del mensaje SSL_DetectionBall
golero = Jugador('golero')
defensaIzquierda= Jugador('defensaIzquierda')
defensaDerecho = Jugador('defensaDerecho')
atacante1 = Jugador('atacante1')
atacante2 = Jugador('atacante2')

def vision_callback(data):
    global ball, golero, defensaIzquierda, defensaDerecho, atacante1, atacante2
    ball = data.balls  # Actualizamos la información del balón
    # Recorremos la lista de robots azules detectados en el marco de detección
    if len(data.robots_blue) > 0:
        for robot in data.robots_blue:
            if robot.robot_id == 0:
                golero.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 0
                golero.set_orientacion(robot.orientation)
            if robot.robot_id == 1:
                defensaIzquierda.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 1
                defensaIzquierda.set_orientacion(robot.orientation)
            if robot.robot_id == 2:
                defensaDerecho.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 2
                defensaDerecho.set_orientacion(robot.orientation)
            if robot.robot_id == 3:
                atacante1.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 3
                atacante1.set_orientacion(robot.orientation)
            if robot.robot_id == 4:
                atacante2.set_ubicacion(robot.pixel_x, robot.pixel_y)  # Actualizamos la información del robot 4
                atacante2.set_orientacion(robot.orientation)    

if __name__ == "__main__":
    rospy.init_node("grsim_pria", anonymous=False)  # Inicializamos el nodo ROS con el nombre "grsim_pria"
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)  # Nos suscribimos al tópico "/vision" para recibir los marcos de detección
    golero.publisher= rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)  # Creamos un publicador para enviar comandos al robot 0 azul
    defensaIzquierda.publisher= rospy.Publisher("/robot_blue_1/cmd", SSL, queue_size=10)
    defensaDerecho.publisher= rospy.Publisher("/robot_blue_2/cmd", SSL, queue_size=10)
    atacante1.publisher= rospy.Publisher("/robot_blue_3/cmd", SSL, queue_size=10)
    atacante2.publisher= rospy.Publisher("/robot_blue_4/cmd", SSL, queue_size=10)
    

    r = rospy.Rate(1000)  # Establecemos la frecuencia de publicación en 1000 Hz

    jugadores_equipo = [golero, defensaIzquierda, defensaDerecho, atacante1, atacante2]

    golero_x = 0  # Inicializamos la posición x del robot 0 en 0
    golero_y = 0  # Inicializamos la posición y del robot 0 en 0
    defensaIzquierda_x = 0  # Inicializamos la posición x del robot 1 en 0
    defensaIzquierda_y = 0  # Inicializamos la posición y del robot 1 en 0
    defensaDerecho_x = 0  # Inicializamos la posición x del robot 2 en 0
    defensaDerecho_y = 0  # Inicializamos la posición y del robot 2 en 0
    atacante1_x = 0  # Inicializamos la posición x del robot 3 en 0
    atacante1_y = 0  # Inicializamos la posición y del robot 3 en 0
    atacante2_x = 0  # Inicializamos la posición x del robot 4 en 0
    atacante2_y = 0  # Inicializamos la posición y del robot 4 en 0
    ball_x = 0  # Inicializamos la posición x del balón en 0
    ball_y = 0  # Inicializamos la posición y del balón en 0

    golero_msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al robot
    defensaIzquierda_msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al
    defensaDerecho_msg = SSL()  # robot 2
    atacante1_msg = SSL()  # Creamos una instancia del mensaje SSL para enviar comandos al
    atacante2_msg = SSL()  # robot 4
    

    while not rospy.is_shutdown():
        try:
            ball_x = ball[0].x  # Obtenemos la posición x del balón #pelota.get_ubicacion()['x']
            ball_y = ball[0].y  # Obtenemos la posición y del balón #pelota.get_ubicacion()['y']
            golero_x = golero.get_ubicacion()['x'] # Obtenemos la posición x del robot 0
            golero_y = golero.get_ubicacion()['y']  # golero
            defensaIzquierda_x = defensaIzquierda.get_ubicacion()['x'] # Obtenemos la posición x del
            defensaIzquierda_y = defensaIzquierda.get_ubicacion()['y'] # defensaIzquierda
            defensaDerecho_x = defensaDerecho.get_ubicacion()['x'] # Obtenemos la posición x del
            defensaDerecho_y = defensaDerecho.get_ubicacion()['y'] # defensaDerecho
            atacante1_x = atacante1.get_ubicacion()['x'] # Obtenemos la posición x del
            atacante1_y = atacante1.get_ubicacion()['y'] # atacante1
            atacante2_x = atacante2.get_ubicacion()['x'] # Obtenemos la posición x del
            atacante2_y = atacante2.get_ubicacion()['y'] # atacante2            
        except:
            pass

        golero_msg.kicker = 0
        golero_msg.dribbler =False
        defensaIzquierda_msg.kicker = 0
        defensaIzquierda_msg.dribbler = False
        defensaDerecho_msg.kicker = 0
        defensaDerecho_msg.dribbler = False
        atacante1_msg.kicker = 0
        atacante1_msg.dribbler = False
        atacante2_msg.kicker = 0
        atacante2_msg.dribbler = False

######################################################################################
#Golero


            
        
        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA  
        distance_ball_cua= ((ball_x - golero_x)**2 + (ball_y - golero_y )**2)
       
        if ball_x > -800 and distance_ball_cua> 250000:
            # Si la pelota esta lejos del golero y del arco el golero va a su posicion inicial

            dis_cerca=10000

            distance_pos = golero.set_posicion_distan()
           
            if distance_pos< dis_cerca:
                golero_msg = golero.mirar_frente(golero_msg) #se posiciona al frente
                 
            else:
                golero_msg, orient = golero.ir_a_posicion(distance_pos,golero_msg) #vuelve a la pasicion
                golero.set_orientacion(orient)

        else:
            print("entro al else")
            # Si la pelota esta cerca del golero o se acerca al arco seguir la pelota
            #distance_ball_cua= ((ball_x - golero_x)**2 + (ball_y - golero_y )**2)
            if distance_ball_cua<12100:
                golero_msg = golero.agarra_pelota(golero_msg) #retiene la pelota

                
#######################################
#Jugador cercano
                # jugadores_equipo = [defensaIzquierda, defensaDerecho, atacante1, atacante2]
                distancia_minima=16000000 
                #jugador_cercano=golero
                for jugador in jugadores_equipo:
                    if jugador is not golero:
                        distancia = math.sqrt((jugador.get_ubicacion()['x'] - golero_x)**2 + (jugador.get_ubicacion()['y'] - golero_y)**2)
                        if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= golero.get_ubicacion()['x']:
                            distancia_minima = distancia
                            jugador_cercano = jugador

               
                 
                 #calculamos la direccion   
                goal_angle = math.atan2(jugador_cercano.get_ubicacion()['y']- golero_y , jugador_cercano.get_ubicacion()['x']- golero_x)
                heading_pase= goal_angle - golero.get_orientacion()
                heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))
     

                #gira hasta mirar al jugador del pase
                golero_msg = golero.pase_a_jugador(heading_pase,golero_msg)
                

            else:

                golero_msg = golero.ir_a_pelota(ball_x,ball_y,distance_ball_cua,golero_msg)  

 #########################################################################3 

#Defensa Izquierdo      


        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA   
        distance_ball_cua2= ((ball_x - defensaIzquierda_x)**2 + (ball_y - defensaIzquierda_y)  **2)
        
        
        if  distance_ball_cua2> 250000 :
            #ball_x > 0 anddistance_ball_cua> 250000 
            # Si la pelota esta lejos del defensa 
            pos_x = -500
            pos_y = 1500
            dis_cerca=10000
            distance_pos = (pos_x - defensaIzquierda_x)**2 +( pos_y - defensaIzquierda_y)**2
            
            if distance_pos< dis_cerca:
                # Si el jugador está cerca de su posición objetivo, detenerse
                defensaIzquierda_msg.cmd_vel.linear.x = 0
                defensaIzquierda_msg.cmd_vel.angular.z = 0
                 

            else:
                # Si el golero no está en su posición objetivo, moverse hacia allá
                goal_angle = math.atan2(pos_y- defensaIzquierda_y, pos_x- defensaIzquierda_x)
                heading_pos = goal_angle - defensaIzquierda.get_orientacion()
                heading_pos= math.atan2(math.sin(heading_pos), math.cos(heading_pos))

                if abs(heading_pos)<0.2:
                   # si la idea de la programacion es la misma, controlar con otro rango la velocidad de los jugadores
                   defensaIzquierda_msg.cmd_vel.linear.x = (distance_pos /2000000)+ 0.5#1.5
                   defensaIzquierda_msg.cmd_vel.angular.z = 0
                else:
                    defensaIzquierda_msg.cmd_vel.linear.x = 0
                    defensaIzquierda_msg.cmd_vel.angular.z = heading_pos*5+1 if heading_pos > 0 else heading_pos*5-1 #1.9 if heading_pos > 0 else -1.9

        else:

            # Si la pelota esta cerca del defensa
            if distance_ball_cua2< 12100:
                # Si esta cerca detenerse
                defensaIzquierda_msg.cmd_vel.linear.x = 0
                defensaIzquierda_msg.cmd_vel.angular.z = 0
                defensaIzquierda_msg.dribbler =True
                #print("agarro la pelota",  distance_ball_cua)              
 ####################################               
                # programar pase
                #jugadores_equipo = [ atacante1, atacante2]
                distancia_minima=16000000 
                jugador_cercano =    defensaIzquierda
                for jugador in jugadores_equipo:
                    if jugador is not defensaIzquierda:
                        distancia = math.sqrt((jugador.get_ubicacion()['x'] - defensaIzquierda_x)**2 + (jugador.get_ubicacion()['y'] - defensaIzquierda_y)**2)
                        if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= defensaIzquierda.get_ubicacion()['x']:
                            distancia_minima = distancia
                            jugador_cercano = jugador

                #print("El jugador más cercano al golero con x  nayor es:", jugador_cercano.get_ubicacion()['x'] )
                 
                 #calculamos la direccion   
                goal_angle = math.atan2(jugador_cercano.get_ubicacion()['y']- defensaIzquierda_y , jugador_cercano.get_ubicacion()['x']- defensaIzquierda_x)
                #goal_angle = math.atan2(2000- golero_y , 0- golero_x)
                heading_pase= goal_angle - defensaIzquierda.get_orientacion()
                #print("orientacion golero", golero.get_orientacion() )
            
                heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))
                #print("Pase del jugador angulo :", heading_pase)
                #gira hasta mirar al jugador del pase
                #while
                if abs(heading_pase)<0.05:
                    print("apunta al jugador")
                    defensaIzquierda_msg.cmd_vel.linear.x = 0   
                    defensaIzquierda_msg.cmd_vel.angular.z = 0
                        #patear la pelota
                    defensaIzquierda_msg.kicker = 3
                    #print("le pega a la pelota")
                        
                else:
                    print("esta girando")
                    defensaIzquierda_msg.cmd_vel.linear.x = 0 
                    defensaIzquierda_msg.cmd_vel.angular.z =heading_pase+0.5 if heading_pase > 0 else heading_pase-0.5
                    #print(golero_msg.cmd_vel.angular.z )

            else:

                goal_angle = math.atan2(ball_y - defensaIzquierda_y, ball_x - defensaIzquierda_x)
                heading_ball = goal_angle - defensaIzquierda.get_orientacion()
                heading_ball = math.atan2(math.sin(heading_ball), math.cos(heading_ball))
        
                if abs(heading_ball) < 0.2:
                    # Si la orientación es correcta, avanzar hacia la pelota
                    defensaIzquierda_msg.cmd_vel.linear.x = (distance_ball_cua2/2000000)+ 0.5#calculo de velocidad segun la distancia
                    defensaIzquierda_msg.cmd_vel.angular.z = 0
                else:
                    # Si la orientación no es correcta, girar
                    defensaIzquierda_msg.cmd_vel.linear.x = 0
                    defensaIzquierda_msg.cmd_vel.angular.z = heading_ball*5+1 if heading_ball > 0 else heading_ball*5-1 

    
                                                                                                                            
                                                                                                                          
       
 ##########################################################  
 

        golero.publisher.publish(golero_msg)  # Publicamos el mensaje de comandos para el robot 0 azul
        defensaIzquierda.publisher.publish(defensaIzquierda_msg) #Publicamos el m

        r.sleep()