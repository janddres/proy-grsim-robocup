#! /usr/bin/env python3
import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionBall  # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador

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
    dis_cerca=10000

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
                # distancia_minima=16000000 
                # #jugador_cercano=golero
                # for jugador in jugadores_equipo:
                #     if jugador is not golero:
                #         distancia = math.sqrt((jugador.get_ubicacion()['x'] - golero_x)**2 + (jugador.get_ubicacion()['y'] - golero_y)**2)
                #         if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= golero.get_ubicacion()['x']:
                #             distancia_minima = distancia
                #             jugador_cercano = jugador

               
                jugador_cercano = golero.jugador_cercano(jugadores_equipo,golero.posicion)
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
        distance_ball_cua= ((ball_x - defensaIzquierda_x)**2 + (ball_y - defensaIzquierda_y)  **2)
        
        if  distance_ball_cua> 250000 :

            #distance_pos = (pos_x - defensaIzquierda_x)**2 +( pos_y - defensaIzquierda_y)**2
            distance_pos=defensaIzquierda.set_posicion_distan()
            if distance_pos< dis_cerca:

                defensaIzquierda_msg=defensaIzquierda.mirar_frente(defensaIzquierda_msg)

            else:
          
                defensaIzquierda_msg, orient = defensaIzquierda.ir_a_posicion(distance_pos,defensaIzquierda_msg)
              
                defensaIzquierda.set_orientacion(orient)
        else:

            # Si la pelota esta cerca del defensa
            if distance_ball_cua< 12100:

                defensaIzquierda_msg=defensaIzquierda.agarra_pelota(defensaIzquierda_msg)
            
 ####################################               
                # programar pase
                #jugadores_equipo = [ atacante1, atacante2]
                # distancia_minima=16000000 
                # jugador_cercano =    defensaIzquierda
                # for jugador in jugadores_equipo:
                #     if jugador is not defensaIzquierda:
                #         distancia = math.sqrt((jugador.get_ubicacion()['x'] - defensaIzquierda_x)**2 + (jugador.get_ubicacion()['y'] - defensaIzquierda_y)**2)
                #         if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= defensaIzquierda.get_ubicacion()['x']:
                #             distancia_minima = distancia
                #             jugador_cercano = jugador
                jugador_cercano = defensaIzquierda.jugador_cercano(jugadores_equipo,defensaIzquierda.posicion)
                 
                #calculamos la direccion   
                goal_angle = math.atan2(jugador_cercano.get_ubicacion()['y']- defensaIzquierda_y , jugador_cercano.get_ubicacion()['x']- defensaIzquierda_x)

                heading_pase= goal_angle - defensaIzquierda.get_orientacion()
            
                heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))

                defensaIzquierda_msg=defensaIzquierda.pase_a_jugador(heading_pase, defensaIzquierda_msg)

            else:

                defensaIzquierda_msg=defensaIzquierda.ir_a_pelota(ball_x,ball_y,distance_ball_cua,defensaIzquierda_msg) 

#########################################################################3 
#Defensa Derecho    

        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA   
        distance_ball_cua= ((ball_x - defensaDerecho_x)**2 + (ball_y - defensaDerecho_y)  **2)
        
        if  distance_ball_cua> 250000 :

            #distance_pos = (pos_x - defensaIzquierda_x)**2 +( pos_y - defensaIzquierda_y)**2
            distance_pos=defensaDerecho.set_posicion_distan()
            if distance_pos< dis_cerca:

                defensaDerecho_msg=defensaDerecho.mirar_frente(defensaDerecho_msg)

            else:
                # Si el golero no está en su posición objetivo, moverse hacia allá
 
                defensaDerecho_msg, orient = defensaDerecho.ir_a_posicion(distance_pos,defensaDerecho_msg)
 
                #defensaDerecho.set_orientacion(orient)

        else:

            # Si la pelota esta cerca del defensa
            if distance_ball_cua< 12100:
                
                defensaDerecho_msg=defensaDerecho.agarra_pelota(defensaDerecho_msg)
        
 ####################################               
                # programar pase
                #jugadores_equipo = [ atacante1, atacante2]
                # distancia_minima=16000000 
                # jugador_cercano =    defensaDerecho
                # for jugador in jugadores_equipo:
                #     if jugador is not defensaDerecho:
                #         distancia = math.sqrt((jugador.get_ubicacion()['x'] - defensaDerecho_x)**2 + (jugador.get_ubicacion()['y'] - defensaDerecho_y)**2)
                #         if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= defensaDerecho.get_ubicacion()['x']:
                #             distancia_minima = distancia
                #             jugador_cercano = jugador
                jugador_cercano = defensaDerecho.jugador_cercano(jugadores_equipo,defensaDerecho.posicion)
                 
                 #calculamos la direccion   
                goal_angle = math.atan2(jugador_cercano.get_ubicacion()['y']- defensaDerecho_y , jugador_cercano.get_ubicacion()['x']- defensaDerecho_x)
        
                heading_pase= goal_angle - defensaDerecho.get_orientacion()

            
                heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))

                defensaDerecho_msg=defensaDerecho.pase_a_jugador(heading_pase, defensaDerecho_msg)

            else:

                defensaDerecho_msg=defensaDerecho.ir_a_pelota(ball_x,ball_y,distance_ball_cua,defensaDerecho_msg) 



#########################################################################3
#Atacante1   

        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA   
        distance_ball_cua= ((ball_x - atacante1_x)**2 + (ball_y - atacante1_y)  **2)
        
        if  distance_ball_cua> 250000 :

            #distance_pos = (pos_x - defensaIzquierda_x)**2 +( pos_y - defensaIzquierda_y)**2
            distance_pos=atacante1.set_posicion_distan()
            if distance_pos< dis_cerca:

                atacante1_msg=atacante1.mirar_frente(atacante1_msg)

            else:
                # Si el golero no está en su posición objetivo, moverse hacia allá
 
                atacante1_msg, orient = atacante1.ir_a_posicion(distance_pos,atacante1_msg)
              
                atacante1.set_orientacion(orient)

        else:

            # Si la pelota esta cerca del defensa
            if distance_ball_cua< 12100:
                
                atacante1_msg=atacante1.agarra_pelota(atacante1_msg)    
        
 ####################################               
                # Patea al arco

                #calculamos la direccion   al arco
                # goal_angle = math.atan2(400- atacante1_y , 2000- atacante1_x)
        
                # heading_pase= goal_angle - atacante1.get_orientacion()
            
                # heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))

                # atacante1_msg=atacante1.pase_a_jugador(heading_pase, atacante1_msg)

                atacante1_msg = atacante1.pateo_al_arco(atacante1_msg)

            else:

                atacante1_msg=atacante1.ir_a_pelota(ball_x,ball_y,distance_ball_cua,atacante1_msg)


#########################################################################3
#Atacante2  

        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA   
        distance_ball_cua= ((ball_x - atacante2_x)**2 + (ball_y - atacante2_y)  **2)
        
        if  distance_ball_cua> 250000 :

            #distance_pos = (pos_x - defensaIzquierda_x)**2 +( pos_y - defensaIzquierda_y)**2
            distance_pos=atacante2.set_posicion_distan()
            if distance_pos< dis_cerca:

                atacante2_msg=atacante2.mirar_frente(atacante2_msg)

            else:
                # Si el golero no está en su posición objetivo, moverse hacia allá
 
                atacante2_msg, orient = atacante2.ir_a_posicion(distance_pos,atacante2_msg)
              
                atacante2.set_orientacion(orient)

        else:

            # Si la pelota esta cerca del defensa
            if distance_ball_cua< 12100:
                
                atacante2_msg=atacante2.agarra_pelota(atacante2_msg)    
        
 ####################################               
                # Patea al arco

                #calculamos la direccion   al arco
                # goal_angle = math.atan2(400- atacante2_y , 2000- atacante2_x)
        
                # heading_pase= goal_angle - atacante2.get_orientacion()
            
                # heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))

                # atacante2_msg=atacante2.pase_a_jugador(heading_pase, atacante2_msg)
                atacante2_msg = atacante2.pateo_al_arco(atacante2_msg)

            else:

                atacante2_msg=atacante2.ir_a_pelota(ball_x,ball_y,distance_ball_cua,atacante2_msg)                 

 ##########################################################  
        golero.publisher.publish(golero_msg)  # Publicamos el mensaje de comandos para el robot 0 azul
        defensaIzquierda.publisher.publish(defensaIzquierda_msg) #Publicamos el m
        defensaDerecho.publisher.publish(defensaDerecho_msg) #mensaje de comandos para
        atacante1.publisher.publish(atacante1_msg) #jugador 1 aportado a la batalha
        atacante2.publisher.publish(atacante2_msg) #jugador 2 aportado a la batalha

        r.sleep()
