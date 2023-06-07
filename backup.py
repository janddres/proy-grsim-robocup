import rospy  # Importamos el módulo rospy para interactuar con ROS
from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionBall  # Importamos los mensajes relacionados con la detección de robots y balón
import math  # Importamos el módulo math para realizar operaciones matemáticas
from jugador import Jugador
from pelota import Pelota

ball = SSL_DetectionBall()  # Creamos una instancia del mensaje SSL_DetectionBall
golero = Jugador('golero')
defensa1= Jugador('defensa1')
defensa2 = Jugador('defensa2')
atacante1 = Jugador('atacante1')
atacante2 = Jugador('atacante2')

def vision_callback(data):
    global ball, golero, defensa1, defensa2, atacante1, atacante2
    ball = data.balls  # Actualizamos la información del balón
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

if __name__ == "__main__":
    rospy.init_node("grsim_pria", anonymous=False)  # Inicializamos el nodo ROS con el nombre "grsim_pria"
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)  # Nos suscribimos al tópico "/vision" para recibir los marcos de detección
    golero.publisher= rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)  # Creamos un publicador para enviar comandos al robot 0 azul
    r = rospy.Rate(1000)  # Establecemos la frecuencia de publicación en 1000 Hz

    jugadores_equipo = [golero, defensa1, defensa2, atacante1, atacante2]

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
            golero_y = golero.get_ubicacion()['y']  # golero
            defensa1_x = defensa1.get_ubicacion()['x'] # Obtenemos la posición x del
            defensa1_y = defensa1.get_ubicacion()['y'] # defensa1
            defensa2_x = defensa2.get_ubicacion()['x'] # Obtenemos la posición x del
            defensa2_y = defensa2.get_ubicacion()['y'] # defensa2
            atacante1_x = atacante1.get_ubicacion()['x'] # Obtenemos la posición x del
            atacante1_y = atacante1.get_ubicacion()['y'] # atacante1
            atacante2_x = atacante2.get_ubicacion()['x'] # Obtenemos la posición x del
            atacante2_y = atacante2.get_ubicacion()['y'] # atacante2            
        except:
            pass

        ######################################################################################
#Golero

        golero_msg.kicker = 0
        golero_msg.dribbler =False
        #Calculamos la posicion de la pelota respeto al jugador
        # Primero la distancia de la pelota al jugador, ESTIMAMOS El CUADRADO DE LA DISTANCIA  
        distance_ball_cua= ((ball_x - golero_x)**2 + (ball_y - golero_y )**2)
       
        if ball_x > -800 and distance_ball_cua> 250000:
            # Si la pelota esta lejos del golero y del arco el golero va a su posicion inicial
            # pos_x = -1500
            # pos_y = 0
            # posfrente_x=2000
            # posfrente_y=0
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
            distance_ball_cua= ((ball_x - golero_x)**2 + (ball_y - golero_y )**2)
            if distance_ball_cua<12100:
                golero_msg = golero.agarra_pelota(golero_msg) #retiene la pelota

                
#######################################
#Jugador cercano
                # jugadores_equipo = [defensa1, defensa2, atacante1, atacante2]
                distancia_minima=16000000 
                jugador_cercano=golero
                for jugador in jugadores_equipo:
                    if jugador is not golero:
                        distancia = math.sqrt((jugador.get_ubicacion()['x'] - golero_x)**2 + (jugador.get_ubicacion()['y'] - golero_y)**2)
                        if distancia < distancia_minima and jugador.get_ubicacion()['x'] >= golero.get_ubicacion()['x']:
                            distancia_minima = distancia
                            jugador_cercano = jugador

                print("El jugador más cercano al golero con x  nayor es:", jugador_cercano.get_ubicacion()['x'] )
                 
                 #calculamos la direccion   
                goal_angle = math.atan2(jugador_cercano.get_ubicacion()['y']- golero_y , jugador_cercano.get_ubicacion()['x']- golero_x)
                heading_pase= goal_angle - golero.get_orientacion()
                heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))
                print("Pase del jugador angulo :", heading_pase)

                #gira hasta mirar al jugador del pase
                golero_msg = golero.pase_a_jugador(heading_pase,golero_msg)

                # if abs(heading_pase)<0.05:
                #     print("apunta al jugador")
                #     golero_msg.cmd_vel.linear.x = 0   
                #     golero_msg.cmd_vel.angular.z = 0
                #         #patear la pelota
                #     golero_msg.kicker = 3
                #     print("le pega a la pelota")
                        
                # else:
                #     print("esta girando")
                #     golero_msg.cmd_vel.linear.x = 0 
                #     golero_msg.cmd_vel.angular.z =heading_pase+0.5 if heading_pase > 0 else heading_pase-0.5
                #     print(golero_msg.cmd_vel.angular.z )

                #golero.publisher.publish(golero_msg)
                #######################################
                

            else:

                golero_msg = golero.ir_a_pelota(ball_x,ball_y,distance_ball_cua,golero_msg)  
                # goal_angle = math.atan2(ball_y - golero_y , ball_x - golero_x)
                # heading_ball = goal_angle - golero.get_orientacion()
                # heading_ball = math.atan2(math.sin(heading_ball), math.cos(heading_ball))
       
                # if abs(heading_ball) < 0.2:
                #     # Si la orientación es correcta, avanzar hacia la pelota
                #     golero_msg.cmd_vel.linear.x = (distance_ball_cua/4000000)+ 0.5#calculo de velocidad segun la distancia
                #     golero_msg.cmd_vel.angular.z = 0
                # else:

                #     # Si la orientación no es correcta, girar
                #     golero_msg.cmd_vel.linear.x = 0
                #     golero_msg.cmd_vel.angular.z = heading_ball*5+0.5 if heading_ball > 0 else heading_ball*5-0.5#control para que gire mas optimo


        golero.publisher.publish(golero_msg)  # Publicamos el mensaje de comandos para el robot 0 azul

        r.sleep()