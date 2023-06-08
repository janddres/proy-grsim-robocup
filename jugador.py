from geometry_msgs.msg import Twist  # Importamos el mensaje Twist para el control de movimiento
from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim
import math  # Importamos el módulo math para realizar operaciones matemáticas
import random

class Jugador:
    def __init__(self, posicion):
        self.posicion = posicion #En que posicion juega
        self.ubicacion = {'x':0,'y':0} #diccionario donde se guardan los parametros de laubicación del jugador
        #self.ubicacion_y = 0
        self.orientacion = 0 #Angulo de orientacion
        self.publisher = None
        self.msg = SSL()
        self.ref = {'pos_x':0,'pos_y':0,'posfrente_x':0,'posfrente_y':0}

    def get_ubicacion(self):
        return self.ubicacion  

    def set_ubicacion(self,x,y):
        self.ubicacion['x'] = x
        self.ubicacion['y'] = y

    def set_orientacion(self,angulo):
        self.orientacion = angulo

    def get_orientacion(self):
        return self.orientacion 

    def set_publisher(self, valor):
        self.publisher = valor

    def get_publiser(self):
        return self.publisher
    
    def set_ref(self):
        if self.posicion == 'golero':
            self.ref = {'pos_x':-1500,'pos_y':0,'posfrente_x':2000,'posfrente_y':0}
        if self.posicion == 'defensaIzquierda':
            self.ref = {'pos_x':-500,'pos_y':1300,'posfrente_x':2000,'posfrente_y':0}  
        if self.posicion == 'defensaDerecho':
            self.ref = {'pos_x':-500,'pos_y':-1300,'posfrente_x':2000,'posfrente_y':0} 
        if self.posicion == 'atacante1':
            self.ref = {'pos_x':500,'pos_y':1000,'posfrente_x':2000,'posfrente_y':0}
        if self.posicion == 'atacante2':
            self.ref = {'pos_x':500,'pos_y':-1000,'posfrente_x':2000,'posfrente_y':0} 
        return (self.ref)
    
    def set_posicion_distan(self):
        distance_pos = (self.ref['pos_x'] - self.ubicacion['x'])**2 +( self.ref['pos_y'] - self.ubicacion['y']) **2
        return distance_pos



    def mirar_frente(self, msg):
        self.msg = msg
        refrefencia = self.set_ref()
        self.msg.cmd_vel.linear.x = 0
        goal_angle = math.atan2(refrefencia['posfrente_y'] - self.ubicacion['y'], refrefencia['posfrente_x'] - self.ubicacion['x'])
        heading_posfrente = goal_angle - self.orientacion
        #heading_posfrente = goal_angle - self.get_orientacion()
        heading_posfrente= math.atan2(math.sin(heading_posfrente), math.cos(heading_posfrente))
        #gira hasta mirar al frente
        if abs(heading_posfrente)<0.2:
            self.msg.cmd_vel.angular.z = 0
        else:
            self.msg.cmd_vel.linear.x = 0
            self.msg.cmd_vel.angular.z = heading_posfrente*5+0.5 if heading_posfrente > 0 else heading_posfrente*5-0.5 
        
        return(self.msg)
    
    
    def ir_a_posicion(self,distance_pos,msg):
        self.msg = msg
        refrefencia = self.set_ref()
        # Si el jugador no está en su posición objetivo, moverse hacia allá
        goal_angle = math.atan2(refrefencia['pos_y'] - self.ubicacion['y'], refrefencia['pos_x'] - self.ubicacion['x'])
        heading_pos = goal_angle - self.get_orientacion()
        heading_pos= math.atan2(math.sin(heading_pos), math.cos(heading_pos))

        if abs(heading_pos)<0.2:
            # si la idea de la programacion es la misma, controlar con otro rango la velocidad de los jugadores
            self.msg.cmd_vel.linear.x = (distance_pos /2000000)+ 0.5#1.5
            self.msg.cmd_vel.angular.z = 0
        else:
            self.msg.cmd_vel.linear.x = 0
            self.msg.cmd_vel.angular.z = heading_pos*5+0.5 if heading_pos > 0 else heading_pos*5-0.5

        return(self.msg, self.get_orientacion())


    def agarra_pelota(self,msg):
        self.msg = msg
        # Si esta cerca detenerse
        self.msg.cmd_vel.linear.x = 0
        self.msg.cmd_vel.angular.z = 0
        
        # programar pase
        #time.sleep(1)
        self.msg.dribbler =True

        return(self.msg)
    
    def pase_a_jugador(self,heading_pase,msg):
        self.msg = msg
        #gira hasta mirar al jugador del pase
        if abs(heading_pase)<0.05:
            self.msg.cmd_vel.linear.x = 0   
            self.msg.cmd_vel.angular.z = 0
                #patear la pelota
            self.msg.kicker = 3
        else:
            self.msg.cmd_vel.linear.x = 0 
            self.msg.cmd_vel.angular.z =heading_pase+0.5 if heading_pase > 0 else heading_pase-0.5

        return(self.msg)

    def ir_a_pelota(self,ball_x,ball_y,distance_ball_cua,msg):
        self.msg = msg
        goal_angle = math.atan2(ball_y - self.ubicacion['y'], ball_x - self.ubicacion['x'])
        heading_ball = goal_angle - self.get_orientacion()
        heading_ball = math.atan2(math.sin(heading_ball), math.cos(heading_ball))

        if abs(heading_ball) < 0.2:
            # Si la orientación es correcta, avanzar hacia la pelota
            self.msg.cmd_vel.linear.x = (distance_ball_cua/4000000)+ 0.5#calculo de velocidad segun la distancia
            self.msg.cmd_vel.angular.z = 0
        else:
            # Si la orientación no es correcta, girar
            self.msg.cmd_vel.linear.x = 0
            self.msg.cmd_vel.angular.z = heading_ball*5+0.5 if heading_ball > 0 else heading_ball*5-0.5#control para que gire mas optimo 

        return(self.msg)
    
    def jugador_cercano(self,jugadores_equipo, rol):
        distancia_minima=16000000 
        for player in jugadores_equipo:
            if player.posicion is not rol:
                distancia = math.sqrt((player.get_ubicacion()['x'] - self.ubicacion['x'])**2 + (player.get_ubicacion()['y'] - self.ubicacion['y'])**2)
                if distancia < distancia_minima and player.get_ubicacion()['x'] >= self.get_ubicacion()['x']:
                    distancia_minima = distancia
                    jugador_cercano = player
        return(jugador_cercano)

    def pateo_al_arco(self,msg):
        self.msg = msg
        disparo1 = math.atan2(400- self.ubicacion['y'] , 2000- self.ubicacion['x'])
        disparo2 = math.atan2(-400- self.ubicacion['y'] , 2000- self.ubicacion['x'])
        disparo3 = math.atan2(200- self.ubicacion['y'] , 2000- self.ubicacion['x'])
        disparo4 = math.atan2(-200- self.ubicacion['y'] , 2000- self.ubicacion['x'])
        disparos = [disparo1,disparo2,disparo3,disparo4]

        goal_angle = random.choice(disparos)

        heading_pase= goal_angle - self.get_orientacion()
        heading_pase= math.atan2(math.sin(heading_pase), math.cos(heading_pase))

        #gira hasta mirar al jugador del pase
        if abs(heading_pase)<0.05:
            self.msg.cmd_vel.linear.x = 0   
            self.msg.cmd_vel.angular.z = 0
                #patear la pelota
            self.msg.kicker = 4
        else:
            self.msg.cmd_vel.linear.x = 0 
            self.msg.cmd_vel.angular.z =heading_pase+0.5 if heading_pase > 0 else heading_pase-0.5

        return(self.msg)