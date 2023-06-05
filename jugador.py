from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim

class Jugador:
    def __init__(self, posicion):
        self.posicion = posicion #En que posicion juega
        self.ubicacion = {'x':0,'y':0} #lista donde se guardan los parametros de laubicación del jugador
        #self.ubicacion_y = 0
        self.orientacion = 0 #Angulo de orientacion
        self.publisher = None

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
        