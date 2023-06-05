from grsim_ros_bridge_msgs.msg import SSL  # Importamos el mensaje SSL para la comunicación con grSim

class Pelota:
    def __init__(self):
        self.confience = 0#confianza
        self.area = 0
        self.ubicacion_pelota = {'x':0.0,'y':0.0} #lista donde se guardan los parametros de laubicación de la pelota


    def set_ubicacion(self,x,y):
        self.ubicacion_pelota['x'] = x
        self.ubicacion_pelota['y'] = y

    def get_ubicacion(self):
        return self.ubicacion_pelota    

    def set_confience(self,valor):
        self.confience = valor

    def get_confience(self):
        return self.confience        