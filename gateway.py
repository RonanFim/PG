
from is_wire.core import Channel, Message, Logger
from is_msgs.robot_pb2 import RobotConfig
from google.protobuf.empty_pb2 import Empty
from driver import ATSPdriver
from time import time
import options_pb2


class ATSPgateway:

    def __init__(self, channel, driver, robot_par):
        self.channel = channel
        self.driver = driver
        self.param = robot_par
        now = time()    # time in seconds
        #self.last_speed_command = now
        self.last_sampling = now
        self.sonar_topic = "RobotGateway.{}.SonarScan"
        self.pose_topic = "RobotGateway.{}.Pose"
        self.log = Logger(name="gatewayLog")


    # Chamada pelo ServiceProvider, cria um objeto RobotConfig, preenche
    # com a velocidade do robo e retorna o objeto
    def get_configuration(self, msg, ctx):
        self.config = RobotConfig()
        #self.config.speed = self.driver.get_speed()
        #self.log.info("Pediu velocidade")
        spd = self.driver.get_speed()
        #self.log.info("Recebeu velocidade")
        self.config.speed.linear = spd.linear
        self.config.speed.angular = spd.angular
        return self.config

    # Chamada pelo ServiceProvider, envia um comando de velocidade ao robo
    def set_configuration(self, robot_conf, ctx):
        if robot_conf.HasField("speed"):
            self.driver.set_speed(robot_conf.speed)
        return Empty()

    # def enforce_safety(self):
    #     if time() > safety_deadline():
    #         # is::warn("event=EnforceSafety.HoldLimit");
    #         self.driver.stop()
    #         # quick way to avoid re-warning if no command was sent to the robot
    #         self.last_speed_command += 3600

    # Publica os dados dos sensores a cada periodo de amostragem
    def publish_sensors(self):
        if time() > self.sampling_deadline():
            # obtem valores do sensor ultrassom e publica a mensagem.
            # Quem estiver inscrito naquele topico em que a mensagem foi
            # publicada, vai recebe-la
            sonar_scan = self.driver.get_sonar_scan()
            if (sonar_scan):
                self.channel.publish(Message(content=sonar_scan), topic=self.sonar_topic.format(self.param.id))
            # obtem valores da odometria e publica a mensagem.
            # Quem estiver inscrito naquele topico em que a mensagem foi
            # publicada, vai recebe-la
            pose = self.driver.get_pose()
            if (pose):
                self.channel.publish(Message(content=pose), topic=self.pose_topic.format(self.param.id))
            # Atualiza tempo, somando periodo de amostragem
            self.last_sampling += 1/self.param.sampling_rate

    def safety_deadline(self):
        return self.last_speed_command + self.param.speed_hold_limit.seconds
    
    def sampling_deadline(self):
        return self.last_sampling + 1/self.param.sampling_rate

    def next_deadline(self):
        #return min(self.safety_deadline(), self.sampling_deadline())
        return self.sampling_deadline()

    # Chamada continuamente pelo service.py
    # Publica os dados dos sensores        
    def run(self):
        #self.enforce_safety()
        self.publish_sensors()
