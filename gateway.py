
from is_wire.core import Channel, Message, Logger
from is_msgs.robot_pb2 import RobotConfig
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

    def __del__(self):
        pass

    def get_configuration(self):
        self.config = RobotConfig()
        self.config.speed = self.driver.get_speed()
        return self.config

    def set_configuration(self, robot_conf):
        if robot_conf.HasField("speed"):
            self.driver.set_speed(robot_conf.speed)
        return Empty()

    # def enforce_safety(self):
    #     if time() > safety_deadline():
    #         # is::warn("event=EnforceSafety.HoldLimit");
    #         self.driver.stop()
    #         # quick way to avoid re-warning if no command was sent to the robot
    #         self.last_speed_command += 3600

    def publish_sensors(self):
        if time() > self.sampling_deadline():
            sonar_scan = self.driver.get_sonar_scan()
            if (sonar_scan):
                self.channel.publish(Message(content=sonar_scan), topic=self.sonar_topic.format(self.param.id))
            pose = self.driver.get_pose()
            if (pose):
                self.channel.publish(Message(content=pose), topic=self.pose_topic.format(self.param.id))
            self.last_sampling += 1/self.param.sampling_rate

    def safety_deadline(self):
        return self.last_speed_command + self.param.speed_hold_limit.seconds
    
    def sampling_deadline(self):
        return self.last_sampling + 1/self.param.sampling_rate

    def next_deadline(self):
        #return min(self.safety_deadline(), self.sampling_deadline())
        return self.sampling_deadline()
        
    def run(self):
        #self.enforce_safety()
        self.publish_sensors()



# >>> from is_wire.core import Logger
# >>> log = Logger(name="Ronan")
# >>> log.info("vel={:.2f}", 32.0320909)