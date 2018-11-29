
# Antes de rodar o service.py, executar os comandos na Raspberry:
# To stop getty:
# sudo systemctl stop serial-getty@ttyAMA0.service

# Problemas de permissao:
# sudo chmod a+rw /dev/ttyAMA0

from is_wire.core import Channel, Message, Subscription, Logger
from google.protobuf.json_format import Parse
from is_msgs.robot_pb2 import RangeScan, RobotConfig
from is_msgs.common_pb2 import Pose, Speed
import options_pb2
import sys
import time

# Carrega parametros do robo e endereco do broker do arquivo Json
def load_options():
    log = Logger(name='LoadOptions')
    op_file = sys.argv[1] if len(sys.argv) > 1 else 'options.json'
    try:
        with open(op_file, 'r') as f:
            try:
                op = Parse(f.read(), options_pb2.RobotGatewayOptions())
                log.info('Options: \n{}', op)
            except Exception as ex:
                log.critical(
                    'Unable to load options from \'{}\'. \n{}', op_file, ex)
            except:
                log.critical('Unable to load options from \'{}\'', op_file)
    except Exception as ex:
        log.critical('Unable to open file \'{}\'', op_file)

    message = op.DESCRIPTOR.full_name
    # validation
    if op.robot_parameters.sampling_rate < 0.1:
        message += " 'sampling_rate' field must be equal or greater than 0.1. "
        message += "Given {}".format(op.robot_parameters.sampling_rate)
        raise Exception(message)

    return op



# Carrega parametros do robo e endereco do broker do arquivo Json
op_config = load_options()

ATSPlog = Logger(name='unpack')

service_name = "RobotGateway.{}".format(op_config.robot_parameters.id)
sonar_topic = "RobotGateway.{}.SonarScan".format(op_config.robot_parameters.id)
pose_topic = "RobotGateway.{}.Pose".format(op_config.robot_parameters.id)

# instania channel passando endereco do broker
# estabelece conexao com o broker
channel = Channel(op_config.broker_uri)
ATSPlog.info("event=ChannelInitDone")

# Inscreve nos topicos que deseja receber mensagem
subscription = Subscription(channel)
subscription.subscribe(pose_topic)
subscription.subscribe(sonar_topic)
ATSPlog.info("SubscriptionsDone")


# Envia mensagem de getConfig
get_req = Message(reply_to=subscription)
# Salva correlation_id do request
cor_id = get_req.correlation_id
# print("cor_id: ", cor_id)
# Broadcast message to anyone interested (subscribed)
channel.publish(message=get_req, topic=service_name + ".GetConfig")

# Envia msg de setConfig
config = RobotConfig()
config.speed.linear = 1.5
config.speed.angular = 0.0
set_req = Message(content=config, reply_to=subscription)
# Broadcast message to anyone interested (subscribed)
channel.publish(topic=service_name + ".SetConfig", message=set_req)

print("Andou!")

time.sleep(3)

print("Parou?")

# Envia msg de setConfig
config = RobotConfig()
config.speed.linear = 0.0
config.speed.angular = 0.0
set_req = Message(content=config, reply_to=subscription)
# Broadcast message to anyone interested (subscribed)
channel.publish(topic=service_name + ".SetConfig", message=set_req)


while 1:

    # Verifica se chegou alguma mensagem nos topicos inscritos
    message = channel.consume()


    # Se for uma mensagem retornada do get_configuration...
    # Apenas mensagens vindas de RPC possuem correlation_id
    if message.has_correlation_id():
        if message.correlation_id == cor_id: # Mensagem retornada do get config
            # print("correlation: ", message.correlation_id)
            ATSPlog.info("get_config")
            robconf = message.unpack(RobotConfig) # desserializa
            # print("Vel linear: ", robconf.speed.linear)
            # print("Vel angular: ", robconf.speed.angular)

            # Envia mensagem de getConfig
            get_req = Message(reply_to=subscription)
            # Salva correlation_id do request
            cor_id = get_req.correlation_id
            # print("cor_id: ", cor_id)
            # Broadcast message to anyone interested (subscribed)
            channel.publish(message=get_req, topic=service_name + ".GetConfig")


    # Se for uma mensagem de ultrassom...
    if message.topic == sonar_topic:
        scan = message.unpack(RangeScan) # desserializa
    #     for num in range(3):
    #         print(scan.angles[num],"   ",scan.ranges[num])

    # Se for uma mensagem de odometria...
    if message.topic == pose_topic:
        ATSPlog.info("odometria")
        pos = message.unpack(Pose) # desserializa
        # print("pos x: ", pos.position.x)
        # print("pos y: ", pos.position.y)
        # print("orient: ", pos.orientation.roll)

        # Envia msg de setConfig
        config = RobotConfig()
        config.speed.linear = 0.0
        config.speed.angular = 0.0
        set_req = Message(content=config, reply_to=subscription)
        # Broadcast message to anyone interested (subscribed)
        channel.publish(topic=service_name + ".SetConfig", message=set_req)

    
    # print(" ")

    # time.sleep(0.075)
