from is_wire.core import Channel, Message, Logger
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.robot_pb2 import RobotConfig
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import Parse
from time import time
import options_pb2
import sys
import socket

from driver import ATSPdriver
from gateway import ATSPgateway

# Logger para imprimir mensagens
ATSPlog = Logger(name="ATSPlog")

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

service_name = "RobotGateway.{}".format(op_config.robot_parameters.id)

# instancia driver passando a porta serial
driver = ATSPdriver(op_config.robot_parameters.robot_uri)
ATSPlog.info("event=RobotInitDone")

# instania channel passando endereco do broker
# estabelece conexao com o broker
channel = Channel(op_config.broker_uri)
ATSPlog.info("event=ChannelInitDone")

# instancia gateway passando o channel e o driver criados, alem dos parametros do robo
gateway = ATSPgateway(channel=channel, driver=driver, robot_par=op_config.robot_parameters)

# Cria o ServiceProvider passando o channel
server = ServiceProvider(channel)

# logs = LogInterceptor()  # Log requests to console
# server.add_interceptor(logs)

# Linka cada tipo de mensagem recebida a um metodo do gateway
server.delegate(topic=service_name + ".GetConfig", request_type=Empty, reply_type=RobotConfig, function=gateway.get_configuration)
server.delegate(topic=service_name + ".SetConfig", request_type=RobotConfig, reply_type=Empty, function=gateway.set_configuration)

ATSPlog.info("event=InitAllDone")


while(1):
    try:
        # Espera receber uma mensagem no canal dentro de um timeout
        message = channel.consume(timeout=max(gateway.next_deadline()-time(),0))
        # Verifica se essa mensagem esta dentro dos delegates
        if server.should_serve(message):
            server.serve(message) # Executa o pedido da mensagem
    except socket.timeout:
        pass

    # Publica os sensores
    gateway.run()


# Exemplo de uso do Logger:
# >>> from is_wire.core import Logger
# >>> log = Logger(name="Ronan")
# >>> log.info("vel={:.2f}", 32.0320909)