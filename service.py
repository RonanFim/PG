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


ATSPlog = Logger(name="ATSPlog")

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

op_config = load_options()

service_name = "ATSP"

driver = ATSPdriver(op_config.robot_parameters.robot_uri)
ATSPlog.info("event=RobotInitDone")

channel = Channel(op_config.broker_uri)
ATSPlog.info("event=ChannelInitDone")

gateway = ATSPgateway(channel=channel, driver=driver, robot_par=op_config.robot_parameters)

server = ServiceProvider(channel)

logs = LogInterceptor()  # Log requests to console
server.add_interceptor(logs)

server.delegate(topic=service_name + ".GetConfig", request_type=Empty, reply_type=RobotConfig, function=gateway.get_configuration)
server.delegate(topic=service_name + ".SetConfig", request_type=RobotConfig, reply_type=Empty, function=gateway.set_configuration)

ATSPlog.info("event=InitAllDone")

# print(gateway.next_deadline())
# print(time())
while(1):
    try:
        #ATSPlog.info("consume 1!")
        message = channel.consume(timeout=max(gateway.next_deadline()-time(),0))
        #ATSPlog.info("Passou pelo consume!")
        if server.should_serve(message):
            server.serve(message)
        #ATSPlog.info("Dei run!")
    except socket.timeout:
        # ATSPlog.info("timeout!")
        pass
    gateway.run()
