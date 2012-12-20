# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2008, Willow Garage, Inc.
# Revision $Id$

import roslib
roslib.load_manifest('industrial_io_client')
import rospy
import yaml
# TODO:Factor out custom serialization over socket as one of multiple
# possible back-ends.
import socket
import struct


class Client:
    class DeviceMessage:
        def __init__(self):
            self.network_data_table = 0
            self.device_offset = 0
            self.bit_byte_offset = 0
            self.value = 0

    class Input:
        def __init__(self):
            self.name = None
            self.type = None
            self.network_data_table_code = None
            self.device_offset_code = None
            self.bit_byte_offset_code = None
            self.value_code = None
        def __repr__(self):
            return self.__str__()
        def __str__(self):
            return '%s: %s %s %s %s %s'%(self.name, self.type, self.network_data_table_code, self.device_offset_code, self.bit_byte_offset_code, self.value_code)

    class Output:
        def __init__(self):
            pass

    class SocketClient:
        def __init__(self):
            self.host = None
            self.port = None
            self.socket = None

    def __init__(self, config_file):
        self.inputs = {}
        self.outputs = {}
        self.device = None
        self.subscribers = []
        self.publishers = []
        self.load_config(config_file)

    def load_config(self, config_file):
        try:
            data = yaml.load(open(config_file))
        except yaml.YAMLError as e:
            print('YAML parse error')
            raise
        if 'inputs' in data:
            for i in data['inputs']:
                inp = self.Input()
                if type(i) != dict or len(i) != 1:
                    raise Exception('parse error')
                for k,v in i.iteritems():
                    inp.name = rospy.resolve_name(k)
                    if type(v) != dict or \
                       not v.has_key('type') or \
                       not v.has_key('transformation'):
                        raise Exception('parse error')
                    inp.type = v['type']
                    tx = v['transformation']
                    if type(tx) != dict or \
                       not tx.has_key('network_data_table') or \
                       not tx.has_key('device_offset') or \
                       not tx.has_key('bit_byte_offset') or \
                       not tx.has_key('value'):
                        raise Exception('parse error')
                    inp.network_data_table_code = tx['network_data_table']
                    inp.device_offset_code = tx['device_offset']
                    inp.bit_byte_offset_code = tx['bit_byte_offset']
                    inp.value_code = tx['value']
                    self.inputs[inp.name] = inp
        else:
            print('No inputs found in configuration; not subscribing to anything')

        if 'outputs' in data:
            raise Exception('output support not implemented')
        else:
            print('No outputs found in configuration; not publishing anything')

        if 'device' not in data:
            raise Exception('no device specified')
        dev = data['device']
        if type(dev) != dict or not dev.has_key('type'):
            raise Exception('parse error on device')
        typ = dev['type']
        if typ == 'generic_io_socket_server':
            if not dev.has_key('host') or not dev.has_key('port'):
                raise Exception('parse error on device')
            sc = self.SocketClient()
            sc.host = str(dev['host'])
            sc.port = int(dev['port'])
            self.device = sc
        else:
            raise Exception('unsupported device type')

    def handle_ros_message(self, data):
        print 'Received message on topic %s (type %s)'%(data._connection_header['topic'], data._connection_header['type'])
        if data._connection_header['topic'] not in self.inputs:
            print self.inputs
            raise Exception('received message on unexpected topic')
        inp = self.inputs[data._connection_header['topic']]
        # Set up the context that's expected by user's transformation
        #   m: the incoming messsage
        m = data
        # Evaluate the user's transformation for each field and force them to integers.
        dev_msg = self.DeviceMessage()
        dev_msg.network_data_table = int(eval(str(inp.network_data_table_code)))
        dev_msg.device_offset = int(eval(str(inp.device_offset_code)))
        dev_msg.bit_byte_offset = int(eval(str(inp.bit_byte_offset_code)))
        dev_msg.value = int(eval(str(inp.value_code)))

        self.write_to_device(dev_msg)

    def initialize_device(self):
        self.device.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print 'Connecting to server at %s:%d...'%(self.device.host, self.device.port)
        try:
            self.device.socket.connect((self.device.host, self.device.port))
        except socket.error as e: 
            print 'Error connecting to server'
            raise
        print 'Connected.'

    def write_to_device(self, dev_msg):
        # Pack the data in like so (from simple_message/simple_message.h):
        #
        # <PREFIX> Not considered part of the message
        # int LENGTH (HEADER + DATA) in bytes
        #
        # <HEADER>
        # int MSG_TYPE identifies type of message (standard and robot specific values)
        # int COMM_TYPE identified communications type
        # int REPLY CODE (service reply only) reply code
        # <BODY>
        # ByteArray DATA variable length data determined by message
        #                    type and and communications type.

        # For our message, we'll have prefix + header + body of 4 integers
        length = 1*4+3*4+4*4;
        fmt_str = '<IIIIIIII'
        # TODO: define msg_type_io in simple_message.h
        msg_type_io = 20
        comm_type_topic = 1
        reply_type_invalid = 0
        buffer = struct.pack(fmt_str,
                             length,
                             msg_type_io,
                             comm_type_topic,
                             reply_type_invalid,
                             dev_msg.network_data_table,
                             dev_msg.device_offset,
                             dev_msg.bit_byte_offset,
                             dev_msg.value)
        self.device.socket.send(buffer)

    def read_from_device(self):
        pass

    def initialize_ros(self):
        for inp in self.inputs.values():
            type_split = inp.type.split('/')
            if len(type_split) != 2:
                raise Exception('invalid type')
            module = type_split[0]
            roslib.load_manifest(module)
            module += '.msg'
            message = type_split[1]
            mod = __import__(module, globals(), locals(), [message])
            print("Subscribing to %s (%s)"%(inp.name, inp.type))
            self.subscribers.append(rospy.Subscriber(inp.name, getattr(mod, message), self.handle_ros_message))
