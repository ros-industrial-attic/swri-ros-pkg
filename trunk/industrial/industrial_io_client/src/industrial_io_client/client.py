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
roslib.load_manifest('rospy')
import rospy
import yaml
import sys


class Client:
    class Input:
        def __init__(self):
            self.name = None
            self.type = None
            self.network_data_table = None
            self.device_offset = None
            self.bit_byte_offset = None
            self.value = None
        def __repr__(self):
            return self.__str__()
        def __str__(self):
            return '%s: %s %s %s %s %s'%(self.name, self.type, self.network_data_table, self.device_offset, self.bit_byte_offset, self.value)

    class Output:
        def __init__(self):
            pass

    def __init__(self, config_file):
        self.inputs = {}
        self.outputs = {}
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
                    inp.network_data_table = tx['network_data_table']
                    inp.device_offset = tx['device_offset']
                    inp.bit_byte_offset = tx['bit_byte_offset']
                    inp.value = tx['value']
                    self.inputs[inp.name] = inp
        else:
            print('No inputs found in configuration; not subscribing to anything')
        if 'outputs' in data:
            raise Exception('output support not implemented')
        else:
            print('No outputs found in configuration; not publishing anything')

    def cb(self, data):
        print 'Received message on topic %s (type %s)'%(data._connection_header['topic'], data._connection_header['type'])
        if data._connection_header['topic'] not in self.inputs:
            print self.inputs
            raise Exception('received message on unexpected topic')
        inp = self.inputs[data._connection_header['topic']]
        m = data
        print 'Transformed message:'
        print '  network_data_table: %s'%(eval(inp.network_data_table))
        print '  device_offset: %s'%(eval(inp.device_offset))
        print '  bit_byte_offset: %s'%(eval(inp.bit_byte_offset))
        print '  value: %s'%(eval(inp.value))

    def subscribe_advertise(self):
        for inp in self.inputs.values():
            type_split = inp.type.split('/')
            if len(type_split) != 2:
                raise Exception('invalid type')
            module = type_split[0]
            module += '.msg'
            message = type_split[1]
            mod = __import__(module, globals(), locals(), [message])
            print("Subscribing to %s (%s)"%(inp.name, inp.type))
            sub = rospy.Subscriber(inp.name, getattr(mod, message), self.cb)

USAGE = 'Usage: client.py <config.yaml>'

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(USAGE)
        sys.exit(1)
    c = Client(sys.argv[1])
    c.subscribe_advertise()

    rospy.init_node('industrial_io_client')
    rospy.spin()
