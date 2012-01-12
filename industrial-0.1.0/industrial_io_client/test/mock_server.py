#!/usr/bin/env python
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

import socket
import struct
import yaml
import sys

def go(config_file):
    data = yaml.load(open(config_file))
    if 'device' not in data:
        raise Exception('no device specified')
    dev = data['device']
    if type(dev) != dict or not dev.has_key('type'):
        raise Exception('parse error on device')
    typ = dev['type']
    if typ == 'generic_io_socket_server':
        if not dev.has_key('host') or not dev.has_key('port'):
            raise Exception('parse error on device')
        host = dev['host']
        port = dev['port']
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((host, port))
        sock.listen(1)
        while True:
            print 'Listening for connection'
            conn, addr = sock.accept()
            print 'Accepted new connection'
            while True:
                length = 1*4+3*4+4*4;
                fmt_str = '<IIIIIIII'
                data = ''
                while len(data) < length:
                    new_data = conn.recv(length-len(data))
                    if not new_data:
                        break
                    data += new_data
                if len(data) != length:
                    break
                values = struct.unpack(fmt_str, data)
                print 'Received:'
                print values
    else:
        raise Exception('unsupported device type')


USAGE = 'Usage: mock_server.py <config.yaml>'

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(USAGE)
        sys.exit(1)
    go(sys.argv[1])

