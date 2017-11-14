#!/usr/bin/env python
''' test the connection to the thrift server with a client calling ping
    the TBinaryProtocol must be used
'''
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements. See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership. The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License. You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied. See the License for the
# specific language governing permissions and limitations
# under the License.
#
import sys
import os
sys.path.append('../gen-py')
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/lib")

from ExtIf import ExtService
from ExtIf.ttypes import *

from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol


class TFacnetClient():
    """ localhost client to the facenet server
        will fail if server is restarted
     """
    def __init__(self, port = 1833):
        print("facenet client localhost port {}".format( port))
        # Make socket
        self.transport = TSocket.TSocket('localhost', port)

        # Buffering is critical. Raw sockets are very slow
        self.transport = TTransport.TBufferedTransport(self.transport)

        # Wrap in a protocol
        self.protocol = TBinaryProtocol.TBinaryProtocol(self.transport)

        # Create a client to use the protocol encoder
        self.client = ExtService.Client(self.protocol)

        # Connect!
        self.transport.open()

    def __del__(self):
        self.transport.close()

    def ping(self, msg):
        return self.client.ping(msg)

    def get_available_names(self):
        return self.client.GeAvailableNames()

    def send_add_person(self, img, height, width, str_name):
        param = TImageParams()
        param.hight = height
        param.width = width
        param.name = str_name
        data = TDataRaw()
        data.raw_data = img
        self.client.AddPerson(data, param)

    def send_drive_to_person(self, str_name):
        self.client.DriveToPerson(str_name)

    def send_start_driving_cmd(self):
        return self.client.StartDriving()
