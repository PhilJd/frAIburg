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
import argparse
import glob
import sys
import os
sys.path.append('gen-py')
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/lib")

import thrift
from ExtIf import ExtService
from ExtIf import ttypes
from ExtIf.ttypes import *

from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from thrift.server import TServer

class TestFacnetClient():
    """localhost client """
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

    def ping(self):
        print('thrif facenet client calls ping wiht response: {}'\
            .format(self.client.ping("test test")))



def main():
    import argparse

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--port', type=int, default=1833,
                        help='thrift TSocket client: select localhost port')

    args = parser.parse_args()
    print("facenet client localhost port {}".format( args.port))
    client = TestFacnetClient(args.port)
    client.ping()


if __name__ == '__main__':
    main()
