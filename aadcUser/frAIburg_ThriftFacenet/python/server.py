#!/usr/bin/env python

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
# https://git1-us-west.apache.org/repos/asf?p=thrift.git;a=blob;f=tutorial/py/PythonServer.py;hb=HEAD
import glob
import sys
import os
import cv2
import numpy as np
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

from cStringIO import StringIO
from PIL import Image
from facenet_interface import NearestNeighborHandler

import base64
from scipy.misc import toimage
from scipy.ndimage import imread
import time

stopped = True
nn_handler = NearestNeighborHandler()

class ExtServiceHandler:

    def ping(self, sender):
        print("ping form client with msg: "+ sender)
        return "hello from facenet server"

    def raw_data_to_nparray(self, raw_data, params):
        transportOut = TTransport.TMemoryBuffer()
        protocolOut = TBinaryProtocol.TBinaryProtocol(transportOut)
        raw_data.write(protocolOut)
        data_bytes = transportOut.getvalue()
        # thrift prepends 7 bytes to the raw_data, remove them
        data_bytes = data_bytes[7:]
        data_bytes = StringIO(data_bytes)
        flat_data =  np.asarray(Image.open(data_bytes), dtype=np.uint8)
        # flat_data = np.fromstring(data_bytes, dtype=np.uint8)
        return flat_data.reshape((params.height, params.width, 3))

    def rawData(self, transport_def, raw_data, params):
        global nn_handler
        image = self.raw_data_to_nparray(raw_data, params)
        detection_result = nn_handler.get_persons(image)
        print(detection_result)
        return detection_result

    def AddPerson(self, raw_data, params):
        global nn_handler
        print("rawImageDataAndroid active!" + params.name)
        # image = self.raw_data_to_nparray(raw_data, params)
        image = self._raw_data_to_jpg(raw_data)
        with open("/tmp/a.jpg", 'wb+') as f:
            f.write(image)
        image = np.array(imread("/tmp/a.jpg"), dtype=np.uint8)
        nn_handler.add_person(image, params.name, new_image=True)
        #self._save_img_show( raw_data, params)


    def DriveToPerson(self, name):
        global stopped, nn_handler
        """ This message is received for selected persons"""
        print("rawStringDataAndroid active: "+ name)
        stopped = False
        nn_handler.selected_person = name

    def IsStopped(self):
        global stopped
        return stopped

    def GoalReached(self):
        global stopped, nn_handler
        stopped = True
        nn_handler.selected_person = ""

    def GeAvailableNames(self):
        global nn_handler
        return nn_handler.get_list_of_persons_with_more_than_three_images()

    def StartDriving(self):
        global stopped
        print("Toggle Driving")
        stopped = not stopped
        return True

    def _save_img_show(self, raw_data, params):

        # thrift prepends 18 bytes to the raw_data, remove them
        data_bytes = self._raw_data_to_jpg(raw_data)
        print("jpg %d bytes" % len(data_bytes))
        save_dir = 'facenet_client_img'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        filename = os.path.join(save_dir ,params.name +'.jpg')
        print ("thrift img saved to:" + filename)
        with open(filename, 'wb+') as f:
            f.write(data_bytes)
        toimage(np.array(imread(filename))).show()#TODO

    def _raw_data_to_jpg(self, raw_data):

        transportOut = TTransport.TMemoryBuffer()
        protocolOut =  TBinaryProtocol.TBinaryProtocol(transportOut)
        raw_data.write(protocolOut)
        data_bytes = transportOut.getvalue()
        print("transportOut.getvalue(): %d bytes" % len(data_bytes))

        # base64 used for save transport over http
        # seven bytes are inserted in the byte array, so the image
        # header is invalid. we delete them manually
        data_bytes = base64.b64decode(data_bytes[7:])

        # thrift prepends 18 bytes to the raw_data, remove them
        return data_bytes[15:] #15



if __name__ == '__main__':
    handler = ExtServiceHandler()
    processor = ExtService.Processor(handler)
    transport = TSocket.TServerSocket(host="localhost", port=1833)
    tfactory = TTransport.TBufferedTransportFactory()
    pfactory = TBinaryProtocol.TBinaryProtocolFactory()
    server = TServer.TThreadedServer(processor, transport, tfactory, pfactory)
    print('Server is online.')
    server.serve()
