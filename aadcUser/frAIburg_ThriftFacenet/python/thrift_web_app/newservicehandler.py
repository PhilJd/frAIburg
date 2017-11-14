#!/usr/bin/python
"""Implementation of the py thrift server handler
   interface set in newservice.thrifts
 """
import numpy as np
from scipy.misc import toimage
from scipy.ndimage import imread
import numpy as np
from matplotlib import pylab as plt
import sys
sys.path.append('./gen-py')

import os

from newservice import NewService
from newservice.ttypes import * #TTransport

from thrift.protocol import TJSONProtocol

import base64

from interface_facenet import TFacnetClient

class NewServiceHandler:
    ''' implementation for the thrift server interface for the webapp'''

    def __init__(self):
        '''
        args name_list string list with all aviavle names send to the client
                must be in a valid format to use as an id: no spaces and no dots
        '''
	    #list with all available persons to choose in the app
	    # get_all_names is needed for the client load conted after a reset

        self._facenet_client = TFacnetClient()
        
    def get_person_list(self):
        list_names = self._facenet_client.get_available_names()
        if not NewServiceHandler._check_name_list(list_names) :
            print("NewServiceHandler name list not valid")
        return list_names

    def ping(self, s1):
        # thrift interface implementation
        print("thrift NewServiceHandler called ping with msg %s" % s1)
        return self._facenet_client.ping(s1)

    def get_all_names(self):
        # thrift interface implementation
        print("thrift NewServiceHandler called get_all_names")

        # names will be shown in reversed order in the web app
        return self.get_person_list()

    def add_person(self, raw_data, params):
        # thrift interface implementation
        # return ture if a NEW person was added
        print ("thrift NewServiceHandler add_person called with {} ".format(params))
        if not NewServiceHandler._is_name_valid(params.name):
            print("thrift NewServiceHandler ERROR client send",
                  " a not vaild id name ", params.name)

        #self._save_img_show(raw_data, params)

        self._facenet_client.send_add_person(
            raw_data.raw_data,#raw_data,
            params.height, params.width, params.name)

        if params.name in self.get_person_list():
            #person is available to select
            return True
        else:
            print("thrift person was not in facent available list list: " + params.name)
            return False

    def remove_person(self, s1):
        # thrift interface implementation
        print ("thrift NewServiceHandler remove_person(): %s" % s1)
        if not NewServiceHandler._is_name_valid(s1):
            print("thrift NewServiceHandler ERRO9R client ,"
                  "send a not vaild id name ", s1)
        if s1 in self.get_person_list():
            # while not needed person name added only once
            # TODO
            print("remove not yet implemented!")
            return True
        else:
            print("thrift NewServiceHandler ERROR remove person not found")
            return False

    def drive_to(self, s1):

        print ("thrift NewServiceHandler drive_to: %s" % s1)
        self._facenet_client.send_drive_to_person(s1)
        return True

    def start_driving(self):
        print ("thrift NewServiceHandler start driving")
        return self._facenet_client.send_start_driving_cmd()


    def _save_img_show(self, raw_data, params):

        # thrift prepends 18 bytes to the raw_data, remove them
        data_bytes = self._raw_data_to_jpg(raw_data)
        print("jpg %d bytes" % len(data_bytes))

        save_dir = 'client_img'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        filename = os.path.join(save_dir ,params.name +'.jpg')
        print ("thrift img saved to:", filename)
        with open(filename, 'wb+') as f:
            f.write(data_bytes)
        toimage(np.array(imread(filename))).show()#TODO

    def _raw_data_to_jpg(self, raw_data):

        transportOut = TTransport.TMemoryBuffer()
        protocolOut = TJSONProtocol.TJSONProtocol(transportOut)
        raw_data.write(protocolOut)
        data_bytes = transportOut.getvalue()

        print("transportOut.getvalue(): %d bytes" % len(data_bytes))
        print("raw_data.raw_data: %d bytes" % len(raw_data.raw_data))
        #base64 used for save transport over http
        data_bytes = base64.b64decode(data_bytes)

        # thrift prepends 18 bytes to the raw_data, remove them

        return data_bytes[18:]
        # return data_bytes[18:]
        # return base64.b64decode(raw_data.raw_data[7:])

    @staticmethod
    def _is_name_valid(name):
        ''' check if a string name is in a vaild format:
            no spaces (first trimmed), not dots.
            needed to ensure that the name are same on the server and client side
            if used as an id.
        args: name string
        returns: True if vaild
        '''
        invalid_char = [" ", "."]
        if any (x in name for x in invalid_char):
            return False
        else :
            return True

    @staticmethod
    def _correct_name(name):
        ''' corrects a name to use as an id: trimmed no spaces and dots
        args name string
        returns string in a correct format
        '''
        return name.strip().replace(" ", "_").replace(".", "_")

    @staticmethod
    def _check_name_list(name_list):
        ''' check name string list if all names are in a vaild format to use as
            an id
            prints error msg
        args name string list
        returns False if not valid
        '''
        for name in name_list:
            if not NewServiceHandler._is_name_valid(name):
                print("NewServiceHandler name \"%s\" not a valid id format:"
                " no spaces and no dots" % name)
                return False

        return True
