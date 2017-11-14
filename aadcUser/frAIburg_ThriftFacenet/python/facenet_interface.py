# Copyright team frAIburg, 2017
# Author: Philipp Jund (jundp@informatik.uni-freiburg.de)
#
# Interface to facenet by David Sandberg

import os
import sys
import numpy as np
from random import randint
from PIL import Image
from sklearn import neighbors
from scipy import misc
from ExtIf import ttypes  # thrift types
from ExtIf.ttypes import *
import tensorflow as tf
from facenet.src import facenet
import facenet.src.align.detect_face as face_detector


MINIMIUM_DIST = 1.1
DATASET_DIR = "/home/aadc/Downloads/person_dataset/"
# graph protobuf
MODEL = "/home/aadc/ADTF/weights/facenet/20170511-185253/20170511-185253.pb"
#MODEL = "/home/aadc/ADTF/weights/facenet/20170512-110547/20170512-110547.pb"

MIN_FACE_SIZE = 20
DETECTION_THRESHOLDS = [0.6, 0.7, 0.7]  # three steps' DETECTION_THRESHOLDS
SCALE_FACTOR = 0.709
FACE_ADD_MARGIN = 32   # margin that is added to the detections
FACE_SIZE = (160, 160)  # the size to which the detections are resized


class NearestNeighborHandler:
    """ Keeps a database of known embeddings. Efficiently gets the name of
        the nearest neighbor.
    """

    def __init__(self):
        self.facenet_controller = FacenetController()
        self.embeddings = []
        self.labels = []
        self.persons = []
        self.selected_person = ""
        self.nn_classifier = neighbors.KNeighborsClassifier(n_neighbors=3)
        self._precompute_embeddings()
        print(self.labels)
        print(self.persons)
        self.nn_classifier.fit(self.embeddings, self.labels)

    def _precompute_embeddings(self):
        """ Computes the embeddings for the persons in DATASET_DIR. """
        # We assume that the pictures of each person are in a
        # correspondingly named directory
        print("Precomputing embeddings for known persons.")
        for name in os.listdir(DATASET_DIR):
            person_dir = os.path.join(DATASET_DIR, name)
            if not os.path.isdir(person_dir):
                continue
            # We'll only deal with very few persons, so we save them in a
            # list where the index within this list serves as an id.
            for img_file_name in os.listdir(person_dir):
                path = os.path.join(person_dir, img_file_name)
                try:
                    img = np.asarray(Image.open(path), dtype=np.uint8)
                except OSError:  # unknown file format
                    continue
                # Setting fit_nn_classifier to False as we'll fit the
                # nn_classifier once after all initial persons have been added
                self.add_person(img, name, fit_nn_classifier=False)

    def add_person(self, image, name, fit_nn_classifier=True, new_image=False):
        """ Computes the embedding and adds it to the database. """
        _, emb = self.facenet_controller.get_embeddings(image)
        if len(emb) != 1:  # more than one or no face found
            print("Error: more than one or no face found. No person added.")
            return
        self.embeddings += [emb[0]]
        if name not in self.persons:
            self.persons.append(name)
        self.labels.append(self.persons.index(name))
        #if fit_nn_classifier:
        self.nn_classifier.fit(self.embeddings, self.labels)
        if new_image:
            folder = os.path.join(DATASET_DIR, name)
            if not os.path.isdir(folder):
                os.mkdir(folder)
            idx = self.labels.count(self.persons.index(name))
            path = os.path.join(folder, str(idx) + ".jpg")
            print("facenet_interface: saved img to {}".format(path))
            Image.fromarray(image).save(path)

    def get_persons(self, image):
        """ Finds faces and labels them with a name if they're known. """
        bounding_boxes, embs = self.facenet_controller.get_embeddings(image)
        if len(embs) == 0:
            return []
        dists, instance_indices = self.nn_classifier.kneighbors(embs, 3, True)
        # todo(phil): verify bounding box format
        print("dists, instance_indices", dists, instance_indices)
        result = []
        print(self.persons)
        for i, (idx, d) in enumerate(zip(instance_indices, dists)):
            labels = [self.labels[l] for l in idx]
            print("labels, distance:", labels, d)
            unique_ids, counts = np.unique(labels, return_counts=True)
            if len(unique_ids) == 3:
                name = "unknown"
                d = np.mean(d)
            else:
                print("counts.argmax()", counts.argmax())
                print("d", d, "idx", idx, "unique_ids", unique_ids)
                d = np.mean(d[labels == unique_ids[counts.argmax()]])
                print("d", d)
                if d > MINIMIUM_DIST:
                    name = "unknown"
                else:
                    name = self.persons[int(unique_ids[counts.argmax()])]
            result += [TObjectResult(name, d, self.selected_person == name,
                                     *bounding_boxes[i, :4])]
        return result

    def get_list_of_persons_with_more_than_three_images(self):
        """ Returns all persons that have at least 3 ground truth images."""
        return [p for i, p in enumerate(self.persons) if self.labels.count(i) >= 3]


class FacenetController:

    def __init__(self):
        self.session = None
        # The face detection nets
        self.pnet, self.rnet, self.onet = None, None, None
        self.images_pl = None
        self.embeddings_op = None
        self.is_train_pl = None
        self.setup_facenet()

    def setup_facenet(self):
        """ Sets up input placeholders and fetch operations. """
        with tf.Graph().as_default():
            self.session = tf.Session()
            facenet.load_model(MODEL)
            # Get input and output tensors
            graph = tf.get_default_graph()
            self.images_pl = graph.get_tensor_by_name("input:0")
            self.embeddings_op = graph.get_tensor_by_name("embeddings:0")
            self.is_train_pl = graph.get_tensor_by_name("phase_train:0")
            # setup face detection networks
            self.pnet, self.rnet, self.onet = \
                face_detector.create_mtcnn(self.session, None)
        print("Tensorflow net is ready.")

    def get_embeddings(self, image):
        """ Detects faces in the image and computes the embeddings. """
        bounding_boxes, _ = \
            face_detector.detect_face(image, MIN_FACE_SIZE,
                                      self.pnet, self.rnet, self.onet,
                                      DETECTION_THRESHOLDS, SCALE_FACTOR)
        if len(bounding_boxes) == 0:
            return [], []
        images = []
        for box in bounding_boxes:
            img_size = np.asarray(image.shape)[0:2]
            box = self._add_margin(box, img_size)
            cropped = image[int(box[1]):int(box[3]), int(box[0]):int(box[2]), :]
            cropped = misc.imresize(cropped, FACE_SIZE, interp='bilinear')
            images += [facenet.prewhiten(cropped)]
        fd = { self.images_pl: images , self.is_train_pl: False }
        return (bounding_boxes,
                self.session.run(self.embeddings_op, feed_dict=fd))

    def _add_margin(self, box, img_size):
        """ Adds a margin to the detected bounding boxes. """
        box[0] = np.maximum(box[0] - FACE_ADD_MARGIN / 2, 0)
        box[1] = np.maximum(box[1] - FACE_ADD_MARGIN / 2, 0)
        box[2] = np.minimum(box[2] + FACE_ADD_MARGIN / 2, img_size[1])
        box[3] = np.minimum(box[3] + FACE_ADD_MARGIN / 2, img_size[0])
        return box
