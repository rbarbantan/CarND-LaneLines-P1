import numpy as np
import cv2
from tqdm import tqdm
import os
from sklearn.utils import shuffle
import csv

delta_steering = 0.2


def flip(img, label):
    return np.fliplr(img), -label


def create_dataset(path_to_data, version='v2'):
    """
    Reads images and their corresponding labels.
    :param path_to_data: path to the folder holding the images and labels csv
    :param version: v1 simulator generates relative paths to images, while v2 saves absolute ones
    :return: pair of images and labels (steering values)
    """
    images = []
    labels = []

    with open(f'{path_to_data}/driving_log.csv', 'r') as driving_log:
        reader = csv.reader(driving_log)
        lines = [l for l in reader]

        for line in tqdm(lines):

            if line[0] == 'center':  # skip header
                continue
            if version == 'v1':  # relative paths
                parent = f'{path_to_data}/'
            else:  # absolute paths
                parent = ''

            label = float(line[3])

            center_image_path = parent + line[0]
            if os.path.exists(center_image_path):
                center_img = cv2.cvtColor(cv2.imread(center_image_path), cv2.COLOR_BGR2RGB)

                # original centered image
                images.append(center_img)
                labels.append(label)

                # flipped center image
                flip_center, flip_center_label = flip(center_img, label)
                images.append(flip_center)
                labels.append(flip_center_label)

            left_image_path = parent + line[1].lstrip()
            if os.path.exists(left_image_path):
                left_img = cv2.cvtColor(cv2.imread(left_image_path), cv2.COLOR_BGR2RGB)

                # original left image
                images.append(left_img)
                labels.append(label + delta_steering)

                # flipped left image
                flip_left, flip_left_label = flip(left_img, label + delta_steering)
                images.append(flip_left)
                labels.append(flip_left_label)

            right_image_path = parent + line[2].lstrip()
            if os.path.exists(right_image_path):
                right_img = cv2.cvtColor(cv2.imread(right_image_path), cv2.COLOR_BGR2RGB)

                # original right image
                images.append(right_img)
                labels.append(label - delta_steering)

                # flipped right image
                flip_right, flip_right_label = flip(right_img, label - delta_steering)
                images.append(flip_right)
                labels.append(flip_right_label)

    return shuffle(np.array(images), np.array(labels, dtype='float32'))
