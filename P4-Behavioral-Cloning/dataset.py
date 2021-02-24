import numpy as np
import cv2
from tqdm import tqdm

delta_steering = 0.2


def flip(img, label):
    return np.fliplr(img), -label


def create_dataset(path_to_data):
    """
    Reads images and their corresponding labels.
    :param path_to_data: path to the folder holding the images and labels csv
    :return: pair of images and labels (steering values)
    """
    images = []
    labels = []

    with open(f'{path_to_data}/driving_log.csv', 'r') as driving_log:
        for line in tqdm(driving_log.readlines()):
            items = line.split(',')

            center_img = cv2.imread(items[0])
            left_img = cv2.imread(items[1])
            right_img = cv2.imread(items[2])
            label = float(items[3])

            # original centered image
            images.append(center_img)
            labels.append(label)

            # original left image
            images.append(left_img)
            labels.append(label + delta_steering)

            # original right image
            images.append(right_img)
            labels.append(label - delta_steering)

            # flipped center image
            flip_center, flip_center_label = flip(center_img, label)
            images.append(flip_center)
            labels.append(flip_center_label)

            # flipped left image
            flip_left, flip_left_label = flip(left_img, label + delta_steering)
            images.append(flip_left)
            labels.append(flip_left_label)

            # flipped right image
            flip_right, flip_right_label = flip(right_img, label - delta_steering)
            images.append(flip_right)
            labels.append(flip_right_label)

    return np.array(images), np.array(labels)
