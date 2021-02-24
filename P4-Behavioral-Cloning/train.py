import argparse
from dataset import create_dataset
from tensorflow.keras.applications.inception_v3 import InceptionV3
from tensorflow.keras import Model
from tensorflow.keras.layers import Input, Flatten, Dense, GlobalAveragePooling2D, Conv2D
import tensorflow as tf


def get_model():
    inputs = Input(shape=(160, 320, 3))  # input as received from simulation

    x = tf.keras.layers.Conv2D(filters=6, kernel_size=(5, 5), activation="relu")(inputs)
    x = tf.keras.layers.MaxPooling2D(pool_size=(2, 2))(x)

    x = tf.keras.layers.Conv2D(filters=16, kernel_size=(5, 5), activation="relu")(x)
    x = tf.keras.layers.MaxPooling2D(pool_size=(2, 2))(x)

    x = tf.keras.layers.Flatten()(x)
    x = tf.keras.layers.Dense(120, activation='relu')(x)
    x = tf.keras.layers.Dropout(0.5)(x)

    x = tf.keras.layers.Dense(84, activation='relu')(x)
    x = tf.keras.layers.Dropout(0.5)(x)

    out = Dense(1)(x)

    model = Model(inputs=inputs, outputs=out)
    model.compile(loss='mse', optimizer='adam')
    model.summary()

    # needs graphviz installed
    tf.keras.utils.plot_model(model, to_file='data/model.png')

    return model


def train(args):
    x, y = create_dataset(args.data_path)
    print(f'Dataset shape {x.shape}')

    model = get_model()

    model.fit(x, y, epochs=5, batch_size=32, validation_split=0.2, shuffle=True)

    model.save('model.h5')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', default='data', help='Path to the recorded data')

    args = parser.parse_args()

    train(args)
