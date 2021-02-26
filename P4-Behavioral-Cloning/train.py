import argparse
from dataset import create_dataset
from tensorflow.keras.applications.inception_v3 import InceptionV3
from tensorflow.keras import Model, Sequential
from tensorflow.keras.layers import Input, Flatten, Dense, MaxPooling2D, Conv2D, Lambda, Cropping2D, Dropout
import tensorflow as tf
from sklearn.model_selection import train_test_split


def get_simple_model():
    model = Sequential()
    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160, 320, 3)))
    model.add(Flatten())
    model.add(Dense(1))

    model.compile(loss='mse', optimizer='adam')
    return model


def get_model():
    inputs = Input(shape=(160, 320, 3))  # input as received from simulation
    x = Lambda(lambda img: img / 255.0 - 0.5)(inputs)  # normalization
    x = Cropping2D(cropping=((70, 25), (0, 0)))(x)  # cropping out top and bottom of the image

    x = Conv2D(filters=6, kernel_size=(5, 5), activation="relu")(x)
    x = MaxPooling2D(pool_size=(2, 2))(x)

    x = Conv2D(filters=16, kernel_size=(5, 5), activation="relu")(x)
    x = MaxPooling2D(pool_size=(2, 2))(x)

    x = Flatten()(x)
    x = Dense(120, activation='relu')(x)
    x = Dropout(0.5)(x)

    x = Dense(84, activation='relu')(x)
    x = Dropout(0.5)(x)

    out = Dense(1, activation='linear')(x)

    model = Model(inputs=inputs, outputs=out)
    model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(learning_rate=1e-4))
    model.summary()

    # needs graphviz installed
    tf.keras.utils.plot_model(model, to_file='data/model.png')

    return model


def train(args):
    x, y = create_dataset(args.data_path, version='v1')
    print(f'Dataset shape {x.shape}')

    x_train, x_val, y_train, y_val = train_test_split(x, y, test_size=0.2)

    #gen = tf.keras.preprocessing.image.ImageDataGenerator(
    #    featurewise_center=True,
    #    featurewise_std_normalization=True)
    #gen.fit(x_train[:1000])

    model = get_model()
    #model = get_simple_model()

    #model.fit(gen.flow(x_train, y_train, batch_size=32),
    #          validation_data=gen.flow(x_val, y_val, batch_size=32),
    #          validation_steps=len(x_val)/32,
    #          epochs=5,
    #          steps_per_epoch=len(x_train)/32)
    model.fit(x, y, batch_size=32, validation_split=0.2, epochs=10)

    model.save('model.h5')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', default='data', help='Path to the recorded data')

    args = parser.parse_args()

    train(args)
