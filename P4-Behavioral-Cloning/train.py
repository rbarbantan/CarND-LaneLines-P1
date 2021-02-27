import argparse
from dataset import create_dataset
from tensorflow.keras import Model, Sequential
from tensorflow.keras.layers import Input, Flatten, Dense, MaxPooling2D, Conv2D, Lambda, Cropping2D, Dropout
import tensorflow as tf
import matplotlib.pyplot as plt
import time


def get_simple_model():
    """
    Simplest model to test that the pipeline is working correctly
    :return: keras model
    """
    model = Sequential()
    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160, 320, 3)))
    model.add(Flatten())
    model.add(Dense(1))

    model.compile(loss='mse', optimizer='adam')
    return model


def get_lenet_model():
    """
    LeNet architecture
    :return: keras model
    """
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
    tf.keras.utils.plot_model(model, to_file='data/lenet.png')

    return model


def get_pilotnet_model():
    """
    Nvidia PilotNet architecture
    :return: keras model
    """
    inputs = Input(shape=(160, 320, 3))  # input as received from simulation
    x = Lambda(lambda img: img / 255.0 - 0.5)(inputs)  # normalization
    x = Cropping2D(cropping=((70, 25), (0, 0)))(x)  # cropping out top and bottom of the image

    x = Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), activation="relu")(x)
    x = Conv2D(filters=64, kernel_size=(3, 3), activation="relu")(x)
    x = Conv2D(filters=64, kernel_size=(3, 3), activation="relu")(x)

    x = Flatten()(x)
    x = Dense(100, activation='relu')(x)
    x = Dense(50, activation='relu')(x)
    x = Dense(10, activation='relu')(x)
    out = Dense(1, activation='linear')(x)

    model = Model(inputs=inputs, outputs=out)
    model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3))
    model.summary()

    # needs graphviz installed
    tf.keras.utils.plot_model(model, to_file='data/pilotnet.png')

    return model


def plot_history(history):
    """
    summarize history for loss
    """
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('model loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()


def lr_scheduler(epoch, lr):
    if epoch < 5:
        return 1e-3
    else:
        return 1e-4


def train(args):
    x, y = create_dataset(args.data_path, version='v2')
    print(f'Dataset shape {x.shape}')

    # model = get_lenet_model()
    model = get_pilotnet_model()

    reduce_lr = tf.keras.callbacks.LearningRateScheduler(lr_scheduler)
    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=f"./logs/{time.time()}")
    history = model.fit(x, y, batch_size=32, validation_split=0.2, epochs=10, callbacks=[reduce_lr, tensorboard_callback])
    plot_history(history)

    model.save('model.h5')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', default='data', help='Path to the recorded data')

    args = parser.parse_args()

    train(args)
