import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import pandas as pd
import numpy as np

time_steps = 10
model_name = "temp_model"

def convertToMatrix(data, step):
    X = []
    for i in range(len(data) - step):
        d = i+step
        X.append(data[i:d,])
    return np.array(X)

def read_format_csv(filename):
    #TODO does the model perform better with this data?
    #       If so do we need all of the data or just some?
    ignore_cols = ("time","IMU#","accelx","accely","accelz","compassx","compassy","compassz")
    data = pd.read_csv(filename)

    for col in ignore_cols:
        data.pop(col)

    return np.array(data)

def get_in(filename):
    inputs = convertToMatrix(read_format_csv(filename), time_steps)
    return inputs


def predict_csv(filename):
    model = keras.models.load_model(model_name)

    
    data_in = get_in(filename) 
    prediction = model.predict(data_in) # Predict the output for the input data

    prediction_comp = prediction.flatten()

    # data is fed in 10 points at a time so need to add 10 zero points 
    #   at start of prediction and angle vectors the same length
    for i in range(time_steps):
        prediction_comp = np.insert(prediction_comp, 0, 0)

    # Scale prediction from (0 to 1) to (0 to 100)
    prediction_comp *= 100
    return prediction_comp


#csv_file supplied by MATLAB
outVect = predict_csv(csv_file)
