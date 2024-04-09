"""
File name: app.py

This script utilizes a Flask web application to display sensor data and predict the probability of wildfire
outbreaks based on temperature, humidity, and smoke levels. The sensor data is retrieved from a 
Firebase Realtime Database.
Predictions are stored in a Firebase Realtime Database.
The prediction model is loaded from a pre-trained Keras model.

Author: Ashley Kanyatte

"""

from flask import Flask, render_template
from firebase import firebase
from keras.models import load_model
import numpy as np
import threading
import time

# Initialize Firebase connections
firebase1 = firebase.FirebaseApplication('https://sensorstore-7aa66-default-rtdb.firebaseio.com/', None)
firebase2 = firebase.FirebaseApplication('https://predictions-ba982-default-rtdb.firebaseio.com/', None)

# Load pre-trained Keras model
model = load_model('fire_pred.h5')

# Initialize Flask application
app = Flask(__name__)


"""
Function to continuously perform fire probability prediction
and store the result in Firebase Realtime Database.
"""
def perform_prediction_and_store():
    while True:
        # Get the latest sensor data from Firebase database
        all_entries = firebase1.get('/', None)
        last_humidity = None
        last_smoke = None
        last_temperature = None

        # Iterate over all sensor entries to get the latest data
        for entry_key, entry_data in all_entries.items():
            last_humidity = entry_data['humidity']
            last_smoke = entry_data['smoke']
            last_temperature = entry_data['temperature']

        # Perform prediction using the model
        input_data = np.array([[last_temperature, last_humidity, last_smoke]])
        prediction = model.predict(input_data) * 100
        predicted_probability = prediction[0][0]

        # Store the prediction result in the database
        firebase2.post('/', {'predicted_probability': float(predicted_probability)})

        # Wait for some time before repeating the process
        time.sleep(345) 

# Start the background task for prediction and storage
prediction_thread = threading.Thread(target=perform_prediction_and_store)
prediction_thread.daemon = True
prediction_thread.start()

"""
Route handler for the home page.
Displays sensor data and predicted fire probability.
"""
@app.route('/', methods=['GET'])
def home():
    # Get all entries from the database
    all_entries = firebase1.get('/', None)
    
    # Initialize variables to store the last entry's data
    last_humidity = None
    last_smoke = None
    last_temperature = None
    
    # Iterate over each entry to find the last one
    for entry_key, entry_data in all_entries.items():
        last_humidity = entry_data['humidity']
        last_smoke = entry_data['smoke']
        last_temperature = entry_data['temperature']

    # Convert input data to a NumPy array and reshape it
    input_data = np.array([[last_temperature, last_humidity, last_smoke]])
    
    # Perform prediction using the model
    prediction = model.predict(input_data) * 100
    predicted_probability = prediction[0][0]  
    
    # Pass the retrieved data and prediction result to the HTML template
    return render_template('index.html', humidity=last_humidity, smoke=last_smoke, temperature=last_temperature, prediction=predicted_probability)

# Run the Flask application
if __name__ == '__main__':
    app.run(host="0.0.0.0", port=3000, debug=False)
    