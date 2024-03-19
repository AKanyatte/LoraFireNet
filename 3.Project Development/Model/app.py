from flask import Flask, render_template, request
from firebase import firebase
from keras.models import load_model
import numpy as np

firebase = firebase.FirebaseApplication('https://sensorstore-7aa66-default-rtdb.firebaseio.com/', None)
model = load_model('fire_pred.h5')

app = Flask(__name__)


@app.route('/', methods=['GET'])
def home():
    # Get all entries from the database
    all_entries = firebase.get('/', None)
    
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
    input_data = np.array([[last_humidity, last_smoke, last_temperature]])
    
    # Perform prediction using the model
    prediction = model.predict(input_data)
    predicted_probability = prediction[0][0]  # Assuming it's a binary classification
    
    # Pass the retrieved data and prediction result to the HTML template
    return render_template('index.html', humidity=last_humidity, smoke=last_smoke, temperature=last_temperature, prediction=predicted_probability)


if __name__ == '__main__':
    app.run(port=3000, debug=True)

