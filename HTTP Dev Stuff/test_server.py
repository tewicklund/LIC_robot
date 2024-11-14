from flask import Flask, request, jsonify
from datetime import datetime

app = Flask(__name__)

@app.route('/robot', methods=['POST'])
def receive_robot_data():
    data = request.json
    print("Received POST request:")
    print(f"epoch_timestamp: {data.get('epoch_timestamp')}")
    print(f"stop_number: {data.get('stop_number')}")
    print(f"arrive_depart: {data.get('arrive_depart')}")
    print(f"Received at: {datetime.now()}")
    return jsonify({"status": "success", "message": "Data received"}), 200

@app.route('/LIC_triggers', methods=['POST'])
def receive_LIC_data():
    data = request.json
    print("Received POST request:")
    print(f"Sensor ID: {data.get('sensor_id')}")
    print(f"Timestamp: {data.get('timestamp')}")
    print(f"Received at: {datetime.now()}")
    return jsonify({"status": "success", "message": "Data received"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
