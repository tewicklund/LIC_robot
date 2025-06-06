from flask import Flask, request, jsonify
from datetime import datetime
import time

log_file_name="test_results.csv"
f=open(log_file_name,'w')
f.write("POST Name,POST Type,Server Timestamp,Client Timestamp,Stop Number,Descriptor")
f.close()

app = Flask(__name__)

@app.route('/robot', methods=['POST'])
def receive_robot_data():
    data = request.json
    print("Received POST request:")
    print(f"test_name: {data.get('test_name')}")
    print(f"epoch_timestamp: {data.get('epoch_timestamp')}")
    print(f"stop_number: {data.get('stop_number')}")
    print(f"arrive_depart: {data.get('arrive_depart')}")
    print(f"Received at: {datetime.now()}")

    f=open(log_file_name,'a')
    f.write(str(data.get('test_name'))+",Robot POST,"+str(time.time())+','+str(data.get('epoch_timestamp'))+','+str(data.get('stop_number'))+','+str(data.get('arrive_depart'))+'\n')
    f.close()
    return jsonify({"status": "success", "message": "Data received"}), 200

@app.route('/LIC_triggers', methods=['POST'])
def receive_LIC_data():
    data = request.json
    print("Received POST request:")
    print(f"Sensor ID: {data.get('sensor_id')}")
    print(f"Timestamp: {data.get('timestamp')}")
    print(f"Received at: {datetime.now()}")

    f=open(log_file_name,'a')
    f.write("N/A,LIC Trigger POST,"+str(time.time())+','+str(data.get('timestamp'))+','+str(data.get('sensor_id'))+',N/A,'+'\n')
    f.close()

    return jsonify({"status": "success", "message": "Data received"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
