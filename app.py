from flask import Flask, render_template, request, send_file, jsonify
import json
import subprocess
import os
import json
import numpy as np
import csv

def convert_numpy(obj):
    if isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    elif isinstance(obj, np.generic):
        return obj.item()
    else:
        return obj
    

app = Flask(__name__)

@app.route("/")
def home():
    try:
        with open("config.json") as fp:
            data = json.load(fp)
        return render_template("index.html", config_data=json.dumps(data))
    except FileNotFoundError:
        return "Configuration file not found.", 404


@app.route("/navigation", methods=["POST"])
def navigation():
    print("Got data!")
    
    # Print the received data
    data = request.get_json()
    print("Received data:", data)

    # Call the separate ROS script with subprocess
    try:
        os.environ["LD_PRELOAD"] = "/usr/lib/x86_64-linux-gnu/libtiff.so.5"
        subprocess.Popen(["python", "main.py"], stdin=subprocess.PIPE)
    except Exception as e:
        print(f"Error launching ROS script: {e}")
    
    return "Received data...", 200


@app.route("/navigation_status")
def navigation_status():
    if os.path.exists("complete.txt"):
        return "Navigation complete!", 200
    else:
        return "Navigation in progress...", 202


@app.route('/plates.txt')
def download_plates():
    return send_file('plates.txt', as_attachment=True)


drone_position = {"x": 0, "y": 0, "z": 0}

@app.route('/update_drone_position', methods=['POST'])
def update_drone_position():
    data = request.json
    drone_position.update(data)
    return jsonify({"status": "ok"})

@app.route('/get_drone_position')
def get_drone_position():
    return jsonify(drone_position)


@app.route('/get_car_locations')
def get_car_locations():
    try:
        with open("coords.json", "rb") as f:
            car_locations = json.load(f)
        car_locations = convert_numpy(car_locations)
        return jsonify(car_locations)
    except FileNotFoundError:
        return jsonify([]) 
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    

@app.route('/get_car_plates')
def get_car_plates():
    try:
        result = []
        with open("plates.txt", "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    result.append([int(row[0]), row[1]])
        return jsonify(result)
    except FileNotFoundError:
        return jsonify([]) 
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)  # Disable auto-reloader