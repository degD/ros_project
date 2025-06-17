from flask import Flask, render_template, request, send_file, jsonify
import json
import subprocess
from dotenv import load_dotenv
import os
import json
import numpy as np
import csv
import signal

def convert_numpy(obj):
    if isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    elif isinstance(obj, np.generic):
        return obj.item()
    else:
        return obj
    


load_dotenv()
MAPTILER_KEY = os.getenv("MAPTILER_KEY")
    

app = Flask(__name__)

@app.route("/")
def home():
    try:
        with open("config.json") as fp:
            data = json.load(fp)
        return render_template("index.html", config_data=json.dumps(data), maptile_key=MAPTILER_KEY)
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
        global proc
        os.environ["LD_PRELOAD"] = "/usr/lib/x86_64-linux-gnu/libtiff.so.5"
        data_str = json.dumps(data)
        proc = subprocess.Popen(
            ["python", "main.py", data_str]
        )
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
        with open("coords.json", "r") as f:
            car_locations = json.load(f)
        car_locations = convert_numpy(car_locations)
        # Invert x_global for each car entry
        for car in car_locations:
            if isinstance(car, dict) and "x_global" in car:
                car["x_global"] = -car["x_global"]
                car["angle"] = -car["angle"]
        
        return jsonify(car_locations)
    except FileNotFoundError:
        return jsonify([]) 
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@app.route('/stop_navigation', methods=['POST'])
def stop_navigation():
    try:
        # Remove the completion file if it exists
        if os.path.exists("complete.txt"):
            os.remove("complete.txt")
        # Stop the main.py process if running
        if proc:
            try:
                os.kill(proc, signal.SIGTERM)
            except ProcessLookupError:
                pass  # Process already stopped
        return jsonify({"status": "stopped"}), 200
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