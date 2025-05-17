
from flask import Flask, render_template, request, jsonify
import json

app = Flask(__name__)

@app.route("/")
def home():
    try:
        with open("config.json") as fp:
            data = json.load(fp)
        return render_template("index.html", config_data=json.dumps(data))
    except FileNotFoundError:
        pass

@app.route("/submit_area", methods=["POST"])
def submit_area():
    coords = request.json.get("coordinates", [])
    print("Received polygon:", coords)
    return jsonify({"status": "ok", "waypoints": coords})

@app.route("/control", methods=["POST"])
def control():
    action = request.json.get("action")
    print(f"Received action: {action}")
    return jsonify({"status": "ok"})

if __name__ == "__main__":
    app.run(debug=True)
