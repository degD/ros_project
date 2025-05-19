
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

@app.route("/navigation", methods=["POST"])
def navigation():
    print("Got data!")
    return "Received data...", 200

if __name__ == "__main__":
    app.run(debug=True)
