from flask import Flask, render_template, request, send_file
import json
import subprocess
import os

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


if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)  # Disable auto-reloader