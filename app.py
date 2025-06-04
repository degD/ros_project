from flask import Flask, render_template, request, jsonify
import json
import rospy
import ros_library as ros_library

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

    waypoints = ros_library.convertNavDataToWaypoints(data)
    ros_library.simpleNavigation(waypoints)
    
    return "Received data...", 200


if __name__ == "__main__":
    # rospy.init_node("navigation_node")
    app.run(debug=True)
