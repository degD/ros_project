from flask import Flask, render_template, request, jsonify
import json
import rospy
import ros_navigation

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
    
    # Print the received data
    data = request.get_json()
    print("Received data:", data)

    waypoints = ros_navigation.convertNavDataToWaypoints(data)
    ros_navigation.simpleNavigation(waypoints)
    
    return "Received data...", 200


if __name__ == "__main__":
    rospy.init_node("navigation_node", anonymous=True)
    app.run(debug=True)
