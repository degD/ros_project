import numpy as np
import detect
import plate
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
import math
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import requests
import mmrotate
import json

EPS = 0.5
MAX_DIST = 2.0  # Maximum distance for each step
INFERENCE_THRESHOLD = 0.4
INFERENCE_Z = 20
VEHICLE_EPS = 5
URL_POS = "http://localhost:5000/update_drone_position"  
PLATE = True   # as a result makes plate texts DISABLED and skips recognizing plates.

def get_current_pose():
    """
    Helper function to get the current pose of the UAV.
    Returns:
        Pose: The current pose of the UAV.
    """
    latest_pose = {"pose": None}
    def pose_callback(msg):
        latest_pose["pose"] = msg

    sub = rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_pose["pose"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    sub.unregister()
    if latest_pose["pose"] is None:
        rospy.logwarn("Could not get current pose.")
    return latest_pose["pose"]
def get_depth_image(timeout_sec=2.0):
    """
    Alır: Vi-Sensor'dan derinlik görüntüsünü alır.
    Döner: Derinlik görüntüsünü (float32, metre cinsinden) bir NumPy array olarak döner.

    Returns:
        np.ndarray: Derinlik görüntüsü, float32 tipinde. Başarısız olursa None döner.
    """
    bridge = CvBridge()
    depth_data = {"image": None}

    def depth_callback(msg):
        try:
            depth_data["image"] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Depth image conversion failed: {e}")

    sub = rospy.Subscriber('/firefly/vi_sensor/camera_depth/camera/image_raw', Image, depth_callback)
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(timeout_sec)

    while not rospy.is_shutdown() and depth_data["image"] is None and rospy.Time.now() < timeout:
        rate.sleep()

    sub.unregister()

    if depth_data["image"] is None:
        rospy.logwarn("Derinlik görüntüsü alınamadı.")
        return None

    return depth_data["image"]

def send_position_to_flask():
    pose = get_current_pose()
    if pose is None:
        return
    pos = pose.position
    url = URL_POS
    data = {"x": pos.x, "y": pos.y, "z": pos.z}
    try:
        requests.post(url, json=data)
    except Exception as e:
        rospy.logerr(f"Failed to send position to Flask: {e}")
import numpy as np

def angle_between(v1, v2):
    """İki vektör arasındaki açıyı radian cinsinden döner."""
    v1_u = v1 / (np.linalg.norm(v1) + 1e-8)
    v2_u = v2 / (np.linalg.norm(v2) + 1e-8)
    dot = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.arccos(dot)

def rotate_vector(vec, angle):
    """2D vektörü saat yönünün tersine döndürür."""
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    return R @ vec

def is_free_toward(pointcloud, direction_vector, threshold=1.5, cone_angle=np.radians(15)):
    """
    Belirli bir yöne doğru, konik alanda bir engel olup olmadığını kontrol eder.
    """
    for pt in pointcloud:
        vec = np.array([pt[0], pt[1]])
        dist = np.linalg.norm(vec)
        if dist > threshold:
            continue
        angle = angle_between(vec, direction_vector)
        if abs(angle) < cone_angle:
            return False
    return True

def find_alternate_direction(pointcloud, original_dir, angle_offsets=np.radians([-45,-30,-15,15,30,45])):
    """
    Orijinal yön engelliyse, verilen sapma açılarıyla rotasyon deneyerek yeni güvenli yön arar.
    """
    for offset in angle_offsets:
        new_dir = rotate_vector(original_dir, offset)
        if is_free_toward(pointcloud, new_dir):
            return new_dir
    return None

def extract_pointcloud_from_depth(depth_image, fx=554.25469, fy=554.25469, cx=320.5, cy=240.5, scale=1.0):
    """
    Derinlik görüntüsünden 2D düzlemde (x,y) koordinatları çıkarır.
    """
    h, w = depth_image.shape
    points = []
    for v in range(0, h, 10):  # daha seyrek örnekleme
        for u in range(0, w, 10):
            d = depth_image[v, u]
            if np.isnan(d) or d <= 0:
                continue
            z = d * scale
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points.append([x, y])
    return points

def adjust_goal_if_blocked(current_pos: Point, next_wp: Point, dist_limit=MAX_DIST) -> Point:
    """
    Derinlik verisine dayalı konik engel taraması ile hedef günceller.
    """
    depth_img = get_depth_image()
    if depth_img is None:
        rospy.logwarn("Derinlik verisi alınamadı. Orijinal hedef kullanılacak.")
        return next_wp

    pointcloud = extract_pointcloud_from_depth(depth_img)
    
    # Hedef yön vektörünü hesapla
    dx = next_wp.x - current_pos.x
    dy = next_wp.y - current_pos.y
    vec = np.array([dx, dy])
    dist = np.linalg.norm(vec)
    direction = vec / (dist + 1e-6)
    dist = min(dist, dist_limit)

    if is_free_toward(pointcloud, direction):
        return next_wp

    rospy.loginfo("Hedef yönü engelli. Alternatif yön aranıyor...")
    new_dir = find_alternate_direction(pointcloud, direction)
    if new_dir is None:
        rospy.logwarn("Hiçbir alternatif yön bulunamadı. Orijinal hedefe devam.")
        return next_wp

    # Yeni yön ile hedef oluştur
    new_goal = Point()
    new_goal.x = current_pos.x + new_dir[0] * dist
    new_goal.y = current_pos.y + new_dir[1] * dist
    new_goal.z = next_wp.z

    rospy.loginfo(f"Yeni yön seçildi: ({new_goal.x:.2f}, {new_goal.y:.2f}, {new_goal.z:.2f})")
    return new_goal
def plan_path_with_avoidance(current_pos: Point, final_goal: Point, step_size=0.4, max_steps=50):
    """
    Orijinal hedefe küçük adımlarla yaklaşırken her adımda engel kontrolü yapar.
    Engel varsa yön sapması yaparak engelden kaçınmaya çalışır.
    Args:
        current_pos: Başlangıç pozisyonu (Point).
        final_goal: Nihai hedef pozisyonu (Point).
        step_size: Her adımda gidilecek mesafe (metre).
        max_steps: Maksimum adım sayısı (güvensiz durumda sonsuz döngüye girmemek için).
    Returns:
        List[Point]: Ara hedef listesi. Yol bulunamazsa None döner.
    """
    path = []
    pos = Point()
    pos.x, pos.y, pos.z = current_pos.x, current_pos.y, current_pos.z

    for _ in range(max_steps):
        dx = final_goal.x - pos.x
        dy = final_goal.y - pos.y
        dz = final_goal.z - pos.z
        dist = np.linalg.norm([dx, dy])

        if dist < step_size:
            # Nihai hedefe ulaşıldı
            path.append(final_goal)
            rospy.loginfo("Nihai hedefe ulaşıldı.")
            return path

        # Bir sonraki ara hedefi hesapla
        direction = np.array([dx, dy]) / (dist + 1e-6)
        tentative_goal = Point()
        tentative_goal.x = pos.x + direction[0] * step_size
        tentative_goal.y = pos.y + direction[1] * step_size
        tentative_goal.z = final_goal.z

        # Engel varsa yeni yön bul
        new_goal = adjust_goal_if_blocked(pos, tentative_goal, dist_limit=step_size)

        if new_goal is None:
            rospy.logwarn("Yol bulunamadı, kaçınma başarısız.")
            return None

        # Eğer yeni hedef, önerilenden çok farklı ise (engelden kaçma yapıldıysa)
        if np.linalg.norm([new_goal.x - tentative_goal.x, new_goal.y - tentative_goal.y]) > 1e-3:
            rospy.loginfo(f"Engel var, yeni ara hedef: ({new_goal.x:.2f}, {new_goal.y:.2f})")
        
        path.append(new_goal)
        pos = new_goal

    rospy.logwarn("Maksimum adım sayısına ulaşıldı, yol tamamlanamadı.")
    return None



def stepMove(target_point: Point):
    """
    Moves the UAV to the target point in steps, each not exceeding MAX_DIST.
    Args:
        target_point (Point): The final destination point.
    """
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Aborting stepMove.")
        return

    start = current_pose.position
    distance = math.sqrt(
        (target_point.x - start.x) ** 2 +
        (target_point.y - start.y) ** 2 +
        (target_point.z - start.z) ** 2
    )
    rospy.loginfo(f"Starting stepMove: distance to target = {distance:.2f} m")

    steps = int(math.ceil(distance / MAX_DIST)) if distance > 0 else 1
    rospy.loginfo(f"Splitting move into {steps} steps.")

    for i in range(1, steps + 1):
        dx = target_point.x - start.x
        dy = target_point.y - start.y
        dz = target_point.z - start.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance < EPS:
            break

        # Move a max of MAX_DIST or the remaining distance
        step_size = min(MAX_DIST, distance)
        ratio = step_size / distance
        intermediate = Point(
            start.x + dx * ratio,
            start.y + dy * ratio,
            start.z + dz * ratio
        )
        rospy.loginfo(f"Step {i}/{steps}: Moving to\n{intermediate}")
        simpleMove(intermediate)
        start = intermediate

    rospy.loginfo("Arrived at target using stepMove.")



def simpleMove(point: Point):
    """
    Publishes a PoseStamped message to a ROS topic and waits until the robot arrives at the target point.

    Args:
        point (Point): The target point to move to.
    """
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position = point
    pose_stamped.pose.orientation.w = 1.0

    rospy.loginfo(f"Publishing target pose:\n{point}")

    latest_pose = {"pose": None}
    def pose_callback(msg):
        latest_pose["pose"] = msg

    sub = rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    rate = rospy.Rate(40)
    arrived = False

    rospy.loginfo("Waiting for the robot to arrive...")
    while not rospy.is_shutdown() and not arrived:
        pub.publish(pose_stamped)
        pose = latest_pose["pose"]
        if pose is not None:
            if (
                abs(pose.position.x - point.x) < EPS and
                abs(pose.position.y - point.y) < EPS and
                abs(pose.position.z - point.z) < EPS
            ):
                arrived = True
        rate.sleep()

    rospy.loginfo("Robot has arrived at the target point.")
    sub.unregister()
    send_position_to_flask()
def is_direction_blocked(depth_image, angle_offset_deg=0, distance_threshold=200.0, width_pixels=10):
    """
    Belirli bir yönde engel olup olmadığını derinlik görüntüsüne göre kontrol eder.
    depth_image: np.ndarray, genellikle uint8 (cm) ya da float32 (metre) tipinde olur.
    """
    import numpy as np
    height, width = depth_image.shape
    center_x = width // 2
    FOV_DEG = 90.0
    pixels_per_degree = width / FOV_DEG

    target_x = int(center_x + angle_offset_deg * pixels_per_degree)
    target_x = max(width_pixels // 2, min(width - width_pixels // 2 - 1, target_x))
    start_x = target_x - width_pixels // 2
    end_x = target_x + width_pixels // 2

    vertical_region = depth_image[height//2:, start_x:end_x]

    valid_depths = vertical_region[(vertical_region > 0) & (~np.isnan(vertical_region))]

    if valid_depths.size == 0:
        rospy.logwarn("Hiç geçerli derinlik verisi bulunamadı. Engel varsayılmadı.") 
        return False

    min_distance = valid_depths.min()

    rospy.loginfo(f"Min derinlik değeri: {min_distance:.2f} (eşik: {distance_threshold})") 

    if min_distance < distance_threshold:
        rospy.loginfo("Engel tespit edildi.") 
        return True
    else:
        rospy.loginfo("Engel yok, yol temiz.") 
        return False



def convertNavDataToWaypoints(navData):
    """
    Converts navigation data into a list of waypoint Point objects relative to the home position.

    Args:
        navData (dict): A dictionary containing navigation data with the following keys:
            - "tiles": A list of coordinate pairs (x, y) representing tile positions.
            - "home": A coordinate pair (x, y) representing the home position.
            - "height": A numeric value representing the height for all waypoints.

    Returns:
        list: A list of Point objects, each representing a waypoint relative to the home position at the specified height.
    """

    waypoints = list()
    for pt in navData["tiles"]:
        point = Point(pt["x"] - navData["home"]["x"], pt["y"] - navData["home"]["y"], navData["height"])
        waypoints.append(point)
    return waypoints


def simpleNavigation(waypoints: list, image=False):
    wpNum = len(waypoints)
    for i, wp in enumerate(waypoints):
        rospy.loginfo(f"Moving to waypoint #{i}/{wpNum}...")

        current_pose = get_current_pose()
        if current_pose is None:
            rospy.logerr("Current pose alınamadı, hareket durduruldu.")
            break

        adjusted_wp = adjust_goal_if_blocked(current_pose.position, wp)
        stepMove(adjusted_wp)

        if image:
            rospy.sleep(3)
            takePhoto()


def avoid_obstacle_and_move(target_point: Point):
    """
    Belirtilen hedefe doğru giderken engellerden kaçınarak ilerler.
    Her adımda çevre kontrol edilir ve gerekirse yön değiştirilir.
    """
    rospy.loginfo("Engelden kaçınmalı hareket başlatıldı...")
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Mevcut konum alınamadı. İşlem iptal.")
        return

    path = plan_path_with_avoidance(current_pose.position, target_point)

    if path is None:
        rospy.logwarn("Engelden kaçınarak gidecek bir yol bulunamadı.")
        return

    rospy.loginfo(f"{len(path)} adımlı kaçınmalı yol bulundu. Hareket ediliyor...")
    for point in path:
        simpleMove(point)

    rospy.loginfo("Hedefe başarıyla ulaşıldı.")



# Subscribe to the topic '/firefly/image_raw' and save the RGB image into 'images' dir.
# With the drone position (x,y,z) as file name.
def takePhoto(filename=None, front=False):
    bridge = CvBridge()
    latest_image = {"image": None}
    latest_pose = get_current_pose()

    if latest_pose is None:
        rospy.logerr("Current pose unavailable. Cannot take photo.")
        return

    def image_callback(msg):
        latest_image["image"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    if not front:
        sub = rospy.Subscriber('/firefly/image_raw', Image, image_callback)
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout
    else:
        sub = rospy.Subscriber('/firefly/vi_sensor/camera_depth/camera/image_raw', Image, image_callback)
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_image["image"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    sub.unregister()

    if latest_image["image"] is None:
        rospy.logwarn("Could not capture image.")
        return
    
    # Enhance brightness and vibrancy
    # latest_image = cv2.convertScaleAbs(latest_image["image"], alpha=1.2, beta=30)

    # Save the image with the drone's position as the filename
    position = latest_pose.position
    if not filename:
        filename = f"images/{position.x:.2f}_{position.y:.2f}_{position.z:.2f}.png"
    else:
        filename = f"images/{filename}.png"

    os.makedirs("images", exist_ok=True)
    cv2.imwrite(filename, latest_image["image"])
    rospy.loginfo(f"Photo saved as {filename}")

# Remove all the images in the 'images' folder
def resetPhotos():
    folder = "images"
    if not os.path.exists(folder):
        rospy.loginfo(f"No images to delete. Folder '{folder}' does not exist.")
        return

    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
                rospy.loginfo(f"Deleted {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {file_path}: {e}")
    rospy.loginfo("All images have been deleted.")

def resetOutput():
    folder = "output"
    if not os.path.exists(folder):
        rospy.loginfo(f"No output images to delete. Folder '{folder}' does not exist.")
        return

    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
                rospy.loginfo(f"Deleted {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {file_path}: {e}")
    rospy.loginfo("All output images have been deleted.")

# Run inference on all the images and return their global coordinates
def inferenceImages():
    folder = "images"
    if not os.path.exists(folder):
        rospy.loginfo(f"No images to delete. Folder '{folder}' does not exist.")
        return 

    coords = set()
    def is_far_enough(new_coord):
        for existing_coord in coords:
            distance = math.sqrt(
                (new_coord[0] - existing_coord[0]) ** 2 +
                (new_coord[1] - existing_coord[1]) ** 2
            )
            if distance < VEHICLE_EPS:
                return False
        return True

    for filename in os.listdir(folder):
        print(filename)
        for data in detect.inference_with_absolute_coordinates(filename):
            if data["score"] > INFERENCE_THRESHOLD:
                new_coord = (
                    round(data["x_global"]), 
                    round(data["y_global"]), 
                    INFERENCE_Z
                )
                if is_far_enough(new_coord):
                    coords.add(new_coord)

                print(f'\t{data["x_global"]}, {data["y_global"]}')

    for p in coords:
        print(f"({p[0]}, {p[1]}, {p[2]})")
    print(f"LEN: {len(coords)}")

    return coords

def rotateDrone(degrees):
    """
    Rotates the drone in place by 'degrees' (relative to its current heading).
    No persistence in future commands (once complete, orientation remains changed).
    """
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Cannot rotate.")
        return

    # Convert current orientation to Euler angles (roll, pitch, yaw)
    q = (current_pose.orientation.x,
         current_pose.orientation.y,
         current_pose.orientation.z,
         current_pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(q)

    # Calculate new orientation
    new_yaw = yaw + math.radians(degrees)
    new_q = quaternion_from_euler(roll, pitch, new_yaw)

    if new_yaw >= math.radians(360):
        new_yaw -= math.radians(360)

    # Build a PoseStamped with same position but updated orientation
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position = current_pose.position
    pose_stamped.pose.orientation.x = new_q[0]
    pose_stamped.pose.orientation.y = new_q[1]
    pose_stamped.pose.orientation.z = new_q[2]
    pose_stamped.pose.orientation.w = new_q[3]

    # Publish until the drone is close to the desired yaw
    rate = rospy.Rate(10)
    arrived = False
    rospy.loginfo(f"Rotating by {degrees} degrees...")

    while not rospy.is_shutdown() and not arrived:
        pub.publish(pose_stamped)
        updated_pose = get_current_pose()
        if updated_pose:
            uq = (updated_pose.orientation.x,
                  updated_pose.orientation.y,
                  updated_pose.orientation.z,
                  updated_pose.orientation.w)
            _, _, current_yaw = euler_from_quaternion(uq)
            print(current_yaw, new_yaw)
            if abs(current_yaw - new_yaw) < 0.05:  # Radians threshold
                arrived = True
        rate.sleep()

    rospy.loginfo("Drone rotation complete.")


def stepRotateDrone(degrees, stepsize=90):
    
    if degrees < 0:
        stepsize *= -1

    if -90 <= degrees <= 90:
        rotateDrone(degrees)
    else:
        times_step = int(degrees // stepsize)
        angles = [stepsize]*times_step + [degrees - stepsize*times_step]
        for x in angles:
            rotateDrone(x)


# Fly over the car (for example (10,-8))
# Rotate around it, maybe with an altitude of 10m or less
# Get photos of the front and back
# Then read the plates
def readCarPlate(car_x, car_y, car_angle, save=False, behind=3, alt=10, index=-1):
    """
    Move the drone behind the car using car_x, car_y, and car_angle.
    A placeholder for reading the plate is returned.
    """
    altitude = alt
    behind_offset = behind

    deg_angle = math.degrees(car_angle) - 90
    behind_x = car_x - behind_offset * math.sin(car_angle)
    behind_y = car_y - behind_offset * math.cos(car_angle)
    # behind_x = car_x
    # behind_y = car_y
    rospy.loginfo(f"Moving behind ({behind}) the car at ({behind_x}, {behind_y}) with angle {car_angle}.")
    stepMove(Point(behind_x, behind_y, altitude))
    stepMove(Point(behind_x, behind_y, 0.7))
    stepRotateDrone(-1*deg_angle)

    # Try plate recognizing
    if PLATE:
        takePhoto("plate", front=True)
        plateText = plate.plateRecognizerDetect("images/plate.png")
    else:
        plateText = "DISABLED"

    if save:
        with open("plates.txt", "a") as f:
            f.write(f"{index},{plateText.upper()}\n")

    # Reset rotation
    stepRotateDrone(deg_angle)
    stepMove(Point(behind_x, behind_y, altitude))

    return plateText


def readPlates(abs_coords):
    """
    Reads plates starting from the closest coordinate to the drone's current position.

    Args:
        abs_coords (list): A list of dictionaries containing absolute coordinates with keys:
            - "x_global": X-coordinate of the detection.
            - "y_global": Y-coordinate of the detection.
            - "angle": Angle of the detection.
    """
    indexed_abs_coords = [(i, detection) for i, detection in enumerate(abs_coords)]
    while indexed_abs_coords:
        current_pose = get_current_pose()
        if current_pose is None:
            rospy.logerr("Current pose unavailable. Cannot determine closest coordinate.")
            return

        # Calculate distances and find the closest coordinate
        current_x, current_y = current_pose.position.x, current_pose.position.y
        i, closest = min(
            indexed_abs_coords,
            key=lambda detection: math.sqrt(
                (detection[1]["x_global"] - current_x) ** 2 +
                (detection[1]["y_global"] - current_y) ** 2
            )
        )

        # Process the closest coordinate
        readCarPlate(
            closest["x_global"],
            closest["y_global"],
            closest["angle"],
            save=True,
            behind = closest["w_global"] + 1.4,
            index=i
        )

        # Remove the processed coordinate from the list
        indexed_abs_coords.remove((i, closest))


def resetPlates():
    """
    Deletes the plates.txt file if it exists.
    """
    filename = "plates.txt"
    if os.path.exists(filename):
        try:
            os.remove(filename)
            rospy.loginfo(f"Deleted {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {filename}: {e}")
    else:
        rospy.loginfo(f"No plates.txt file to delete.")


if __name__ == "__main__":
    rospy.init_node("uav_navigation_test", anonymous=True)

    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Başlangıç pozisyonu alınamadı.")
        exit()

    # Sabit hedef: x ekseninde 5 metre ilerisi
    final_goal = Point(
        current_pose.position.x + 5.0,
        current_pose.position.y,
        current_pose.position.z
    )

    rospy.loginfo(f"Test başlatılıyor. Hedef: ({final_goal.x:.2f}, {final_goal.y:.2f}, {final_goal.z:.2f})")

    avoid_obstacle_and_move(final_goal)




    # resetPlates()
    # resetPhotos()
    # MAX_DIST = 10.0
    # simpleNavigation([Point(0, 0, 80)], image=True)
    # simpleNavigation([Point(0, 0, 10)], image=False)
    # MAX_DIST = 1.0
    # # detect.init()

    # for f in os.listdir("images"):
    #     detect.draw_detections_on_image(f, f"output/det_{f}")
    #     abs_coords = detect.inference_with_absolute_coordinates(f, remove_repetations=True)

    # # for detection in abs_coords:
    # #     readCarPlate(
    # #         detection["x_global"], 
    # #         detection["y_global"], 
    # #         detection["angle"],
    # #         save=True)

    # # simpleNavigation([Point(-16, -4, 20)])
    # # readCarPlate(-14.6, -1.74, 1.51, save=True)

    # readPlates(abs_coords)

# {'x_global': 24.170509596565196, 'y_global': 6.748032890355546, 'altitude': 79.8, 'angle': 0.5816325, 'score': 0.8759265, 'w_global': 1.9124779614028897, 'h_global': 0.8567711158815933}
    # readCarPlate(24, 6.7, 0.581, save=False, behind=0, alt=5)
