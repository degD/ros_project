from typing import Tuple
from mmdet.apis import init_detector, inference_detector
import mmrotate
import warnings
from pathlib import Path
from PIL import Image, ImageDraw
from math import tan, radians
import numpy as np
import os

warnings.filterwarnings("ignore")

def init():
    global IMAGE_DIR
    global DEVICE
    global CONFIG_FILE
    global MODEL
    global CHECKPOINT_FILE
    global FOV

    IMAGE_DIR = 'images/'
    DEVICE = 'cuda:0'
    CONFIG_FILE = 'oriented_rcnn_r50_fpn_1x_dota_le90.py'
    CHECKPOINT_FILE = 'oriented_rcnn_r50_fpn_1x_dota_le90-6d2b2ce0.pth'
    MODEL = init_detector(CONFIG_FILE, CHECKPOINT_FILE, device=DEVICE)
    FOV = [60, 60]  # Later on get it from the config.json
    # Camera resolution is 1024x1024


def fov_to_ground_size(
    fov_horizontal_deg: float,
    fov_vertical_deg: float,
    altitude_m: float
) -> Tuple[float, float]:
    """
    Convert camera FOV and altitude to ground footprint size in meters.
    """
    fov_h_rad = radians(fov_horizontal_deg)
    fov_v_rad = radians(fov_vertical_deg)

    width_m = 2 * altitude_m * tan(fov_h_rad / 2)
    height_m = 2 * altitude_m * tan(fov_v_rad / 2)

    return width_m, height_m


def get_image_size(path) -> Tuple[int, int]:
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Image not found: {path}")

    with Image.open(path) as im:
        width_px, height_px = im.size  # (width, height)

    return width_px, height_px


def pixel_to_global(
    center: Tuple[float, float],
    vehicle_coord: Tuple[float, float],
    image_size_pixels: Tuple[int, int],
    image_size_meters: Tuple[float, float]
) -> Tuple[float, float]:
    """
    Convert a vehicle's pixel coordinates in an image to global (world) metres.

    Parameters
    ----------
    center : (float, float)
        Global X, Y of the image centre in metres.
    vehicle_coord : (float, float)
        Pixel (x, y) of the vehicle in image coordinates (origin = top-left).
    image_size_pixels : (int, int)
        (width_px, height_px) of the image.
    image_size_meters : (float, float)
        (width_m, height_m) footprint of the image on the ground.

    Returns
    -------
    (float, float)
        Global X, Y of the vehicle in metres.
    """
    x0, y0 = vehicle_coord
    w, h = image_size_pixels

    # Gazebo world coordinates
    gx, gy = h/2-y0, w/2-x0

    # To meters
    wm, hm = image_size_meters
    xm, ym = gx*wm/w, gy*hm/h

    # Shift by center
    cx, cy = center
    return cx+xm, cy+ym


def simple_inference(image_filename: str):
    return inference_detector(MODEL, IMAGE_DIR+image_filename)


def inference_with_absolute_coordinates(image_filename: str, remove_repetations=False):
    result = simple_inference(image_filename)

    # Extract global center and altitude from filename
    image_coords = image_filename.strip(".png").split("_")
    x0 = float(image_coords[0])
    y0 = float(image_coords[1])
    altitude = float(image_coords[2])

    # Get image size in pixels
    img_size_px = get_image_size(IMAGE_DIR + image_filename)

    # Compute image size in meters
    img_size_m = fov_to_ground_size(FOV[0], FOV[1], altitude)

    abs_list = []
    dist = lambda p1, p2: ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

    for class_detections in result:
        for x, y, w, h, angle, score in class_detections:
            # Center in global
            x_g, y_g = pixel_to_global((x0, y0), (x, y), img_size_px, img_size_m)

            # Corners in pixel space (centered at 0,0)
            px_corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]

            # Rotate corners by 'angle'
            cos_a = np.cos(angle)
            sin_a = np.sin(angle)
            px_rot = [(cx*cos_a - cy*sin_a, cx*sin_a + cy*cos_a) for cx, cy in px_corners]

            # Shift to bounding box center
            px_world = [(x + rx, y + ry) for rx, ry in px_rot]

            # Convert each corner to global
            corners_global = [
                pixel_to_global((x0, y0), pt, img_size_px, img_size_m)
                for pt in px_world
            ]

            # Measure global width & height
            w_global = dist(corners_global[0], corners_global[1])
            h_global = dist(corners_global[1], corners_global[2])

            abs_list.append({
                "x_global": x_g,
                "y_global": y_g,
                "altitude": altitude,
                "angle": angle,
                "score": score,
                "w_global": w_global/2,
                "h_global": h_global/2
            })

    if remove_repetations and abs_list:
        threshold = 1.0
        filtered = []
        for det in abs_list:
            too_close = False
            for f in filtered:
                dx = det["x_global"] - f["x_global"]
                dy = det["y_global"] - f["y_global"]
                if (dx ** 2 + dy ** 2) ** 0.5 < threshold:
                    too_close = True
                    break
            if not too_close:
                filtered.append(det)
        abs_list = filtered

    return abs_list


def draw_rotated_box(draw, cx, cy, w, h, angle_rad, outline="red", width=2):
    """Draw a rotated rectangle on a PIL canvas."""
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    dx = w / 2
    dy = h / 2
    corners = np.array([
        [-dx, -dy],
        [ dx, -dy],
        [ dx,  dy],
        [-dx,  dy]
    ])

    rot = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    rotated_corners = np.dot(corners, rot.T) + np.array([cx, cy])
    polygon = [tuple(p) for p in rotated_corners]
    draw.polygon(polygon, outline=outline, width=width)


def draw_image_center(draw, width: int, height: int, color="blue", radius=5):
    """Draw a dot at the center of the image."""
    cx, cy = width // 2, height // 2
    draw.ellipse([(cx - radius, cy - radius), (cx + radius, cy + radius)],
                 fill=color, outline=color)


def draw_detections_on_image(image_filename: str, output_path: str = None, score_threshold: float = 0.4):
    """Draw rotated boxes for detections and mark image center."""
    image_path = Path(IMAGE_DIR) / image_filename
    result = simple_inference(image_filename)

    img = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(img)

    # Mark image center
    w, h = img.size
    draw_image_center(draw, w, h, color="blue", radius=5)

    for class_detections in result:
        for x, y, w_box, h_box, angle, score in class_detections:
            if score >= score_threshold:
                draw_rotated_box(draw, x, y, w_box, h_box, angle, outline="lime", width=3)
                draw.ellipse([(x - 1, y - 1), (x + 1, y + 1)], fill="blue", outline="blue")
    if output_path:
        img.save(output_path)
    else:
        return img


if __name__ == "__main__":
    IMAGE_DIR = ""
    OUTPUT_DIR = "output/"
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # List all image files in the IMAGE_DIR
    # image_files = [f for f in os.listdir(IMAGE_DIR) if f.endswith(".png")]
    image_files = ["0.12_0.00_79.84.png"]
    init()

    for image_file in image_files:
        # Perform inference and draw detection boxes
        # output_path = os.path.join(OUTPUT_DIR, f"det_{image_file}")
        # draw_detections_on_image(image_file, output_path, score_threshold=0.3)
        # print(f"Processed {image_file} -> {output_path}")
        inference_with_absolute_coordinates(image_file)
