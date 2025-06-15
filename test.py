from PIL import Image, ImageEnhance
import numpy as np
import matplotlib.pyplot as plt

def apply_s_curve(img, amount=0.5):
    """
    Applies an S-curve contrast enhancement to the image.
    `amount` controls the strength of the effect (0-1).
    """
    img_np = np.array(img).astype(np.float32) / 255.0

    # S-curve: y = 3x^2 - 2x^3
    s_curve = lambda x: (1 - amount) * x + amount * (3 * x ** 2 - 2 * x ** 3)
    img_np = s_curve(img_np)

    img_np = np.clip(img_np * 255, 0, 255).astype(np.uint8)
    return Image.fromarray(img_np)

# --- Load your image here ---
image = Image.open("images/0.11_-0.00_79.91.png")

# 1. Brightness boost
brightness_enhancer = ImageEnhance.Brightness(image)
bright_img = brightness_enhancer.enhance(1.4)  # Try 1.2–1.5

# 2. S-curve contrast
final_img = apply_s_curve(bright_img, amount=0.5)

# Save or show
final_img.save("0.11_-0.00_79.90_scurve_bright.png")