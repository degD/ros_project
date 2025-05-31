import bpy
import random
import string

def generate_plate_number():
    format_type = random.choices(["3L3D", "4L2D"], weights=[0.8, 0.2])[0]
    city = str(random.randint(1, 81)).zfill(2)
    valid_letters = [l for l in string.ascii_uppercase if l not in 'QWXOI']

    if format_type == "3L3D":
        letters = ''.join(random.choices(valid_letters, k=3))
        digits = str(random.randint(1, 999)).zfill(3)
    else:
        letters = ''.join(random.choices(valid_letters, k=4))
        digits = str(random.randint(1, 99)).zfill(2)
    
    return f"{city} {letters} {digits}"

def create_plate_instance():
    # Duplicate the plate mesh
    plate = bpy.data.objects['LicensePlate'].copy()
    plate.data = bpy.data.objects['LicensePlate'].data.copy()
    plate.name = f"LicensePlate_{random.randint(1000,9999)}"
    bpy.context.collection.objects.link(plate)

    # Duplicate the text
    text = bpy.data.objects['LicensePlateText'].copy()
    text.data = bpy.data.objects['LicensePlateText'].data.copy()
    text.name = f"PlateText_{plate.name}"
    bpy.context.collection.objects.link(text)

    # Generate new number and assign
    plate_number = generate_plate_number()
    text.data.body = plate_number

    # Parent the text to the plate
    text.parent = plate
    text.matrix_parent_inverse = plate.matrix_world.inverted()

    print(f"[+] Created plate: {plate_number}")

# Call it to make one plate
create_plate_instance()