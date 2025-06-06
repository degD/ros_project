import requests
from dotenv import load_dotenv
import os


load_dotenv()

def plateRecognizerDetect(image):
    with open(image, 'rb') as fp:
        response = requests.post(
            'https://api.platerecognizer.com/v1/plate-reader/',
            files=dict(upload=fp),
            headers={'Authorization': os.getenv('PLATE')})

    try:
        return response.json()["results"][0]["plate"]
    except:
        return ""


if __name__ == '__main__':
    print(plateRecognizerDetect('plate2.png'))