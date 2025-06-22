import requests
from dotenv import load_dotenv
import os


load_dotenv()

def plateRecognizerDetect(image):
    try:
        with open(image, 'rb') as fp:
            response = requests.post(
                'https://api.platerecognizer.com/v1/plate-reader/',
                files=dict(upload=fp),
                headers={'Authorization': f"Token {os.getenv('PLATE')}"})

            return response.json()["results"][0]["plate"]
    except:
        return "FAILED"


if __name__ == '__main__':
    print(plateRecognizerDetect('images/plate.png'))