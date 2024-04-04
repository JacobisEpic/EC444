import io
import time
from picamera2 import Picamera2
from PIL import Image
from pyzbar import pyzbar

def find_qr_code(image):
    """Detect QR codes in the given image."""
    decoded_objects = pyzbar.decode(image)
    return decoded_objects

def capture_and_process_frames(picam2):
    while True:
        image_array = picam2.capture_array()
        image = Image.fromarray(image_array).convert('L')

        qr_codes = find_qr_code(image)
        if qr_codes:
            for qr_code in qr_codes:
                url = qr_code.data.decode("utf-8")
                print("QR Code detected, URL:", url)
            break
        else:
            print("No QR code detected. Trying again...")

        time.sleep(1/30)

picam2 = Picamera2()

capture_config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(capture_config)

picam2.start()

try:
    capture_and_process_frames(picam2)
finally:
    picam2.close()
