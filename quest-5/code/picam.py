# Code contributers: Celine Chen, Eric Chen, Jacob Chin, Nuo Lin
import io
import time
import socket
from picamera2 import Picamera2
from PIL import Image
from pyzbar import pyzbar

# finds the qr code using the picam
def find_qr_code(image):
    """Detect QR codes in the given image."""
    decoded_objects = pyzbar.decode(image)
    return decoded_objects
def send_udp_message(message, ip, port):
    """Send a UDP message to the specified IP address and port."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message.encode(), (ip, port))
# detects the scanned qr code and breaks it down the the Fob ID and meter ID
def capture_and_process_frames(picam2):
    while True:
        image_array = picam2.capture_array()
        image = Image.fromarray(image_array).convert('L')
        qr_codes = find_qr_code(image)
        if qr_codes:
            print("QR Code detected")
            send_udp_message("[1,1]", '192.168.1.10', 3337)
            time.sleep(5)
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
