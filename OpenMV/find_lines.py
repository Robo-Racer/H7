import sensor
import image
import time
from pyb import UART
import struct

# Settings
ENABLE_LENS_CORR = False  # Optionally turn on for straighter lines...
TARGET_THETA = 0  # Target line orientation
ERROR_THRESHOLD = 30  # Maximum allowed angular error in degrees
LINE_THICKNESS = 5  # Thickness of the drawn line

# Initialize the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # Using grayscale for faster processing
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

# Initialize UART for Arduino communication
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

while True:
    clock.tick()
    img = sensor.snapshot()
    if ENABLE_LENS_CORR:
        img.lens_corr(1.8)  # Apply lens correction for distortion



        # Apply a threshold to help distinguish the white line from the darker track
#        img.binary([100, 255])  # Adjust the threshold values as needed for your specific lighting conditions



    # Find lines in the image - Adjust threshold for darker lines
    lines = img.find_lines(threshold=800, theta_margin=50, rho_margin=50)  # Lower threshold for black lines
    if lines:
        # Sort lines based on the magnitude of the rho value
        lines.sort(key=lambda x: abs(x.rho()), reverse=True)
        strongest_line = lines[0]  # Take the line with the largest rho

        # Calculate the angular error from the desired orientation
        detected_theta = strongest_line.theta()
        error = detected_theta - TARGET_THETA

        # Check if the detected line is within the acceptable error range
        if abs(error) < ERROR_THRESHOLD:
            img.draw_line(strongest_line.line(), color=(255, 255, 255), thickness=LINE_THICKNESS)
            print("Angle: %d, Error: %d" % (detected_theta, error))
        else:
            img.draw_line(strongest_line.line(), color=(0, 0, 0), thickness=LINE_THICKNESS)
            print("Angle: %d, Error: %d - OUT OF BOUNDS" % (detected_theta, error))

        # Send the error value to Arduino formatted as a string
        uart.write("Error:%d\n" %error)
    # Output frames per second
    print("FPS %f" % clock.fps())
