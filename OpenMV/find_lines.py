import sensor
import image
import time
import pyb

ENABLE_LENS_CORR = False  # turn on for straighter lines...

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # Grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

target_theta = 90  # Assuming we want the line to be vertical
some_threshold = 10


# Initialize UART
uart = pyb.UART(3, 115200)


while True:
    clock.tick()
    img = sensor.snapshot()
    if ENABLE_LENS_CORR:
        img.lens_corr(1.8)  # Correction for lens distortion

    lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)
    if lines:
        # Sort lines based on the magnitude of the rho value
        lines.sort(key=lambda x: abs(x.rho()))
        strongest_line = lines[0]  # Select the strongest line

        detected_theta = strongest_line.theta()
        error = detected_theta - target_theta  # Calculate the angular error
        if abs(error) < some_threshold:
            img.draw_line(strongest_line.line(), color=(0, 255, 0))
            print("Angle: %d, Error: %d" % (detected_theta, error))
        else:
            img.draw_line(strongest_line.line(), color=(255, 0, 0))
            print("Angle: %d, Error: %d - OUT OF BOUNDS" % (detected_theta, error))
#        sent the error value to arduino
        uart.write("%d\n" % error)
    print("FPS %f" % clock.fps())
