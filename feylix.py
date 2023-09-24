import cv2
import numpy as np
import pyautogui
import win32api
import serial
import dxcam
from colorama import Fore, Style
from time import sleep, time
import random
import time

# Settings
COM_PORT = "COM12"  # The COM port number for your Arduino. This can be found in the Device Manager.
X_FOV = 70  # Field of view for the X-axis.
Y_FOV = 70  # Field of view for the Y-axis.
AIM_KEY = 0x01  # Key code for aim action. See https://t.ly/qtrot for full key codes.
TRIGGER_KEY = 0x12  # Key code for trigger action. See https://t.ly/qtrot for full key codes.
XYSPEED = 3
recoilY = 3
LOWER_COLOR = [140, 120, 180]
UPPER_COLOR = [160, 200, 255]
camera = dxcam.create(output_idx=0, output_color="BGR")  # Initialize the camera with settings

class Feylix:
    def listen(self):
        while True:
            if win32api.GetAsyncKeyState(AIM_KEY) < 0:
                self.run("aim")
            if win32api.GetAsyncKeyState(TRIGGER_KEY) < 0:
                self.run("click")
                
    def run(self, action):
        hsv = cv2.cvtColor(Capture().get_screen(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(LOWER_COLOR), np.array(UPPER_COLOR))
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(mask, kernel, iterations=5)
        thresh = cv2.threshold(dilated, 60, 255, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if contours:
            screen_center = (X_FOV // 2, Y_FOV // 2)
            min_distance = float('inf')
            closest_contour = None

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)
                distance = ((center[0] - screen_center[0]) ** 2 + (center[1] - screen_center[1]) ** 2) ** 0.5

                if distance < min_distance:
                    min_distance = distance
                    closest_contour = contour

            x, y, w, h = cv2.boundingRect(closest_contour)
            center = (x + w // 2, y + h // 2)
            cX = center[0]
            cY = y + (h * 0.3)
            cYcenter = center[1] - Y_FOV // 2
            x_diff = cX - X_FOV // 2
            y_diff = cY - Y_FOV // 2
            
            ema_x = 0.0
            ema_y = 0.0
            alpha = 0.2  # Koefisien smoothing (biasanya antara 0.1 hingga 0.3)
            ema_x = (1 - alpha) * ema_x + alpha * x_diff
            ema_y = (1 - alpha) * ema_y + alpha * y_diff

            if action == "aim":
                Mouse().move(ema_x * XYSPEED, ema_y * XYSPEED + recoilY)
            
            if action == "click":
                reaction_time = random.uniform(0.075, 0.125)
                sleep(reaction_time)
                if abs(x_diff) <= 3 and abs(y_diff) <= 7:  # Dirty implementation of a trigger bot. This may be refined later.
                    Mouse().click()
        
    def getPt(self, n1, n2, perc):
        return int(n1 + (n2 - n1) * perc)

    def getRnd(self, min_val, max_val):
        return random.randint(min_val, max_val - 1)

    def MoveGaussian(self, targetX, targetY):
        pInicial = {'x': 0, 'y': 0}
        pLastMove = {'x': 0, 'y': 0}
        pMedio1 = {'x': 0, 'y': 0}
        pMedio2 = {'x': 0, 'y': 0}
        pFinal = {'x': targetX, 'y': targetY}

        if (pFinal['x'] >= pInicial['x'] and pFinal['y'] >= pInicial['y']) or (pFinal['x'] <= pInicial['x'] and pFinal['y'] <= pInicial['y']):
            pMedio1['x'] = pInicial['x'] + (pFinal['x'] - pInicial['x']) / self.getRnd(4, 8)
            pMedio1['y'] = pInicial['y'] + (pFinal['x'] - pInicial['x']) / self.getRnd(4, 8)

            pMedio2['x'] = pInicial['x'] + (pFinal['x'] - pInicial['x']) / self.getRnd(3, 6)
            pMedio2['y'] = pInicial['y'] + (pFinal['x'] - pInicial['x']) / self.getRnd(3, 6)
        else:
            pMedio1['x'] = pInicial['x'] + (pFinal['x'] - pInicial['x']) / self.getRnd(4, 8)
            pMedio1['y'] = pInicial['y'] - (pFinal['x'] - pInicial['x']) / self.getRnd(4, 8)

            pMedio2['x'] = pInicial['x'] + (pFinal['x'] - pInicial['x']) / self.getRnd(3, 5)
            pMedio2['y'] = pInicial['y'] - (pFinal['x'] - pInicial['x']) / self.getRnd(3, 5)

        total = {'x': 0, 'y': 0}

        for i in range(10):
            i /= 9.0
            xa = self.getPt(pInicial['x'], pMedio1['x'], i)
            ya = self.getPt(pInicial['y'], pMedio1['y'], i)
            xb = self.getPt(pMedio1['x'], pMedio2['x'], i)
            yb = self.getPt(pMedio1['y'], pMedio2['y'], i)
            xc = self.getPt(pMedio2['x'], pFinal['x'], i)
            yc = self.getPt(pMedio2['y'], pFinal['y'], i)

            xm = self.getPt(xa, xb, i)
            ym = self.getPt(ya, yb, i)
            xn = self.getPt(xb, xc, i)
            yn = self.getPt(yb, yc, i)

            x = self.getPt(xm, xn, i)
            y = self.getPt(ym, yn, i)

            move = {'x': x - pLastMove['x'], 'y': y - pLastMove['y']}  # Relative movement

            Mouse().move(move['x'], move['y'])

            pLastMove['x'] = x
            pLastMove['y'] = y
                    
class Mouse:
    def __init__(self):
        self.serial_port = serial.Serial()
        self.serial_port.baudrate = 115200
        self.serial_port.timeout = 0
        self.serial_port.port = COM_PORT
        try:
            self.serial_port.open()
        except serial.SerialException:
            print(f"{Fore.RED}[ERROR]{Style.RESET_ALL} Failed to connect because the specified COM port was not found.")
            sleep(10)
        
        # self.acceleration = 0.1  # Adjust the acceleration factor as needed
        # self.deceleration = 0.05  # Adjust the deceleration factor as needed
        # self.current_speed_x = 0
        # self.current_speed_y = 0

    def move(self, x, y):
        # Gradually change the current speed towards the desired speed
        # if x != 0:
        #     x_sign = x / abs(x)
        #     self.current_speed_x += (x - self.current_speed_x) * self.acceleration
        #     self.current_speed_x *= self.deceleration if x_sign != self.current_speed_x / abs(self.current_speed_x) else 1.0
        # else:
        #     self.current_speed_x *= self.deceleration

        # if y != 0:
        #     y_sign = y / abs(y)
        #     self.current_speed_y += (y - self.current_speed_y) * self.acceleration
        #     self.current_speed_y *= self.deceleration if y_sign != self.current_speed_y / abs(self.current_speed_y) else 1.0
        # else:
        #     self.current_speed_y *= self.deceleration

        # Send the updated speed to the mouse
        # self.serial_port.write(f'{float(self.current_speed_x)},{float(self.current_speed_y)}\n'.encode())
        # print (float(self.current_speed_x), float(self.current_speed_y))
        # Add a small delay to control the speed of mouse movement
        #sleep(0.001)  # Use the 'sleep' function from the 'time' module
        self.serial_port.write(f'{float(x)},{float(y)}\n'.encode())

    def click(self):
       self.serial_port.write('CLICK\n'.encode())

class Capture:
    def __init__(self):
        monitor_size = pyautogui.size()
        self.region = self.calculate_region(monitor_size)

    def calculate_region(self, monitor_size):
        x_center = monitor_size.width // 2
        y_center = monitor_size.height // 2
        left = x_center - X_FOV // 2
        top = y_center - Y_FOV // 2
        right = left + X_FOV
        bottom = top + Y_FOV
        return left, top, right, bottom

    def get_screen(self):
        while True:
            screenshot = camera.grab(region=self.region)
            if screenshot is not None:
                return np.array(screenshot)

if __name__ == "__main__":
    Feylix().listen()