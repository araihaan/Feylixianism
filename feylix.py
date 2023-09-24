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
import ctypes
from modules import *

# Settings
COM_PORT = "COM12"  # The COM port number for your Arduino. This can be found in the Device Manager.
X_FOV = 70  # Field of view for the X-axis.
Y_FOV = 70  # Field of view for the Y-axis.
AIM_KEY = 0x01  # Key code for aim action. See https://t.ly/qtrot for full key codes.
XYSPEED = 3
recoilY = 3
LOWER_COLOR = [140, 120, 180]
UPPER_COLOR = [160, 200, 255]
camera = dxcam.create(output_idx=0, output_color="BGR")  # Initialize the camera with settings

class Feylix:
    def __init__(self):
        self.loadConfig()
        
    def listen(self):
        clicked = False
        while True:
            if win32api.GetAsyncKeyState(AIM_KEY) < 0:
                self.run("aim")
            try:
                key = varManager.getVar("key", "TRIGGER")
                fovX = varManager.getVar("fovx", "TRIGGER")
                fovY = varManager.getVar("fovy", "TRIGGER")
                mode = varManager.getVar("mode", "TRIGGER")
                delay = int(varManager.getVar("shootdelay", "TRIGGER"))

                if mode == "Toggle" and keyPressed(key):
                    clicked = not clicked

                    if clicked:
                        beepSound(440, 75)
                        beepSound(700, 100)
                        print("ON")
                    else:
                        beepSound(440, 75)
                        beepSound(200, 100)
                        print("OFF")

                    while keyPressed(key):
                        pass

                elif ((mode == "Toggle" and clicked) or mode == "Holding" and keyPressed(key)) and not any([keyPressed("W"), keyPressed("A"), keyPressed("S"), keyPressed("D")]):
                    self.run("aim")
                    enemy = uiPasteFunctions.findEnemyTrigger(int(fovX), int(fovY))
                    if enemy:
                        Feylix.shoot()
                        time.sleep(200 * delay / 1000)

                time.sleep(0.005)
            except Exception as e:
                print(e)
                pass
                
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
            
        
    def loadConfig(self):
        self.config = configManager.loadConfig()
        
    @staticmethod
    def shoot():
        ctypes.windll.user32.mouse_event(0x0002, 0, 0, 0, 0)  # L DOWN
        time.sleep(0.01)
        ctypes.windll.user32.mouse_event(0x0004, 0, 0, 0, 0)  # L UP
    
                    
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

    def move(self, x, y):
        self.serial_port.write(f'{float(x)},{float(y)}\n'.encode())

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