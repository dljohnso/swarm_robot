from robot_controller2 import RobotController
import time

def driver(sensor_input):
    time.sleep(0.1)
    return (5, 5)
 
for j in range(2):  
    for i in range(1):
        r = RobotController(True)
        r.run(driver, 12)

