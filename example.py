from robot_controller import run

def driver(sensor_input):
    print len(sensor_input)
    return (30, 15)
    

run(driver, 5)
