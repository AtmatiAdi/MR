from mr_controller import RobotController
#from monit import MapPlotter
import json

if __name__ == "__main__":
    json_data = open('map_boxes_1.json')
    data = json.load(json_data)
    scan = data[2]['scan']
 
    ctrl = RobotController(150, 4)
    ctrl.init_map(scan)
    ctrl.start()