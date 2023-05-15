from plane import PlaneModel
from pyshow.pyshow import RenderApi
import math, time
def main():
    config = {
        "range_x": [-50000, 50000],
        "range_y": [-50000, 50000],
        "display_size": [800, 800],
    }
    shower = RenderApi(config)
    shower.init()
    plane = PlaneModel(None)
    plane.target_points = [[-28000, -28000], [-28000, 28000], [28000, 28000], [28000, -28000]] * 20
    
    for i in range(30 * 60 * 60):
        print(f"this is time {i}")
        for ms in range(50):
            # plane.target_course = plane.course + math.pi/2
            # plane.target_course %= 2 * math.pi 
            info, traces = plane.advance(20)
            tsinfo = {
                "name": "target",
                "position":  plane.target_point if  plane.target_point else [0, 0],
                'icon': 'dead',
                'side': 'red',
                "iconsize": 36, 
                'course': 0,
            }
        shower.update({"units": [info]+traces+ [tsinfo]})
        # print(info)
if __name__ == '__main__':
    main()