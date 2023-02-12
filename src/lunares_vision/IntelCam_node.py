import argparse
import logging as lg

from IntelCam import IntelCam

def run_cameras_node(**args):
    args = args['args']
    num_cams = len(args.serial_num)

    if num_cams>1:
        print("Starting the node for ",num_cams," cameras with serial numbers ",args.serial_num)
    elif num_cams==1:
        print("Starting the node for ",num_cams," camera with serial number ",args.serial_num)
    else:
        print("No camera serial number specified. Aborting...")
        exit()

    ic = IntelCam(args)
    ic.run()
    


if __name__ == '__main__':
    lg.basicConfig(level=lg.INFO)
    lg.info("Running Intel camera node...")
    parser = argparse.ArgumentParser()
    parser.add_argument("-vv", "--version", help="show program version", action="store_true")
    parser.add_argument("-tw", "--width", default=640, help="image width")
    parser.add_argument("-th", "--height", default=480, help="image height")
    parser.add_argument("-tk", "--tick", default=30, help="frequency of topic")
    parser.add_argument("-c", "--color", help="enable color", default=True, action="store_true")
    parser.add_argument("-d", "--depth", help="enable depth", default=True, action="store_true")
    parser.add_argument("-a", "--align", help="align rgb and depth", default=True, action="store_true")
    parser.add_argument('-sn','--serial-num', action="append", help='<Required> camera serial number', required=True)

    args = parser.parse_args()

    run_cameras_node(args=args)




