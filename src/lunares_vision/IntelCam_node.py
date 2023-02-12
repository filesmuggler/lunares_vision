import argparse
import logging as lg

from IntelCam import IntelCam

def run_cameras_node(**args):
    args = args['args']
    num_cams = len(args.serial_num)

    if num_cams>1:
        print("Starting the node for ",num_cams," cameras with serial numbers ",args.serial_num)


if __name__ == '__main__':
    lg.basicConfig(level=lg.INFO)
    lg.info("Running Intel camera node...")
    parser = argparse.ArgumentParser()
    parser.add_argument("-vv", "--version", help="show program version", action="store_true")
    parser.add_argument("-tw", "--width", help="image width")
    parser.add_argument("-th", "--height", help="image height")
    parser.add_argument("-tk", "--tick", help="frequency of topic")
    parser.add_argument('-sn','--serial-num', action="append", help='<Required> camera serial number', required=True)
    

    args = parser.parse_args()

    run_cameras_node(args=args)




