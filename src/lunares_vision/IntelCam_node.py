import argparse

def run_cameras_node(**args):
    args = args['args']

    print(args)
    print("")


if __name__ == '__main__':
    print("Running...")
    parser = argparse.ArgumentParser()
    parser.add_argument("-vv", "--version", help="show program version", action="store_true")
    parser.add_argument("-tw", "--width", help="image width")
    parser.add_argument("-th", "--height", help="image height")
    parser.add_argument("-tk", "--tick", help="frequency of topic")
    parser.add_argument("-nc","--num-cams",help="<Required> number of cameras used in the experiment", required=True)
    parser.add_argument('-cl','--cam-list', nargs='+', help='<Required> list of camera serial numbers', required=True)
    

    args = parser.parse_args()

    run_cameras_node(args=args)




