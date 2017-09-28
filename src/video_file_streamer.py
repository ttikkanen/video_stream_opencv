#!/usr/bin/env python
"""Publish video files to sensor_msgs::Image 
"""
__author__ =  'Tommi Tikkanen <tommi.tikkanen at gimltd.fi>'
__version__=  '0.1'
# Python libs
import sys, time

# numpy and scipy
import numpy as np

import datetime
import struct

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

# Disgrace to OpenCV for lacking this information
def get_video_end_time(filename):
    ATOM_HEADER_SIZE = 8
    # difference between Unix epoch and QuickTime epoch, in seconds
    EPOCH_ADJUSTER = 2082844800

    # open file and search for moov item
    f = open(filename, "rb")
    while 1:
        atom_header = f.read(ATOM_HEADER_SIZE)
        if atom_header[4:8] == 'moov':
            break
        else:
            atom_size = struct.unpack(">I", atom_header[0:4])[0]
            f.seek(atom_size - 8, 1)

    # found 'moov', look for 'mvhd' and timestamps
    atom_header = f.read(ATOM_HEADER_SIZE)
    if atom_header[4:8] == 'cmov':
        print "moov atom is compressed"
    elif atom_header[4:8] != 'mvhd':
        print "expected to find 'mvhd' header"
    else:
        f.seek(4, 1)
        creation_date = struct.unpack(">I", f.read(4))[0]
        return datetime.datetime.utcfromtimestamp(creation_date - EPOCH_ADJUSTER)
    return None

# Disgrace to python 2.x for lacking this method!
def get_unix_timestamp(dt):
    return (dt - datetime.datetime(1970,1,1)).total_seconds()


# Finally we can do the actual stuff
def main(args):
    rospy.init_node('video_streamer', anonymous=True)

    video_stream_provider = rospy.get_param("~video_stream_provider")
    video_frame_rate = rospy.get_param("~fps", 22)

    

    # Initialize ros publisher
    # topic where we publish
    image_pub = rospy.Publisher("/camera/image_raw/compressed", CompressedImage)
    # self.bridge = CvBridge()

    # load video file
    
    #metadata = get_video_metadata(video_stream_provider)
    #print "Loaded video from %s" % metadata.creation_date
    video_end_date = get_video_end_time(video_stream_provider)
    video_end_time = 0
    if video_end_date:
        video_end_time = get_unix_timestamp(video_end_date)
    cap = cv2.VideoCapture(video_stream_provider)

    frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    video_frame_rate = cap.get(cv2.CAP_PROP_FPS)
    duration = frame_count / video_frame_rate

    video_start_time = video_end_time - duration
    print "Video duration: %2f minutes" % (duration/60)
    print " - start : %2f" % video_start_time
    print " - end   : %2f" % video_end_time


    frame_index = 0
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            time.sleep(0.01)

            # Retrieve image timestamp            
            #current_frame_time = frame_index * 1.0/video_frame_rate + video_start_time
            
#           video_progress = cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
            current_frame_time = video_start_time + frame_index*(1.0/video_frame_rate) # video_progress
            #print "Video progress: %2f s" % (video_progress)

            # Retrieve current time (may be simulated)
            current_time = rospy.Time.now().to_sec()

            # Check if image timestamp is close enough to current time
            # Works now only if Video is ahead of LIDAR data but other way around!!!
            if current_time < current_frame_time:
                sys.stdout.write('_')
                sys.stdout.flush()
                #print "Don't publish yet: %2f" % (current_time - current_frame_time)
                continue

            # retrieve image
            meta, image = cap.read()
            sys.stdout.write('.')
            sys.stdout.flush()
            frame_index += 1

            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time(current_frame_time)
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            # Publish new image
            image_pub.publish(msg)

            
        except KeyboardInterrupt:
            print "Shutting down video streamer"
            break

if __name__ == '__main__':
    main(sys.argv)