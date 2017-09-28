#!/usr/bin/env python
"""Publish video files to sensor_msgs::Image 
"""
__author__ =  'Tommi Tikkanen <tommi.tikkanen at gimltd.fi>'
__version__=  '0.1'
# Python libs
import sys, time, os
# numpy and scipy
import numpy as np

import datetime
import struct

from progress.bar import Bar

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import rosbag

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

    if len(args) < 3:
        print "Usage: python video_stream_opencv video_to_bag.py input_video_folder output_bag_folder"

    video_folder = args[1]
    output_bag_folder = args[2]

    image_topic = '/camera/image_raw'
    if len(args) > 3:
        image_topic = args[3]
    
    for directory, subdirectories, files in os.walk(video_folder):
        for file in files:
            full_path = os.path.join(directory, file)

            if not "mp4" in file:
                continue


            # load video file
            video_stream_provider = full_path
            print "Found video: %s" % video_stream_provider
            cap = cv2.VideoCapture(video_stream_provider)
            
            print "Extracting video metadata..."
            video_end_date = get_video_end_time(video_stream_provider)
            video_end_time = 0
            if video_end_date:
                video_end_time = get_unix_timestamp(video_end_date)

            video_frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            video_frame_rate = cap.get(cv2.CAP_PROP_FPS)
            video_duration = video_frame_count / video_frame_rate
            video_start_time = video_end_time - video_duration
            video_start_date = video_end_date - datetime.timedelta(0,video_duration)
            print "Video duration: %2f minutes" % (video_duration/60)
            print " - start : %s" % video_start_date
            print " - end   : %s" % video_end_date

            # init output
            bagfile = rosbag.Bag(output_bag_folder + "/" + file + ".bag", 'w')

            # Progress bar
            bar = Bar('Converting ' + file, max=video_frame_count)

            # iterate all video frames
            frame_index = 0
            while frame_index < video_frame_count:
                
                if rospy.is_shutdown():
                    exit(1)

                try:            
                    current_frame_time = video_start_time + frame_index*(1.0/video_frame_rate) # video_progress

                    # retrieve image
                    meta, image = cap.read()

                    frame_index += 1
                    bar.next()

                    #### Create CompressedIamge ####
                    msg = CompressedImage()
                    msg.header.stamp = rospy.Time(current_frame_time)
                    msg.header.frame_id = "camera"
                    msg.header.seq = frame_index
                    msg.format = "jpeg"
                    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

                    # Write frame to bag
                    bagfile.write(image_topic, msg)

                except KeyboardInterrupt:
                    print "Shutting down video streamer"
                    break

            bagfile.close()
            bar.finish()
        # end file iteration
    # end directory iteration
# end main

if __name__ == '__main__':
    main(sys.argv)