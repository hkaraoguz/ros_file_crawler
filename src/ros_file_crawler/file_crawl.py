#!/usr/bin/env python

# --------------------------------------------------------
# ROS File Crawler Node
# Written by Hakan Karaoguz
# --------------------------------------------------------

import rospy
from ros_file_crawler.srv import *
import json
from os.path import splitext, join
from os import walk


class FileCrawl():

    def __init__(self):
        """
        FileCrawl class
        provides a ros service for getting the full paths and file names
        for a given list of extensions and excluded words in a directory
        """

        rospy.init_node("ros_file_crawler")
        rospy.loginfo("Running ros_file_crawler")

        s = rospy.Service('/ros_file_crawler/crawl_for_files',
                          FileList, self.handle_file_list_request)

        rospy.spin()

    def handle_file_list_request(self, req):
        """
        Receives the file list request
        and returns a json string in a dictionary format
        for each extension given
        """

        filenames = dict()
        filepaths = dict()

        for root, dirs, files in walk(req.mainpath, topdown=False):

            for filename in files:

                if filename.lower().endswith(tuple(req.fileextension)):

                    count = 0
                    suffix = splitext(filename)[1]

                    for word in req.excludedwords:
                        if word and word in filename:
                            count =1
                            break

                    if count == 0:
                        if suffix in filenames.keys():
                            filenames[suffix] +=[filename]
                            filepaths[suffix] += [join(root, filename)]
                        else:
                            filenames[suffix] = [filename]
                            filepaths[suffix] = [join(root, filename)]

                    filenames[suffix].sort()
                    filepaths[suffix].sort()

        namesjson = json.dumps(filenames, ensure_ascii=False)
        pathsjson = json.dumps(filepaths,ensure_ascii=False)

        return FileListResponse(pathsjson,namesjson)
