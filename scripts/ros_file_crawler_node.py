#!/usr/bin/env python

from ros_file_crawler.file_crawl import FileCrawl

if __name__ == '__main__':
    try:
        FileCrawl()
    except rospy.ROSInterruptException:
        pass
