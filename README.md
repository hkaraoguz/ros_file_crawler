ros_file_crawler
=====
`ros_file_crawler` provides a ros service for getting the full file paths and file names in a given directory. Desired file extensions and excluded words are be provided as string lists.

Requirements
-----
* python natsort package (`sudo pip install natsort`)

Example Usage
-----
```
rosservice call /ros_file_crawler/crawl_for_files "mainpath: '/home/user/data'
fileextension: ['jpg']
excludedwords: []"
```
This call will return all the filenames and absolute paths of `"jpg"` files located under the `/home/user/data` directory (including subdirectories).

```
rosservice call /ros_file_crawler/crawl_for_files "mainpath: '/home/user/data'
fileextension: ['jpg','png']
excludedwords: ['mask']"
```
This call will return all the filenames and absolute paths of `"jpg","png"` files that do not contain `mask` in their names located under the `/home/user/data` directory (including subdirectories). This is particularly useful when the user needs to get one type of data instead of all the data.
