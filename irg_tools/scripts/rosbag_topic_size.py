#!/usr/bin/env python3
# Utility to show cumulative size of all topics

import sys

import rosbag2_py


def sortSecond(val):
    return val[1]


def main():
    if (len(sys.argv) != 2):
        print('Usage: rosbag_topic_size.py <rosbag-dir>')
        return

    topic_size_dict = {}

    bag_path = sys.argv[1]
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')

    serialization_format = 'cdr'
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_counter = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(data)
        msg_counter += 1

    topic_size = list(topic_size_dict.items())
    topic_size.sort(key=sortSecond, reverse=True)
    for topic, size in topic_size:
        print('{:12d} {}'.format(size, topic))


if __name__ == '__main__':
    main()
