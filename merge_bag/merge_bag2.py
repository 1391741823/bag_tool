#!/usr/bin/env python

import sys
import argparse
from fnmatch import fnmatchcase
from rosbag import Bag
import rospy  # 修复：导入rospy模块

def main():
    parser = argparse.ArgumentParser(description='Merge one or more bag files with the possibilities of filtering topics.')
    parser.add_argument('outputbag',
                        help='output bag file with topics merged')
    parser.add_argument('inputbag', nargs='+',
                        help='input bag files')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    parser.add_argument('-t', '--topics', default="*",
                        help='string interpreted as a list of topics (wildcards \'*\' and \'?\' allowed) to include in the merged bag file')

    args = parser.parse_args()

    topics = args.topics.split(' ')

    total_included_count = 0
    total_skipped_count = 0

    if args.verbose:
        print("Writing bag file: " + args.outputbag)
        print("Matching topics against patterns: '%s'" % ' '.join(topics))

    # Set the global start and end times
    global_start_time = 1664265583.0300949  # Start time of Retail_Street.bag
    global_end_time = 1664265718.5003471    # End time of Retail_Street.bag

    if args.verbose:
        print("Global start time: %f, Global end time: %f" % (global_start_time, global_end_time))

    # Write the output bag file
    with Bag(args.outputbag, 'w') as o:
        for ifile in args.inputbag:
            matchedtopics = []
            included_count = 0
            skipped_count = 0
            if args.verbose:
                print("> Reading bag file: " + ifile)
            with Bag(ifile, 'r') as ib:
                for topic, msg, t in ib:
                    # Adjust timestamps to fit within the global duration
                    adjusted_time = t.to_sec() - ib.get_start_time() + global_start_time
                    if global_start_time <= adjusted_time <= global_end_time:
                        if any(fnmatchcase(topic, pattern) for pattern in topics):
                            if topic not in matchedtopics:
                                matchedtopics.append(topic)
                                if args.verbose:
                                    print("Including matched topic '%s'" % topic)
                            # Write the message with adjusted timestamp
                            o.write(topic, msg, rospy.Time.from_sec(adjusted_time))
                            included_count += 1
                        else:
                            skipped_count += 1
                    else:
                        if args.verbose:
                            print(f"Skipping message with timestamp {t.to_sec()} outside global range.")
            total_included_count += included_count
            total_skipped_count += skipped_count
            if args.verbose:
                print("< Included %d messages and skipped %d" % (included_count, skipped_count))

    if args.verbose:
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))

if __name__ == "__main__":
    main()