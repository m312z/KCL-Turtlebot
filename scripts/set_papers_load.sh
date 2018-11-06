#!/bin/bash
rostopic pub /papers_loaded std_msgs/Bool "data: $1" -1
