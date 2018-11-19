#!/bin/bash
rostopic pub /somebodyat_mock diagnostic_msgs/KeyValue "key: '$1'
value: '$2'" -1 
