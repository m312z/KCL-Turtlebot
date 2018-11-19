#!/bin/bash
rostopic pub /isbusy_mock diagnostic_msgs/KeyValue "key: '$1'
value: '$2'" -1 
