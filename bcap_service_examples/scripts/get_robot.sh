#!/bin/bash

source ~/cobotta_ws/devel/setup.bash

rosservice call /bcap_service '{func_id: 7, vntArgs: [{vt: 3, value: "48"}, {vt: 8, value: "Arm"}, {vt: 8, value: ""}] }'

