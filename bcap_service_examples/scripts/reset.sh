#!/bin/bash

source ~/cobotta_ws/devel/setup.bash

rosservice call /bcap_service '{func_id: 17, vntArgs: [{vt: 3, value: "3"}, {vt: 8, value: "ClearError"}, {vt: 8, value: ""}] }'

