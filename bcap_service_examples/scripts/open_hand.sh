#!/bin/bash

source ~/cobotta_ws/devel/setup.bash

rosservice call /bcap_service '{func_id: 17, vntArgs: [{vt: 3, value: "48"}, {vt: 8, value: "HandMoveA"}, {vt: 8195, value: "30,100"}] }'

