#!/bin/bash

source ~/cobotta_ws/devel/setup.bash

rosservice call /bcap_service '{func_id: 64, vntArgs: [{vt: 3, value: "3"}, {vt: 8, value: "ManualResetPreparation"}, {vt: 8, value: ""}] }'

