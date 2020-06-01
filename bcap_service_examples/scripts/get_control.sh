#!/bin/bash

source ~/cobotta_ws/devel/setup.bash

rosservice call /bcap_service '{func_id: 3, vntArgs: [{vt: 8, value: "b-CAP"}, {vt: 8, value: "CaoProv.DENSO.VRC"}, {vt: 8, value: "localhost"}, {vt: 8, value: ""}] }'
