 #!/bin/bash

xterm -e /opt/Aldebaran/naoqi/naoqi-sdk-2.5.5.5-linux64/naoqi &
sleep 3
xterm -e python actions/init_actions.py &
xterm -e build-linux64/sdk/bin/pnp_naoqi &
sleep 3
python plans/run_plan.py --plan test1


