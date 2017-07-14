DESTDIR=.
scp build-pepper/sdk/lib/libpnp.so nao@$PEPPER_IP:$DESTDIR/lib
scp build-pepper/sdk/bin/pnp_test nao@$PEPPER_IP:$DESTDIR/bin
scp build-pepper/sdk/bin/pnp_naoqi nao@$PEPPER_IP:$DESTDIR/bin
scp plans/*.pnml nao@$PEPPER_IP:$DESTDIR/plans
scp plans/run_plan.py nao@$PEPPER_IP:$DESTDIR/plans
scp actions/*.py nao@$PEPPER_IP:$DESTDIR/actions

