DESTDIR=.
scp build-pepper/sdk/lib/* nao@$PEPPER_IP:$DESTDIR/lib
scp build-pepper/sdk/bin/* nao@$PEPPER_IP:$DESTDIR/bin
scp plans/*.pnml nao@$PEPPER_IP:$DESTDIR/plans
scp plans/run_plan.py nao@$PEPPER_IP:$DESTDIR/plans
scp plans/gen_plan.bash nao@$PEPPER_IP:$DESTDIR/plans
scp actions/*.py nao@$PEPPER_IP:$DESTDIR/actions

