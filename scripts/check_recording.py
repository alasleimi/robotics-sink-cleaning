"""End-to-end checks of the actual Java recorder/replayer; never uses hardware."""
import json
import subprocess
import time
import numpy as np
from sim_session import Session
from check_simulation import wait_until
from sinkcleaning.demonstrations import load


def main():
    with Session(headless=True,fast=True) as s:
        folder=s.folder/'demo'
        print('Recording integration:',s.folder,flush=True)
        p=s.java('simproxy.DemoLauncher','record',str(folder),'0',label='record')
        wait_until(lambda:'RECORDING' in (s.folder/'record.log').read_text(errors='replace'),description='recorder ready')
        q=np.array(s.call('state')['q']); time.sleep(.15)
        np.testing.assert_allclose(s.call('state')['q'],q,atol=.003)
        s.call('handguide',enabled=True)
        for i in range(1,5):
            target=q.copy(); target[0]+=.03*i
            s.call('teach',q=target.tolist())
            wait_until(lambda:np.max(np.abs(np.array(s.call('state')['q'])-target))<.004,description='guided target')
            time.sleep(.12)
        s.call('handguide',enabled=False)
        assert p.wait(timeout=30)==0,(s.folder/'record.log').read_text()
        rows,meta=load(folder)
        assert len(rows)>10 and meta['guidance']=='KUKA HRCMotions.handGuiding'
        assert meta['source']=='simulation' and len(meta['tcp'])==7
        assert np.ptp(rows[:,8])>.08
        before=np.array(s.call('state')['q'])
        refused=s.java('simproxy.DemoLauncher','replay',str(folder),label='replay-no-approach')
        try:
            assert refused.wait(timeout=30)!=0
        except subprocess.TimeoutExpired as error:
            raise AssertionError('Replay start check timed out. Java output:\n'+
                                 (s.folder/'replay-no-approach.log').read_text(errors='replace')) from error
        assert 'Guide the TCP within' in (s.folder/'replay-no-approach.log').read_text()
        np.testing.assert_allclose(s.call('state')['q'],before,atol=.004)
        p=s.java('simproxy.DemoLauncher','replay',str(folder),'--approach',label='replay')
        assert p.wait(timeout=120)==0,(s.folder/'replay.log').read_text()
        np.testing.assert_allclose(s.call('state')['tcp'][:3],rows[-1,1:4],atol=.002)
        timed=s.java('simproxy.DemoLauncher','record',str(s.folder/'timed'),'.4',label='record-timed')
        assert timed.wait(timeout=30)==0,(s.folder/'record-timed.log').read_text()
        timed_rows,_=load(s.folder/'timed')
        assert len(timed_rows)>2
        entered=s.java('simproxy.DemoLauncher','record',str(s.folder/'enter'),'30',label='record-enter')
        wait_until(lambda:'RECORDING' in (s.folder/'record-enter.log').read_text(errors='replace'),description='Enter-stop recorder ready')
        time.sleep(.2); entered.stdin.write(b'\n'); entered.stdin.flush()
        assert entered.wait(timeout=10)==0,(s.folder/'record-enter.log').read_text()
        load(s.folder/'enter')
        cancelled=s.java('simproxy.DemoLauncher','record',str(s.folder/'cancelled'),'30',label='record-cancelled')
        wait_until(lambda:'RECORDING' in (s.folder/'record-cancelled.log').read_text(errors='replace'),description='cancel recorder ready')
        time.sleep(.2); s.call('abort')
        assert cancelled.wait(timeout=10)!=0
        assert not json.loads((s.folder/'cancelled/metadata.json').read_text())['complete']
        s.call('resume')
        disconnected=s.java('simproxy.DemoLauncher','record',str(s.folder/'disconnected'),'30',label='record-disconnected')
        wait_until(lambda:'RECORDING' in (s.folder/'record-disconnected.log').read_text(errors='replace'),description='disconnect recorder ready')
        time.sleep(.2); s.bridge.terminate(); s.bridge.wait(timeout=5)
        assert disconnected.wait(timeout=10)!=0
        assert not json.loads((s.folder/'disconnected/metadata.json').read_text())['complete']
        report={'recorded_samples':len(rows),'guidance':'KUKA API through simulator',
                'checks':['waits for enable','release ends recording','finite monotonic CSV and TCP metadata',
                          'actual motion recorded','distant replay requires explicit approach',
                          'same file replayed through Sunrise Java API','time limit and Enter save complete data',
                          'external cancellation and lost connection leave incomplete takes'],
                'hardware':'not connected'}
        (s.folder/'report.json').write_text(json.dumps(report,indent=2))
        print('Record/replay checks passed:',s.folder/'report.json',flush=True)


if __name__=='__main__':main()
