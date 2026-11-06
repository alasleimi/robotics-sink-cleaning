import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sinkcleaning.demonstrations import HEADER,load,in_frame,resample


def fixture_demo(tmp_path):
    rows=np.zeros((20,22)); rows[:,0]=np.arange(20)*.02
    rows[:,1]=np.linspace(.4,.5,20); rows[:,7]=1
    np.savetxt(tmp_path/'samples.csv',rows,delimiter=',',header=HEADER,comments='')
    (tmp_path/'metadata.json').write_text(json.dumps({'schema':1,'complete':True,'source':'robot','pose_frame':'robot_base'}))
    return rows


def test_robot_format_resampling_and_sink_transform(tmp_path):
    fixture_demo(tmp_path)
    rows,meta=load(tmp_path)
    phase,poses=resample(rows,30)
    assert meta['source']=='robot' and len(phase)==30
    np.testing.assert_allclose(poses[[0,-1],0],[.4,.5])
    frame=np.r_[[.1,0,0],Rotation.from_euler('z',90,degrees=True).as_quat()]
    transformed=in_frame(poses,frame)
    np.testing.assert_allclose(transformed[:,0],0,atol=1e-14)
    np.testing.assert_allclose(transformed[:,1],-(poses[:,0]-.1),atol=1e-14)
    with pytest.raises(ValueError,match='unit XYZW'):in_frame(poses,[0,0,0,0,0,0,0])


def test_reject_partial_invalid_and_reversed_time(tmp_path):
    rows=fixture_demo(tmp_path)
    rows[4,0]=rows[3,0]
    np.savetxt(tmp_path/'samples.csv',rows,delimiter=',',header=HEADER,comments='')
    with pytest.raises(ValueError,match='Timestamps'): load(tmp_path)
    fixture_demo(tmp_path)
    (tmp_path/'metadata.json').write_text('{"schema":1,"complete":false}')
    with pytest.raises(ValueError,match='Incomplete'): load(tmp_path)


def test_handguiding_enable_release_and_velocity_limited_target(robot):
    from server import Engine
    engine=Engine(robot)
    ident=engine.submit({'type':'handguiding','which':'tcp','jointSpeed':.3,'cartSpeed':.1,
                         'control':{'kind':'joint','stiffness':[0]*7,'damping':[.7]*7}})['id']
    for _ in range(5):engine.tick()
    initial=np.array(engine.state['q']); target=initial.copy(); target[0]+=.15
    with pytest.raises(ValueError,match='Enable'):engine.request({'action':'teach','q':target.tolist()})
    engine.request({'action':'handguide','enabled':True})
    engine.request({'action':'teach','q':target.tolist()})
    engine.tick()
    assert np.max(np.abs(np.array(engine.manual_setpoint)-initial))<=.0061
    for _ in range(100):engine.tick()
    assert engine.state['q'][0]-initial[0]>.1
    engine.request({'action':'handguide','enabled':False}); engine.tick()
    assert engine.motions[ident]['state']=='finished'
    assert engine.manual is None


@pytest.fixture
def robot():
    # test_robosuite installs the adapter import path; make this file runnable alone.
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/adapter'))
    from model import Model
    model=Model()
    try: yield model
    finally:model.close()
