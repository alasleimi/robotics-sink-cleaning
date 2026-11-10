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


def test_optional_telemetry_missing_still_trains_but_required_pose_must_exist(tmp_path):
    rows=fixture_demo(tmp_path)
    meta=json.loads((tmp_path/'metadata.json').read_text())
    meta['schema']=2
    (tmp_path/'metadata.json').write_text(json.dumps(meta))
    rows[:,8:21]=np.nan
    np.savetxt(tmp_path/'samples.csv',rows,delimiter=',',header=HEADER,comments='')
    loaded,_=load(tmp_path)
    _,poses=resample(loaded)
    assert np.all(np.isfinite(poses))
    rows[4,1]=np.nan
    np.savetxt(tmp_path/'samples.csv',rows,delimiter=',',header=HEADER,comments='')
    with pytest.raises(ValueError,match='Nonfinite'):load(tmp_path)


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


def test_mouse_guiding_tracks_pose_and_recovers_from_unreachable_target(robot, monkeypatch):
    from server import Engine
    engine=Engine(robot)
    ident=engine.submit({'type':'handguiding','which':'tcp','jointSpeed':.3,'cartSpeed':.1,
                         'control':{'kind':'joint','stiffness':[0]*7,'damping':[.7]*7}})['id']
    engine.tick()
    engine.request({'action':'handguide','enabled':True})
    engine.request({'action':'teachTarget'})

    # Regress the freeze: mouse updates must never invoke the expensive full IK
    # solver, including when a marker stays outside the workspace for a while.
    def full_ik_forbidden(*args, **kwargs):
        pytest.fail('Interactive teaching called the full pose solver')
    monkeypatch.setattr(robot,'inverse',full_ik_forbidden)
    start=robot.pose('tcp')
    target=np.array(start)
    target[:3]+=[-.03,.04,.025]
    target[3:]=(Rotation.from_rotvec([.08,-.04,.06])*Rotation.from_quat(start[3:])).as_quat()

    def follow(pose, steps):
        robot.set_target(pose)
        previous=np.array(engine.manual_setpoint if engine.manual_setpoint is not None else robot.state()['q'])
        for _ in range(steps):
            engine.tick()
            command=np.array(engine.manual_setpoint)
            assert np.max(np.abs(command-previous))<=.0060001
            robot.validate_joints(command)
            previous=command

    def assert_at(pose):
        actual=robot.pose('tcp')
        np.testing.assert_allclose(actual[:3],pose[:3],atol=.002)
        assert (Rotation.from_quat(actual[3:])*Rotation.from_quat(pose[3:]).inv()).magnitude()<.01

    follow(target,250)
    assert_at(target)
    impossible=target.copy(); impossible[:3]=[3.,0.,3.]
    follow(impossible,100)
    follow(start,500)
    assert_at(start)
    engine.request({'action':'handguide','enabled':False}); engine.tick()
    assert engine.motions[ident]['state']=='finished'
    assert engine.manual is None and not engine.follow_target


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
