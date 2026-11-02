"""Full ros2_control/action regression using mock hardware only.

Inject execution-state values through mock GPIO commands. Never connect to a
cabinet; the fallback address is deliberately an unroutable documentation IP.
"""
import os, signal, subprocess, tempfile, time
from pathlib import Path
import yaml
os.environ['ROS_DOMAIN_ID'] = os.environ.get('AUBO_TEST_ROS_DOMAIN_ID', '98')
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus

rclpy.init()
node = rclpy.create_node('mock_motion_action_check')
processes=[]
evidence_dir = tempfile.TemporaryDirectory(prefix='aubo_mock_motion_')
log_path = str(Path(evidence_dir.name) / 'driver.log')
log = None
config_path = None

def wait_future(future, timeout=5):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done(): raise RuntimeError('action timed out')
    return future.result()

def spin_for(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:rclpy.spin_once(node, timeout_sec=.02)

try:
    config=yaml.safe_load((Path(get_package_share_directory('aubo_ros2_driver'))/'config/aubo_controllers.yaml').read_text())
    config['controller_manager']['ros__parameters']['motion_status_injector']={'type':'forward_command_controller/ForwardCommandController'}
    config['motion_status_injector']={'ros__parameters':{'joints':['motion_status'],'interface_name':'execution_state'}}
    with tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(config,f);config_path=f.name
    log=open(log_path,'w')
    p=subprocess.Popen(['ros2','launch','aubo_ros2_driver','aubo_control.launch.py',
        'aubo_type:=aubo_ES3','robot_ip:=192.0.2.1','use_fake_hardware:=true',
        'fake_sensor_commands:=true','controller_profile:=es',f'controllers_file:={config_path}'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p)
    client=ActionClient(node,FollowJointTrajectory,'/joint_trajectory_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=20):raise RuntimeError('trajectory server unavailable')
    spawned=subprocess.run(['ros2','run','controller_manager','spawner','motion_status_injector'],
        stdout=log,stderr=subprocess.STDOUT,timeout=15)
    if spawned.returncode:raise RuntimeError('status injector failed to load')
    publisher=node.create_publisher(Float64MultiArray,'/motion_status_injector/commands',10)
    deadline=time.monotonic()+5
    while publisher.get_subscription_count()==0 and time.monotonic()<deadline:spin_for(.05)
    if publisher.get_subscription_count()==0:raise RuntimeError('status injector subscriber unavailable')
    def inject(value):
        for _ in range(5):publisher.publish(Float64MultiArray(data=[float(value)]));spin_for(.02)
    joints=config['joint_trajectory_controller']['ros__parameters']['joints']
    def start_goal(target):
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=joints
        point=JointTrajectoryPoint(positions=[target]*6)
        point.time_from_start.sec=1
        goal.trajectory.points=[point]
        handle=wait_future(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('goal unexpectedly rejected')
        return handle.get_result_async()
    inject(0)
    result=wait_future(start_goal(.01))
    assert result.status==GoalStatus.STATUS_SUCCEEDED, result
    print('Healthy mock goal: SUCCEEDED')
    result_future=start_goal(.02);spin_for(.15);inject(1)
    result=wait_future(result_future)
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code!=0,result
    print('Safety pause injected through ros2_control GPIO: ABORTED')
    inject(0)
    result=wait_future(start_goal(0))
    assert result.status==GoalStatus.STATUS_SUCCEEDED,result
    print('Fresh goal after recovery: SUCCEEDED')
    result_future=start_goal(.02);spin_for(.15);inject(2)
    result=wait_future(result_future)
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code!=0,result
    print('Hardware fault status injected through ros2_control GPIO: ABORTED')
    text=Path(log_path).read_text()
    assert 'RPC Client login' not in text and 'Connect to ' not in text
    print('No SDK connection; all integration checks passed')
finally:
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    if log is not None:
        log.close()
        if __import__('sys').exc_info()[0] is not None:
            print(Path(log_path).read_text())
    if config_path:
        Path(config_path).unlink(missing_ok=True)
    node.destroy_node();rclpy.shutdown()
    evidence_dir.cleanup()
