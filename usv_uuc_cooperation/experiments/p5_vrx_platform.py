"""Run isolated P5 original/pair observations after assets have been prefetched."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
import uuid


READINESS = '''import json,math,sys,time
from pathlib import Path
from queue import SimpleQueue
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
mode,report,clock_topic,odom_topic=sys.argv[1:]
rclpy.init(args=[]);n=rclpy.create_node('p5_platform_readiness')
q=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
iq=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
s={'clock_samples':0,'clock_s':None,'feedback':None,'image':None};fresh={'clock':0,'feedback':0}
def clock(m):
 t=m.clock.sec+m.clock.nanosec*1e-9
 if s['clock_s'] is not None and t<s['clock_s']:s['clock_samples']=0
 if s['clock_s'] is None or t>s['clock_s']:s['clock_samples']+=1;fresh['clock']=time.monotonic()
 s['clock_s']=t
def feedback(p,q,parent,child,sec,nsec,received):
 if all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>1e-12:
  s['feedback']={'parent':parent,'child':child,'x':p.x,'y':p.y,'z':p.z,'stamp_sec':sec,'stamp_nanosec':nsec}
  fresh['feedback']=received;return True
 return False
def pose(received,m):
 matches=[p for p in m.pose if p.name=='wamv']
 if len(matches)>1:raise RuntimeError('Ambiguous native model name')
 for p in matches:
  if feedback(p.position,p.orientation,'native world:sydney_regatta',p.name,m.header.stamp.sec,m.header.stamp.nsec,received):
   s['feedback'].update(entity_id=p.id,entity_name=p.name)
def odom(m):feedback(m.pose.pose.position,m.pose.pose.orientation,m.header.frame_id,m.child_frame_id,m.header.stamp.sec,m.header.stamp.nanosec,time.monotonic())
def image(m,topic):
 if s['image'] is None and m.width>0 and m.height>0 and m.step>0 and len(m.data)==m.height*m.step:
  s['image']={'topic':topic,'width':m.width,'height':m.height,'encoding':m.encoding,'step':m.step,'data_bytes':len(m.data)}
subs=[n.create_subscription(Clock,clock_topic,clock,q)]
images={};pending=SimpleQueue();native=None;deadline=time.monotonic()+100;scan=0;s['ready']=False
try:
 if mode=='original':
  from gz.transport13 import Node as GazeboNode
  from gz.msgs10.pose_v_pb2 import Pose_V
  native=GazeboNode()
  if native.subscribe(Pose_V,'/world/sydney_regatta/pose/info',lambda m:pending.put((time.monotonic(),m))) is False:raise RuntimeError('Native GZ subscribe failed')
 else:subs.append(n.create_subscription(Odometry,odom_topic,odom,q))
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=0.1);now=time.monotonic()
  while not pending.empty():pose(*pending.get_nowait())
  if mode=='original' and now>=scan:
   for topic,types in n.get_topic_names_and_types():
    if topic.startswith('/wamv/') and 'sensor_msgs/msg/Image' in types and topic not in images:
     images[topic]=n.create_subscription(Image,topic,lambda m,t=topic:image(m,t),iq)
   scan=now+0.5
  s['ready']=s['clock_samples']>=5 and s['feedback'] is not None and now-fresh['clock']<5 and now-fresh['feedback']<5 and (mode=='pair' or s['image'] is not None)
  if s['ready']:break
finally:
 if native is not None:native.unsubscribe('/world/sydney_regatta/pose/info');native=None
 s['image_topics']=sorted(images);Path(report).write_text(json.dumps(s,indent=2)+'\\n')
 n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if s['ready'] else 2)
'''


def ros_command(workspace, arguments):
    setup = 'source /opt/ros/jazzy/setup.bash && source ' + shlex.quote(str(workspace / 'install/setup.bash')) + ' && exec "$@"'
    return ['/bin/bash', '-c', setup, 'p5-vrx', *arguments]


def stop_owned(process):
    remaining = True
    for sig, wait_s in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            remaining = False
            break
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                remaining = False
                break
            time.sleep(0.1)
        if not remaining:
            break
    process.poll()
    return {'pid': process.pid, 'returncode': process.returncode, 'process_group_remaining': remaining}


def run(args):
    paths = [args.out_root, args.vrx_workspace, args.fuel_cache, args.tools_dir] + ([args.urdf] if args.urdf else [])
    if any(not path.is_absolute() for path in paths):
        raise ValueError('All supplied paths must be absolute')
    if Path(args.run_id).name != args.run_id or args.run_id in ('', '.', '..'):
        raise ValueError('--run-id must be one new directory name')
    workspace, fuel, tools = args.vrx_workspace.resolve(), args.fuel_cache.resolve(), args.tools_dir.resolve()
    files = {name: tools / name for name in ('p5_vrx.py', 'P5-VRX-V01.json', 'VRX_官方原例记录器_V02.py')}
    if not fuel.is_dir() or not (workspace / 'install/setup.bash').is_file() or any(not path.is_file() for path in files.values()):
        raise ValueError('Existing VRX install, prefetched Fuel cache and all three tools are required')
    urdf = args.urdf.resolve() if args.urdf else None
    if args.mode in ('pair', 'all'):
        if urdf is None or not urdf.is_file():
            raise ValueError('--urdf is required for pair/all')
        text = urdf.read_text()
        if '<xacro:' in text or '$(arg ' in text or 'OdometryPublisher' not in text:
            raise ValueError('--urdf must be a fully expanded URDF with native ground-truth odometry')
    config = json.loads(files['P5-VRX-V01.json'].read_text())
    source = workspace / 'src/vrx'
    commit = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    dirty = subprocess.run(['git', '-C', str(source), 'status', '--porcelain'], capture_output=True, text=True, check=True).stdout
    expected_commit = config['source_id'].rsplit(':', 1)[-1]
    tracked_dirty = [line for line in dirty.splitlines() if not line.startswith('?? ')]
    root = args.out_root.resolve() / args.run_id
    root.mkdir(parents=True, exist_ok=False)
    base = {'role': args.role, 'framework_version': args.framework_version, 'vrx_workspace': str(workspace),
            'upstream_commit': commit, 'upstream_clean': not bool(dirty), 'upstream_porcelain': dirty,
            'declared_source_id': config['source_id'], 'expected_commit': expected_commit, 'upstream_tracked_clean': not tracked_dirty,
            'build_identity': 'Git identity alone does not prove installed binaries correspond to this checkout',
            'fuel_cache': str(fuel), 'asset_requirement': 'All world and secondary assets must already be prefetched; directory existence is not completeness verification',
            'tools_sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in files.items()},
            'urdf': str(urdf) if urdf else None, 'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest() if urdf else None,
            'platform_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    modes = ['original', 'pair'] if args.mode == 'all' else [args.mode]
    domain_base, results = 20 + int(uuid.uuid4().hex[:4], 16) % 180, []
    previous_term = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt('SIGTERM received')

    signal.signal(signal.SIGTERM, terminate)
    try:
        for index, mode in enumerate(modes):
            out = root / mode
            out.mkdir()
            runtime = out / 'runtime'
            runtime.mkdir(mode=0o700)
            env = {**os.environ, 'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                   'PYTHONDONTWRITEBYTECODE': '1', 'ROS_DOMAIN_ID': str(domain_base + index),
                   'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'GZ_IP': '127.0.0.1',
                   'GZ_PARTITION': 'p5_' + uuid.uuid4().hex, 'GZ_FUEL_CACHE_PATH': str(fuel),
                   'ROS_LOG_DIR': str(out / 'ros_logs'), 'XDG_RUNTIME_DIR': str(runtime)}
            metadata = {**base, 'mode': mode, 'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                        'ros_domain_id': env['ROS_DOMAIN_ID'], 'gz_partition': env['GZ_PARTITION'], 'commands': []}
            processes, handles, status = [], [], 'NOT_STARTED'
            if commit != expected_commit or tracked_dirty:
                metadata.update(status='SOURCE_IDENTITY_FAILED', owned_process_cleanup=[])
                (out / 'launch_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
                results.append({'mode': mode, 'status': 'SOURCE_IDENTITY_FAILED', 'output': str(out)})
                break

            def start(label, arguments):
                stream = (out / (label + '.log')).open('x')
                handles.append(stream)
                process = subprocess.Popen(ros_command(workspace, arguments), cwd=workspace, env=env,
                                           stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append((label, process))
                metadata['commands'].append({'label': label, 'argv': arguments, 'pid': process.pid})
                return process

            try:
                launch = ['ros2', 'launch', 'vrx_gz', 'competition.launch.py', 'world:=sydney_regatta',
                          'headless:=true', 'paused:=false', 'extra_gz_args:=--headless-rendering']
                if mode == 'pair':
                    launch.append('urdf:=' + str(urdf))
                start('launch', launch)
                probe = out / 'readiness.py'
                probe.write_text(READINESS)
                check = start('readiness', ['/usr/bin/python3', '-B', str(probe), mode, str(out / 'readiness.json'),
                              '/clock' if mode == 'original' else config['topics']['clock'], config['topics']['odometry']])
                if check.wait(timeout=115) != 0:
                    status = 'READINESS_FAILED'
                else:
                    ready = json.loads((out / 'readiness.json').read_text())
                    metadata['observed_readiness'] = ready
                    if not ready['ready']:
                        raise RuntimeError('Readiness probe did not confirm real feedback')
                    task = ['/usr/bin/python3', '-B', str(files['VRX_官方原例记录器_V02.py'] if mode == 'original' else files['p5_vrx.py']),
                            '--out', str(out / 'observations'), '--role', args.role]
                    if mode == 'original':
                        task += ['--pose-topic', '/world/sydney_regatta/pose/info', '--model-frame', 'wamv', '--image-topic', ready['image']['topic']]
                    else:
                        task += ['--config', str(files['P5-VRX-V01.json']), '--framework-version', args.framework_version]
                    returned = start('recorder' if mode == 'original' else 'controller', task).wait(timeout=270 if mode == 'original' else 630)
                    child = json.loads((out / 'observations/summary.json').read_text())
                    metadata.update(child_returncode=returned, child_summary=child)
                    verified = child.get('recording_complete') if mode == 'original' else child.get('task_completed')
                    status = 'COMPLETED' if returned == 0 and verified is True else 'OBSERVATION_FAILED'
            except (Exception, KeyboardInterrupt) as exc:
                status = 'INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'ERROR'
                metadata['error'] = f'{type(exc).__name__}: {exc}'
            finally:
                cleanup = [dict(label=label, **stop_owned(process)) for label, process in reversed(processes)]
                for stream in handles:
                    stream.close()
                if any(item['process_group_remaining'] for item in cleanup):
                    status = 'CLEANUP_INCOMPLETE'
                metadata.update(status=status, owned_process_cleanup=cleanup)
                (out / 'launch_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
            results.append({'mode': mode, 'status': status, 'output': str(out)})
            if status != 'COMPLETED':
                break
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        summary = {**base, 'status': 'COMPLETED' if len(results) == len(modes) and all(r['status'] == 'COMPLETED' for r in results) else 'INCOMPLETE',
                   'runs': results, 'course_acceptance': 'NOT_AUTOMATICALLY_ASSESSED'}
        (root / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({'status': summary['status'], 'output': str(root)}, ensure_ascii=False), flush=True)
    return 0 if summary['status'] == 'COMPLETED' else 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=('original', 'pair', 'all'))
    parser.add_argument('--role', required=True, choices=('teacher', 'learner'))
    parser.add_argument('--run-id', required=True)
    for name in ('out-root', 'vrx-workspace', 'fuel-cache', 'tools-dir'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--framework-version', required=True)
    raise SystemExit(run(parser.parse_args()))
