"""One frozen P4 scenario, two real adjacency-dependent dynamic executions."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import time

from usv_uuc_cooperation.config import PACKAGE_ROOT, PROJECT_ROOT, VERSIONS_ROOT, FRAMEWORK_VERSION, read_json, new_run_directory
from usv_uuc_cooperation.control.integrated_executor import simulate
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.scheduling.x1_integrated import build_problem

CONFIG_PATH = PACKAGE_ROOT / 'configs/X1-L4_V01.json'


def load_config():
    config = read_json(CONFIG_PATH)
    source = PROJECT_ROOT / config['x1']['source_path']
    if digest(source) != config['x1']['source_sha256']:
        raise ValueError('P4 scenario source identity changed')
    original = read_json(source)
    preserved = copy.deepcopy(config)
    preserved.pop('x1')
    preserved['config_version'] = original['config_version']
    if preserved != original:
        raise ValueError('Frozen X1 physical scenario differs from P4')
    return config


def group_config(config, mode):
    config = copy.deepcopy(config)
    config['map'].update(adjacency=mode, movement='four-neighbor' if mode == 4 else 'eight-neighbor')
    return config


def write_manifest(folder):
    (folder / 'SHA256.txt').write_text(''.join(f'{digest(p)}  {p.relative_to(folder)}\n'
        for p in sorted(folder.rglob('*')) if p.is_file() and p != folder / 'SHA256.txt'))


def summary(rows, groups):
    text = ['# X1-L4 P4融合实际对照', '', f'{FRAMEWORK_VERSION}；同一P4物理输入，各自邻接初始Pyomo分配，同DYNAMIC_REPLAN。', '',
            '| 邻接 | completed_task_count 完成数 | actual_total_distance_m 实际总航程/m | team_completion_time_s 团队完成时间/s |',
            '|---|---:|---:|---:|']
    for r in rows:
        text.append(f"| {r['adjacency']} | {r['completed_task_count']}/{r['known_task_count']} | {r['actual_total_distance_m']:.9f} | {r['team_completion_time_s'] if r['team_completion_time_s'] is not None else 'null'} |")
    text += ['', '实际总航程统计USV+五UUV在[0,120)的全部位移，包括故障前移动及完工后USV中继。未全完成时间为null。', '',
             '## 归属与队列（同规则，允许不同）']
    for mode, result in groups.items():
        text += ['', f'### {mode}邻接初始队列', '', *[f"- {b}: {'→'.join(q) or '空'}" for b, q in result['initial_queues'].items()]]
        for d in result['replans'][1:]:
            new = [a for a in d['assignments'] if a['old_owner'] != a['boat_id']]
            text.append(f"- t{d['time_s']} {d['reason']}：" + ('；'.join(f"{a['task_id']}→{a['boat_id']}" for a in new) or '保持健康队列'))
    text += ['', '辅助账本（不增加主读数）：', '']
    for r in rows:
        text.append(f"- {r['adjacency']}邻接：{r['total_energy_NEU']:.9f} NEU，通信保持率{r['communication_retention_rate']:.6f}。")
    text += ['', '分配/顺序/动态接手和USV跟随均可能受邻接影响；差值属于系统整体效果。整数代理最优不证明路线、时间或动态联合最优。', '',
             '[同输入与增量核查](统一核查.json) · [运行身份](运行元数据.json) · [固定配置](统一配置.json)', '',
             '教师机械核查与本人运行/独立解释分别登记；本次数据为新仿真，不拼接旧日志。']
    return '\n'.join(text) + '\n'


def run_pair(run_id):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    from usv_uuc_cooperation.metrics.x1_integrated_check import check_initial, check_result, check_pair
    assert_frozen_core()
    config = load_config()
    interpreter = PROJECT_ROOT / '课时资料/P4/P4-L4/.venv/bin/python'
    if not interpreter.is_file():
        raise FileNotFoundError(interpreter)
    folder = new_run_directory('X1', run_id)
    try:
        save(folder / '统一配置.json', config)
        groups, checks, rows = {}, {}, []
        for mode in (4, 8):
            print(f'邻接{mode}：生成同场景初始问题与真实求解', flush=True)
            part = folder / f'邻接{mode}'
            part.mkdir()
            c = group_config(config, mode)
            problem = build_problem(c)
            save(part / '初始问题.json', problem)
            returned = subprocess.run([str(interpreter), '-m', 'usv_uuc_cooperation.scheduling.x1_integrated',
                '--problem', str(part / '初始问题.json')], cwd=PROJECT_ROOT, text=True, capture_output=True, timeout=90)
            (part / '求解器进程.txt').write_text(returned.stdout + returned.stderr)
            if returned.returncode:
                raise RuntimeError(f'Initial solver failed; see {part / "求解器进程.txt"}')
            solved = read_json(part / '初始分配.json')
            initial_check = check_initial(c, problem, solved)
            print(f'邻接{mode}：{solved["status"]}，开始同策略动态执行', flush=True)
            stamp = time.perf_counter()
            result = simulate(c, solved['assignment'], config['x1']['strategy'])
            s = result['summary']
            s['actual_total_distance_m'] = sum(b['distance_m'] for b in s['boat_totals'].values())
            s['distance_observation_scope'] = config['x1']['main_distance_scope']
            result.update(adjacency=mode, framework=FRAMEWORK_VERSION,
                          initial_problem_sha256=digest(part / '初始问题.json'), initial_assignment_sha256=digest(part / '初始分配.json'),
                          execute_wall_s=time.perf_counter() - stamp)
            save(part / '结果.json', result)
            # Verify exactly the serialized artifact later used for display.
            actual_check = check_result(read_json(part / '结果.json'))
            checks[str(mode)] = {'initial': initial_check, 'execution': actual_check}
            save(part / '增量核查.json', checks[str(mode)])
            with (part / '实际控制与能量.jsonl').open('w') as stream:
                for row in result['steps']:
                    stream.write(json.dumps(row, ensure_ascii=False) + '\n')
            write_csv(part / '逐时通信.csv', result['links'])
            save(part / '事件与重规划.json', {'events': result['event_log'], 'replans': result['replans']})
            save(part / '指标.json', s)
            groups[str(mode)] = result
            rows.append({'adjacency': mode, **{k: s[k] for k in ('completed_task_count', 'known_task_count',
                'actual_total_distance_m', 'team_completion_time_s', 'total_energy_NEU')},
                'communication_retention_rate': s['communication']['retention_rate']})
            write_manifest(part)
        pair = check_pair(groups)
        save(folder / '统一核查.json', {'status': 'PASS', 'pair': pair, 'groups': checks,
                                       'learner_acceptance': 'pending independent learner explanation'})
        save(folder / '指标.json', rows)
        (folder / '摘要.md').write_text(summary(rows, groups), encoding='utf-8')
        manifest = VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'lesson': 'X1-L4',
            'command': sys.argv, 'python': sys.executable, 'solver_python': str(interpreter), 'output': str(folder),
            'config_sha256': digest(CONFIG_PATH), 'source_scenario_sha256': config['x1']['source_sha256'],
            'source_manifest_sha256': digest(manifest),
            'source_sha256': dict((name, sha) for sha, name in (line.split('  ', 1) for line in manifest.read_text().splitlines())),
            'status': 'COMPLETE', 'group_execution_count': 2, 'rendered': False,
            'role': 'invocation provenance confirmed in lesson record; mechanical checks do not imply learner acceptance'})
        write_manifest(folder)
        print('两组实际执行和保存产物增量核查PASS', flush=True)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
