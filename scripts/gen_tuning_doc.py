#!/usr/bin/env python3
"""TUNE 태그를 모아 docs/TUNING.md를 만든다. 커밋 전에 실행한다 (라인 번호 갱신).

    python3 scripts/gen_tuning_doc.py          # 다시 쓰기
    python3 scripts/gen_tuning_doc.py --check  # 최신인지 확인만 (테스트가 부른다)

태그 형식:  <이름>: <값>   # TUNE: <설명> | 단위 | 현재값 근거(임시/측정/기획안)
"바꿀 때 영향"과 "권장 범위"는 아래 NOTES에 적는다.
"""
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEARCH = ('config', 'my_algo', 'sim', 'scripts')
OUT = os.path.join(REPO, 'docs', 'TUNING.md')
TAG = re.compile(r'^\s*([A-Za-z_][\w.]*)\s*[:=]\s*(.*?)\s*#\s*TUNE:\s*(.*?)\s*\|\s*(.*?)\s*\|\s*(.*?)\s*$')

# 이름 → (바꿀 때 영향, 권장 범위). 권장 범위는 측정 전 추정이며 근거가 '임시'인 값은 실차에서 다시 정한다.
NOTES = {
    't_d': ('키우면 d_safe·d*가 커지고 v_lim이 작아진다. AEB(relative)와 같이 쓴다', '0.10~0.30'),
    'a_brake': ('줄이면 d_safe가 커지고 접근이 느려진다. 실제보다 크면 급정지 때 여유 m을 못 지킨다', '2.0~4.0'),
    'scan_period': ('d_safe·v_lim·d*의 반응 지연에 더해진다. 0이면 기획안 식 (급정지 전 h가 이산화로 음수가 된다)', 'scan 주기 (0.1)'),
    'W_leader': ('선행차가 옆에 온 후행차를 왼쪽 벽으로 보고 오른쪽 벽에 닿는 문제를 막는다. 틀리면 선행차 차선 위치가 틀어진다', '실측 ±0.02'),
    'm_stop': ('정지 후 간격, MERGE 시작 조건(선행차 앞면이 뒤범퍼보다 m 뒤). 키우면 d*가 멀어진다', '0.10~0.25'),
    'eps_follow': ('v_lim과 hold 판정의 여유. 키우면 d*가 멀어지고 hold가 줄어든다', '0.03~0.10'),
    'belief_model': ('original은 Y를 NY로 오판(폐루프 115/200), survival은 188/200', 'survival'),
    'd_y_min': ('survival에서 NY 확정 거리가 d_y_min + (1−η)(d_y_max − d_y_min)이 된다. 선행차 d_y 추첨 범위이기도 하다', '0.4~0.8'),
    'd_y_max': ('이 거리 밖에서는 belief가 변하지 않는다. d* < d_y_max여야 탐색이 된다', '1.0~1.5'),
    'v_lat': ('original: 양보 우도의 평균 변위. survival: 감지 지연 τ. 실제보다 크면 τ를 짧게 봐서 NY 오판이 는다', '0.3~0.6'),
    'sigma_z': ('original만: 작을수록 scan 하나의 증거가 커진다', '0.01~0.05'),
    'eta': ('키우면 판단이 늦고 오판이 준다. survival의 Y 놓침 비율이 약 1−η', '0.8~0.95'),
    'lambda_clip': ('λ 포화. 줄이면 반대 증거에 빨리 뒤집힌다 (original만 의미 있음)', '10~30'),
    'y_yield_thr': ('줄이면 양보를 빨리 감지하지만 선행차 흔들림을 양보로 본다 (NY 추월 진입 위험)', '0.06~0.12 (3σ 이상)'),
    't_react': ('키우면 NY 판정이 그만큼 늦어지고 (d*에서 기다림) Y 놓침이 준다. 실제 선행차 반응보다 짧으면 Y를 NY로 오판한다', '0.5~2.0 (선행차 횡 응답 측정값)'),
    'p_false_yield': ('키우면 감지 직후 b가 낮아 π(d)가 클 때만 Y로 확정된다', '0.001~0.05'),
    'T_max': ('줄이면 추월 포기가 빨라진다. 통로 길이 / v_B보다 짧아야 한다', '5~12'),
    'w_car': ('δ, 추월 가능 판정, PASS 가속 조건', '측정값 고정'),
    'c_lat': ('키우면 δ가 줄고(벽에서 멀어짐) 추월 가능 판정이 엄격해진다. 추월 차선 벽 거리 = w/2 + c', '0.05~0.15'),
    'W': ('null이면 scan 추정. 고정하면 벽 제거와 δ가 흔들리지 않는다', '0.84~1.3 (실측)'),
    'lane_offset': ('직접 주면 W와 무관하게 차선 위치를 고정한다', 'null'),
    'v_B_nominal': ('d*와 접근 속도 차. v_min(0.867)보다 커야 한다', '0.87~1.1'),
    'v_pass': ('키우면 추월이 빨라지지만 접근 중 v_lim에 걸리는 시간이 늘고 제동 거리가 길어진다', '1.3~1.8'),
    'erpm_gain': ('속도 환산 전체. 틀리면 v_E, v_B, h가 모두 틀어진다', '측정값 고정'),
    'min_erpm': ('v_min = min_erpm / erpm_gain', '측정값 고정'),
    't0_speed_threshold': ('출발 시각과 belief 초기화 시점', '0.2~0.4'),
    'hold_on_speed': ('낮추면 저속에서도 hold로 v_min을 낸다 (정지 직후 튀어나감)', 'hold_off_speed + 0.05 이상'),
    'hold_off_speed': ('높이면 감속 중 hold가 빨리 풀려 정지가 잦다', '0.15~0.30'),
    'd_follow_a': ('방법 a 추종 거리', '0.6~1.0'),
    'T_wait_a': ('방법 a가 추월을 시도하기까지의 시간', '2~5'),
    'd_follow_b': ('d_y_max보다 커야 "정보 없음" 기준선이 된다', '> d_y_max'),
    'd_probe_c': ('작을수록 AEB와 h < 0이 늘어난다 (기준선의 의도)', '0.3~0.5'),
    'k_d': ('키우면 거리 수렴이 빠르지만 속도 명령이 흔들린다', '1.0~2.5'),
    'probe_tol': ('키우면 PROBE에 일찍 들어간다 (T_max·T_wait_a 시작 시점)', '0.03~0.10'),
    'lane_change_tol': ('줄이면 v_pass 가속이 늦어져 추월이 길어지고, 키우면 차선 변경 중 가속해 벽이 AEB 띠에 들어온다', '0.03~0.08'),
    'merge_tol': ('키우면 DONE 판정이 빨라진다', '0.03~0.08'),
    'track_hold': ('키우면 추적 끊김에 둔해지지만 오래된 d로 속도를 낸다', '0.2~0.8'),
    'lidar_to_front': ('d 전체에 더해지는 오프셋. AEB와 다르면 d와 closest가 어긋난다', '측정값 고정'),
    'lidar_to_rear': ('d_rear와 MERGE 판정', '측정값 고정'),
    'cluster_jump': ('키우면 벽 근처 차와 벽 조각이 붙고, 줄이면 먼 차가 쪼개진다', '0.10~0.20'),
    'cluster_min_points': ('키우면 먼 차를 놓치고, 줄이면 잡음 점을 차로 본다', '3~6'),
    'track_x_max': ('검출 시작 거리', '2.5~4.0'),
    'veh_width_min': ('줄이면 벽 조각·기둥을 차로 본다', '0.10~0.20'),
    'veh_width_max': ('선행차가 틀면 보이는 폭이 w·cosψ + L·sinψ로 커진다. 작으면 양보 중 추적이 끊긴다', '0.45~0.60'),
    'veh_depth_max': ('헤딩이 틀어졌을 때 통로 안으로 들어온 벽 조각 제거', '0.55~0.70'),
    'wall_margin': ('c_lat보다 작아야 벽에 붙은 차를 잃지 않는다. 너무 작으면 벽 잡음이 남는다', '0.03~0.08'),
    'rear_fov_half_deg': ('선행차가 후행차를 찾는 범위', '45~75'),
    'track_alpha': ('키우면 d 추정이 빠르고 잡음에 민감하다', '0.3~0.7'),
    'track_beta': ('키우면 v_B 추정이 빠르고 잡음에 민감하다', '0.05~0.3'),
    'track_jump_reset': ('작으면 잡음에 필터가 초기화되어 v_B가 v_E로 튄다', '0.2~0.5'),
    'track_timeout': ('끊긴 뒤 필터를 이어 쓸지', '0.3~1.0'),
    'rear_face_depth': ('키우면 옆면 점이 섞여 틀어진 선행차의 y_B가 치우치고, 줄이면 뒷면 점이 모자라 흔들린다', '0.03~0.08'),
    'side_gate': ('추월 중 선행차를 다시 붙잡는 범위', '0.4~0.8'),
    'corridor_sector_min_deg': ('벽 거리 섹터', '기존 값 유지'),
    'corridor_sector_max_deg': ('벽 거리 섹터', '기존 값 유지'),
    'corridor_min_points': ('벽 인정 점 수', '기존 값 유지'),
    'corridor_wall_percentile': ('벽 거리 분위값', '기존 값 유지'),
    'min_point_x': ('−0.05면 벽 거리 < 0.25 m에서 벽을 놓친다 (추월 차선 0.235 m). −1.0은 곡선에서 편향이 더 작지만 앞쪽 벽을 미리 보는 효과가 없다', '−1.0 (직선 통로)'),
    'wall_follow_gain': ('키우면 횡 수렴이 빠르지만 흔들린다', '0.2~0.5'),
    'wall_error_filter_alpha': ('키우면 반응이 빠르고 잡음에 민감하다', '0.15~0.5'),
    'steer_rate_limit': ('키우면 차선 변경이 빠르고 덜그럭거린다', '0.05~0.15'),
    'steering_deadband': ('0.055 rad는 횡오차 약 0.09 m에 해당한다. 추월 차선 벽 여유(c = 0.10)와 merge_tol보다 크면 줄여야 한다', '0.01~0.055'),
    'max_steering': ('조향 포화', '기존 값 유지'),
    'heading_gain': ('감쇠비 ζ = heading_gain / (2·sqrt(2·wall_follow_gain·L)), 속도와 무관. 0.7이면 ζ ≈ 0.8', '0.5~1.0'),
    'y_ref_rate': ('키우면 차선 변경이 빠르고 yaw가 커진다 (tracker 폭 게이트)', '0.2~0.5'),
    'd_brake_trigger': ('NY 급정지 이벤트가 무장되는 거리', '0.8~1.2'),
    'brake_delay_min': ('급정지 시점 난수 범위', '0.5'),
    'brake_delay_max': ('급정지 시점 난수 범위', '2.0'),
    'start_gap_margin': ('출발 간격 경고 기준', '0.2'),
}


def collect():
    rows = []
    for top in SEARCH:
        for dirpath, dirnames, filenames in os.walk(os.path.join(REPO, top)):
            dirnames[:] = sorted(d for d in dirnames if d not in ('__pycache__', '.venv', 'legacy'))
            for name in sorted(filenames):
                if not name.endswith(('.py', '.yaml', '.yml')) or name == 'gen_tuning_doc.py':
                    continue
                path = os.path.join(dirpath, name)
                with open(path, encoding='utf-8') as f:
                    for no, line in enumerate(f, 1):
                        m = TAG.match(line)
                        if m:
                            rows.append((os.path.relpath(path, REPO), no) + m.groups())
    return rows


def render(rows):
    out = [
        '# 조정 파라미터 (TUNE 태그)',
        '',
        '이 파일은 `scripts/gen_tuning_doc.py`가 만든다. 손으로 고치지 말고, 값·태그를 바꾼 뒤 커밋 전에 다시 실행한다.',
        '라인 번호가 어긋나 보이면 직접 찾는다:',
        '',
        '    grep -rn "TUNE:" config my_algo sim scripts',
        '',
        '태그 형식: `# TUNE: <설명> | 단위 | 현재값 근거(임시/측정/기획안)`.',
        '"기획안" 값은 가정값이며 측정값이 아니다. 권장 범위도 측정 전 추정이다.',
        '',
        '| 파일 | 라인 | 파라미터 | 현재값 | 단위 | 의미 | 근거 | 바꿀 때 영향 | 권장 범위 |',
        '|---|---|---|---|---|---|---|---|---|',
    ]
    for path, no, name, value, desc, unit, basis in rows:
        impact, rng = NOTES.get(name, ('-', '-'))
        out.append('| %s | %d | `%s` | %s | %s | %s | %s | %s | %s |' % (
            path, no, name, value.rstrip(','), unit, desc, basis, impact, rng))
    return '\n'.join(out) + '\n'


def main():
    text = render(collect())
    if '--check' in sys.argv:
        current = open(OUT, encoding='utf-8').read() if os.path.exists(OUT) else ''
        if current != text:
            print('docs/TUNING.md가 최신이 아니다: python3 scripts/gen_tuning_doc.py')
            return 1
        return 0
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write(text)
    print('docs/TUNING.md: %d개' % text.count('\n| '))
    return 0


if __name__ == '__main__':
    sys.exit(main())
