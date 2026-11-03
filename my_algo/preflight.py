"""발행자 점검 규칙 (ROS 비의존). 같은 ROS_DOMAIN_ID에 #2가 있을 때 에고 토픽에 다른 발행자가 섞였는지 본다.

규칙 문자열: "토픽|허용 노드1,허용 노드2|최소 개수|최대 개수"
    노드 이름은 전체 이름(네임스페이스 포함, 예 /exp_drive_mux, /car2/vesc_driver_node).
    허용 목록 밖 노드, 또는 개수 범위 밖이면 위반. 다른 호스트에서 같은 이름 노드가 떠도 개수 초과로 잡힌다.
    (ROS 그래프 API는 호스트 정보를 주지 않으므로 '노드 이름 + 개수'로 판단한다.)
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Rule:
    topic: str
    allowed: tuple
    min_count: int
    max_count: int


def parse_rule(text):
    parts = [p.strip() for p in text.split('|')]
    if len(parts) != 4:
        raise ValueError(f'규칙 형식 "토픽|노드,노드|최소|최대": {text!r}')
    allowed = tuple(n.strip() for n in parts[1].split(',') if n.strip())
    return Rule(parts[0], allowed, int(parts[2]), int(parts[3]))


def check(rules, observed):
    """observed: {토픽: [발행 노드 전체 이름, ...]} → 위반 문자열 목록 (비어 있으면 통과)."""
    problems = []
    for rule in rules:
        pubs = observed.get(rule.topic, [])
        foreign = sorted({n for n in pubs if n not in rule.allowed})
        if foreign:
            problems.append(f'{rule.topic}: 예상 외 발행자 {foreign}')
        if not rule.min_count <= len(pubs) <= rule.max_count:
            problems.append(f'{rule.topic}: 발행자 {len(pubs)}개 {sorted(pubs)} '
                            f'(허용 {rule.min_count}–{rule.max_count})')
    return problems


def full_name(namespace, name):
    ns = namespace.rstrip('/')
    return f'{ns}/{name}' if ns else f'/{name}'
