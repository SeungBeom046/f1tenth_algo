"""sensor_msgs/PointCloud2 ↔ numpy 구조화 배열 (ROS 배포판 비의존).

Foxy의 sensor_msgs_py.read_points는 튜플 생성기를 돌려주고(Humble은 numpy 구조화 배열),
Foxy 바이너리에 sensor_msgs_py가 없을 수도 있어 직접 구현한다. 메시지 필드(fields, point_step, data,
is_bigendian)만 읽으므로 rclpy 메시지와 rosbags 메시지 모두에 쓸 수 있다.
"""

import numpy as np

# sensor_msgs/PointField 데이터 타입 번호 → numpy 타입
_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_dtype(fields, point_step, bigendian=False):
    order = '>' if bigendian else '<'
    names, formats, offsets = [], [], []
    for f in fields:
        base = _TYPES[f.datatype]
        names.append(f.name)
        formats.append(f'{order}{base}' if f.count <= 1 else (f'{order}{base}', (f.count,)))
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': point_step})


def read_cloud(msg, skip_nans=True):
    """모든 필드를 담은 구조화 배열 (점 수,). skip_nans면 x, y, z 중 NaN이 있는 점을 뺀다."""
    dtype = cloud_dtype(msg.fields, msg.point_step, bool(msg.is_bigendian))
    n = msg.width * msg.height
    raw = bytes(msg.data)
    if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
        rows = [np.frombuffer(raw, dtype, count=msg.width, offset=r * msg.row_step)
                for r in range(msg.height)]
        pts = np.concatenate(rows)
    else:
        pts = np.frombuffer(raw, dtype, count=n)
    if skip_nans and len(pts) and all(k in pts.dtype.names for k in ('x', 'y', 'z')):
        ok = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts = pts[ok]
    return pts


def read_xyz(msg, skip_nans=True):
    """(N, 3) float64."""
    pts = read_cloud(msg, skip_nans)
    return np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)


def write_cloud(template, pts):
    """template(PointCloud2)의 header·fields를 유지하고 점 데이터만 pts(구조화 배열)로 바꾼 새 메시지.

    pts의 dtype은 read_cloud가 만든 것과 같아야 한다 (같은 fields, point_step).
    """
    msg = type(template)()
    msg.header = template.header
    msg.fields = template.fields
    msg.is_bigendian = template.is_bigendian
    msg.point_step = template.point_step
    msg.height = 1
    msg.width = len(pts)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = bool(template.is_dense)
    msg.data = np.ascontiguousarray(pts).tobytes()
    return msg
