import csv
import io
from pathlib import Path
import subprocess

import pytest

from astrobiology_actuators import microscope_scan
from astrobiology_actuators.microscope_scan import (
    CSV_FIELDS,
    ScanConfig,
    ScanConfigError,
    capture_image,
    dry_run_lines,
    ffmpeg_command,
    run_scan,
    safe_groups,
    scan_moves,
    scan_points,
    validate,
)

CONFIG_YAML = Path(__file__).resolve().parents[1] / 'config' / 'microscope_scan.yaml'


def make_cfg(**overrides):
    d = {
        'grid_offset': {'x': 5.0, 'y': 10.0},
        'microscope_offset': {'x': 80.0, 'y': 70.0},
        'grid_size': {'x': 50.0, 'y': 50.0},
        'travel_max': {'x': 100.0, 'y': 100.0, 'z': 50.0},
        'z_offset_mm': 2.0,
    }
    d.update(overrides)
    return ScanConfig.from_dict(d)


# -- points --------------------------------------------------------------------

def test_twenty_points_xy_outer_z_inner():
    points = scan_points(make_cfg())
    assert len(points) == 20
    assert [p.index for p in points] == list(range(20))
    assert [p.iz for p in points] == list(range(5)) * 4
    xy = [(p.ix, p.iy) for p in points[::5]]
    assert xy == [(0, 0), (1, 0), (1, 1), (0, 1)]
    for start in range(0, 20, 5):
        assert len({(p.ix, p.iy) for p in points[start:start + 5]}) == 1


def test_xy_hops_are_one_pitch():
    points = scan_points(make_cfg())
    for a, b in zip(points[::5], points[5::5]):
        hop = sorted(abs(a.target[k] - b.target[k]) for k in 'xy')
        assert hop == [0.0, 1.0]


def test_default_origin_centres_the_pattern():
    cfg = make_cfg()
    assert cfg.origin() == {'x': 24.5, 'y': 24.5}
    assert {p.local['x'] for p in scan_points(cfg)} == {24.5, 25.5}


def test_target_formula():
    p = scan_points(make_cfg(scan_origin={'x': 10.0, 'y': 20.0}))[6]   # ix=1, iy=0, lvl 1
    assert p.local == {'x': 11.0, 'y': 20.0, 'z': 10.0}
    # X, Y = microscope - (grid + p): 80 - (5 + 11), 70 - (10 + 20); Z = 2 + 10
    assert p.target == {'x': 64.0, 'y': 40.0, 'z': 12.0}


def test_z_targets_are_offset_plus_level():
    zs = [p.target['z'] for p in scan_points(make_cfg())[:5]]
    assert zs == [2.0, 12.0, 22.0, 32.0, 42.0]


def test_travel_min_defaults_to_zero():
    assert make_cfg().travel_min == {'x': 0.0, 'y': 0.0, 'z': 0.0}


# -- validation ------------------------------------------------------------------

def test_validate_accepts_default_geometry():
    cfg = make_cfg()
    validate(cfg, scan_points(cfg))


@pytest.mark.parametrize('overrides, match', [
    ({'scan_origin': {'x': 49.5, 'y': 0.0}}, 'outside the 50 mm grid'),
    ({'microscope_offset': {'x': 20.0, 'y': 70.0}}, 'target x=.* outside travel'),
    ({'travel_min': {'x': 60.0, 'y': 0.0, 'z': 0.0}}, 'target x=.* outside travel 60'),
    ({'travel_max': {'x': 100.0, 'y': 100.0, 'z': 40.0}}, 'target z=.* outside travel'),
    ({'z_offset_mm': -1.0}, 'target z=-1.000'),
    ({'speed_mm_s': 40.0}, 'speed_mm_s'),
    ({'accel_mm_s2': 0.0}, 'accel_mm_s2'),
    ({'pitch_mm': 0.0}, 'pitch_mm'),
])
def test_validate_rejects(overrides, match):
    cfg = make_cfg(**overrides)
    with pytest.raises(ScanConfigError, match=match):
        validate(cfg, scan_points(cfg))


def test_missing_key_is_a_config_error():
    with pytest.raises(ScanConfigError, match='travel_max'):
        ScanConfig.from_dict({'grid_offset': {'x': 0, 'y': 0},
                              'microscope_offset': {'x': 0, 'y': 0},
                              'grid_size': {'x': 50, 'y': 50}})


def test_bad_axis_value_is_a_config_error():
    with pytest.raises(ScanConfigError, match='grid_offset'):
        make_cfg(grid_offset={'x': 0, 'y': 'a'})


def test_shipped_yaml_is_the_trial_scan():
    cfg = ScanConfig.from_yaml(CONFIG_YAML)
    points = scan_points(cfg)
    validate(cfg, points)
    assert len(points) == 20
    centre = {k: sum(p.target[k] for p in points) / len(points) for k in 'xy'}
    assert centre == {'x': 20.0, 'y': 20.0}
    assert max(p.target['z'] for p in points) <= cfg.travel_max['z']


# -- move sequence -----------------------------------------------------------------

def test_safe_groups():
    assert safe_groups(0.0, {'x': 1, 'y': 2, 'z': 5}) == [{'x': 1, 'y': 2}, {'z': 5}]
    assert safe_groups(9.0, {'x': 1, 'y': 2, 'z': 5}) == [{'z': 5}, {'x': 1, 'y': 2}]
    assert safe_groups(0.0, {'y': 2}) == [{'y': 2}]


def test_scan_moves_keep_z_at_zero_while_xy_moves():
    points = scan_points(make_cfg())
    stops = scan_moves(points)
    assert len(stops) == 21
    for i in (0, 5, 10, 15):        # each new XY position
        t = points[i].target
        assert stops[i][2] == [{'z': 0.0}, {'x': t['x'], 'y': t['y']}, {'z': t['z']}]
    assert stops[1][2] == [{'z': points[1].target['z']}]
    assert stops[-1][0] == 'home' and stops[-1][1] is None
    assert stops[-1][2] == [{'z': 0.0}, {'x': 0.0, 'y': 0.0}]


def test_scan_moves_no_return():
    assert scan_moves(scan_points(make_cfg()), return_home=False)[-1][0] == '#19'


def test_dry_run_moves():
    cfg = make_cfg()
    lines = dry_run_lines(cfg, scan_points(cfg))
    moves = lines[lines.index('moves (after g28, from 0/0/0):') + 1:]
    assert len(moves) == 21
    assert moves[0].strip() == '#0: g1 x 50.5 5 10 + g1 y 35.5 5 10  then  g1 z 2 5 10'
    assert moves[1].strip() == '#1: g1 z 10 5 10'
    assert moves[5].strip() == '#5: g1 z -42 5 10  then  g1 x -1 5 10  then  g1 z 2 5 10'
    assert moves[-1].strip() == 'home: g1 z -42 5 10  then  g1 x -50.5 5 10 + g1 y -34.5 5 10'


# -- live scan (fake gantry) -------------------------------------------------------

class FakeGantry:
    """Records every call; pos() reports the last commanded position."""

    def __init__(self, pos_error=0.0):
        self.calls = []
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.pos_error = pos_error

    def home(self, axes=()):
        self.calls.append(('home',))
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def move_to(self, target, speed, accel):
        assert not ('z' in target and len(target) > 1), 'Z moved with XY'
        if any(axis in target for axis in 'xy'):
            assert self.position['z'] == 0.0, 'XY moved with Z away from 0'
        self.calls.append(('move', dict(target)))
        self.position.update(target)

    def pos(self):
        return {k: v + self.pos_error for k, v in self.position.items()}


@pytest.fixture
def no_settle(monkeypatch):
    monkeypatch.setattr(microscope_scan.time, 'sleep', lambda s: None)


def test_run_scan_logs_every_point(no_settle):
    cfg = make_cfg()
    points = scan_points(cfg)
    gantry, out, logs = FakeGantry(), io.StringIO(), []
    run_scan(gantry, cfg, points, out, log=logs.append)
    rows = list(csv.DictReader(io.StringIO(out.getvalue())))
    assert list(rows[0]) == list(CSV_FIELDS)
    assert len(rows) == 20
    for row, p in zip(rows, points):
        assert int(row['index']) == p.index
        assert float(row['pos_z']) == pytest.approx(p.target['z'])
        assert float(row['error_mm']) == 0.0
    assert gantry.calls[0] == ('home',)
    assert gantry.position == {'x': 0.0, 'y': 0.0, 'z': 0.0}
    assert not any('WARNING' in line for line in logs)


def test_run_scan_skip_home_and_no_return(no_settle):
    cfg = make_cfg()
    gantry = FakeGantry()
    run_scan(gantry, cfg, scan_points(cfg), io.StringIO(),
             home=False, return_home=False, log=lambda m: None)
    assert ('home',) not in gantry.calls
    assert gantry.position == scan_points(cfg)[-1].target


def test_run_scan_warns_when_off_target(no_settle):
    cfg = make_cfg()
    logs = []
    run_scan(FakeGantry(pos_error=0.01), cfg, scan_points(cfg), io.StringIO(),
             log=logs.append)
    assert sum('WARNING off target' in line for line in logs) == 20


# -- CLI -------------------------------------------------------------------------

def test_cli_refuses_uncalibrated_config(tmp_path, capsys):
    cfg_file = tmp_path / 'scan.yaml'
    cfg_file.write_text(CONFIG_YAML.read_text().replace('calibrated: true',
                                                        'calibrated: false'))
    assert microscope_scan.main(['scan', '--config', str(cfg_file)]) == 1
    assert 'calibrated is false' in capsys.readouterr().err


def test_cli_dry_run_z_levels_override(capsys):
    assert microscope_scan.main(['scan', '--dry-run', '--config', str(CONFIG_YAML),
                                 '--z-levels', '0', '2']) == 0
    assert '8 points, Z levels 0, 2 mm' in capsys.readouterr().out


def test_cli_rejects_levels_beyond_travel(capsys):
    assert microscope_scan.main(['scan', '--dry-run', '--config', str(CONFIG_YAML),
                                 '--z-levels', '0', '10']) == 1
    assert 'outside travel' in capsys.readouterr().err


# -- camera ------------------------------------------------------------------------

def test_camera_defaults_without_camera_block():
    cfg = make_cfg()
    assert cfg.camera_enabled is True
    assert cfg.camera_device == '/dev/video2'
    assert cfg.image_path(3) == Path('~/Pictures/test3.jpg').expanduser()


def test_camera_block_is_read():
    cfg = make_cfg(camera={'enabled': False, 'device': '/dev/video0',
                           'image_dir': '/tmp/shots', 'image_prefix': 'img',
                           'skip_frames': 0})
    assert (cfg.camera_enabled, cfg.camera_device, cfg.skip_frames) == (False, '/dev/video0', 0)
    assert cfg.image_path(20) == Path('/tmp/shots/img20.jpg')


def test_ffmpeg_command():
    assert ffmpeg_command('/dev/video2', 'test1.jpg') == [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'v4l2', '-i', '/dev/video2', '-frames:v', '1', 'test1.jpg']
    cmd = ffmpeg_command('/dev/video2', 'test1.jpg', skip_frames=5)
    assert cmd[cmd.index('-vf') + 1] == r'select=gte(n\,5)'


class FakeRun:
    def __init__(self, returncode=0, stderr='', write=True, exc=None):
        self.returncode, self.stderr, self.write, self.exc = returncode, stderr, write, exc
        self.cmds = []

    def __call__(self, cmd, **kwargs):
        self.cmds.append(cmd)
        if self.exc:
            raise self.exc
        if self.write and self.returncode == 0:
            Path(cmd[-1]).write_bytes(b'jpg')
        return subprocess.CompletedProcess(cmd, self.returncode, '', self.stderr)


def test_capture_image_success_replaces_old_file(tmp_path, monkeypatch):
    path = tmp_path / 'test1.jpg'
    path.write_bytes(b'old')
    monkeypatch.setattr(microscope_scan.subprocess, 'run', FakeRun())
    assert capture_image('/dev/video2', path, 5) is None
    assert path.read_bytes() == b'jpg'


@pytest.mark.parametrize('fake, match', [
    (FakeRun(returncode=1, stderr='junk\n/dev/video2: Device or resource busy\n'),
     'exit 1: /dev/video2: Device or resource busy'),
    (FakeRun(exc=subprocess.TimeoutExpired('ffmpeg', 20)), 'timed out'),
    (FakeRun(exc=FileNotFoundError()), 'ffmpeg not found'),
    (FakeRun(write=False), 'wrote no file'),
])
def test_capture_image_failures_return_a_reason(tmp_path, monkeypatch, fake, match):
    path = tmp_path / 'test1.jpg'
    path.write_bytes(b'stale picture from the last run')
    monkeypatch.setattr(microscope_scan.subprocess, 'run', fake)
    reason = capture_image('/dev/video2', path)
    assert reason is not None and match in reason
    assert not path.exists()


def test_run_scan_captures_at_every_point(no_settle):
    cfg = make_cfg()
    calls = []

    def capture(n):
        calls.append(n)
        return Path(f'/tmp/test{n}.jpg'), None

    out, logs = io.StringIO(), []
    run_scan(FakeGantry(), cfg, scan_points(cfg), out, capture=capture, log=logs.append)
    assert calls == list(range(1, 21))
    rows = list(csv.DictReader(io.StringIO(out.getvalue())))
    assert [row['image'] for row in rows] == [f'/tmp/test{n}.jpg' for n in range(1, 21)]
    assert '-> test1.jpg' in logs[1]


def test_run_scan_keeps_going_when_capture_fails(no_settle):
    cfg = make_cfg()
    gantry, out, logs = FakeGantry(), io.StringIO(), []
    run_scan(gantry, cfg, scan_points(cfg), out,
             capture=lambda n: (Path(f'test{n}.jpg'), 'ffmpeg exit 1: busy'), log=logs.append)
    rows = list(csv.DictReader(io.StringIO(out.getvalue())))
    assert len(rows) == 20 and all(row['image'] == '' for row in rows)
    assert sum('WARNING image failed: ffmpeg exit 1: busy' in line for line in logs) == 20
    assert gantry.position == {'x': 0.0, 'y': 0.0, 'z': 0.0}


def test_dry_run_lists_image_names():
    cfg = make_cfg()
    lines = dry_run_lines(cfg, scan_points(cfg))
    assert lines[0].endswith('image')
    assert lines[1].endswith('test1.jpg') and lines[20].endswith('test20.jpg')


class GantryReached(Exception):
    pass


def _no_gantry(*args, **kwargs):
    raise GantryReached


def test_cli_missing_camera_is_refused_before_motion(monkeypatch, capsys):
    monkeypatch.setattr(microscope_scan, 'Gantry', _no_gantry)
    monkeypatch.setattr(microscope_scan.shutil, 'which', lambda name: '/usr/bin/ffmpeg')
    assert microscope_scan.main(['scan', '--config', str(CONFIG_YAML),
                                 '--camera', '/dev/no_such_video']) == 1
    assert 'camera /dev/no_such_video not found' in capsys.readouterr().err


def test_cli_no_images_skips_camera_checks(monkeypatch, capsys):
    monkeypatch.setattr(microscope_scan, 'Gantry', _no_gantry)

    def which(name):
        raise AssertionError('camera checks should be skipped')

    monkeypatch.setattr(microscope_scan.shutil, 'which', which)
    with pytest.raises(GantryReached):
        microscope_scan.main(['scan', '--config', str(CONFIG_YAML), '--no-images'])
    assert 'images: off' in capsys.readouterr().out
