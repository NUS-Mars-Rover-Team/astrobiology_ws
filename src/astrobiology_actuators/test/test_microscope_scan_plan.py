from pathlib import Path

import pytest

from astrobiology_actuators.microscope_scan import (
    ScanConfig,
    ScanConfigError,
    dry_run_lines,
    safe_groups,
    scan_moves,
    scan_points,
    validate,
)

CONFIG_YAML = Path(__file__).resolve().parents[1] / 'config' / 'microscope_scan.yaml'


def make_cfg(**overrides):
    d = {
        'grid_offset': {'x': 5.0, 'y': 10.0, 'z': 2.0},
        'microscope_offset': {'x': 80.0, 'y': 70.0, 'z': 47.0},
        'grid_size': {'x': 50.0, 'y': 50.0},
        'travel_max': {'x': 100.0, 'y': 100.0, 'z': 50.0},
    }
    d.update(overrides)
    return ScanConfig.from_dict(d)


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
    xs = {p.local['x'] for p in scan_points(cfg)}
    assert xs == {24.5, 25.5}


def test_target_is_microscope_minus_grid_offset_plus_point():
    p = scan_points(make_cfg(scan_origin={'x': 10.0, 'y': 20.0}))[5]   # ix=1, iy=0, z=0
    assert p.local == {'x': 11.0, 'y': 20.0, 'z': 0.0}
    # 80 - (5 + 11), 70 - (10 + 20), 47 - (2 + 0)
    assert p.target == {'x': 64.0, 'y': 40.0, 'z': 45.0}


def test_z_levels_lower_the_target():
    zs = [p.target['z'] for p in scan_points(make_cfg())[:5]]
    assert zs == [45.0, 35.0, 25.0, 15.0, 5.0]


def test_validate_accepts_default_geometry():
    cfg = make_cfg()
    validate(cfg, scan_points(cfg))


@pytest.mark.parametrize('overrides, match', [
    ({'scan_origin': {'x': 49.5, 'y': 0.0}}, 'outside the 50 mm grid'),
    ({'microscope_offset': {'x': 20.0, 'y': 70.0, 'z': 47.0}}, 'target x=.* outside travel'),
    ({'travel_max': {'x': 100.0, 'y': 100.0, 'z': 40.0}}, 'target z=.* outside travel'),
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
        ScanConfig.from_dict({'grid_offset': {'x': 0, 'y': 0, 'z': 0},
                              'microscope_offset': {'x': 0, 'y': 0, 'z': 0},
                              'grid_size': {'x': 50, 'y': 50}})


def test_bad_axis_value_is_a_config_error():
    with pytest.raises(ScanConfigError, match='grid_offset'):
        make_cfg(grid_offset={'x': 0, 'y': 'a'})


def test_shipped_yaml_loads_and_validates():
    cfg = ScanConfig.from_yaml(CONFIG_YAML)
    assert cfg.calibrated is False      # placeholders until measured
    validate(cfg, scan_points(cfg))


def test_safe_groups():
    assert safe_groups(0.0, {'x': 1, 'y': 2, 'z': 5}) == [{'x': 1, 'y': 2}, {'z': 5}]
    assert safe_groups(9.0, {'x': 1, 'y': 2, 'z': 5}) == [{'z': 5}, {'x': 1, 'y': 2}]
    assert safe_groups(0.0, {'y': 2}) == [{'y': 2}]


def test_scan_moves_sequence():
    points = scan_points(make_cfg())
    stops = scan_moves(points)
    assert len(stops) == 21
    first = points[0].target
    # home -> first point: travel XY at the clear Z, then lower
    assert stops[0][2] == [{'x': first['x'], 'y': first['y']}, {'z': first['z']}]
    # stepping through the Z levels at one XY position: Z only
    assert stops[1][2] == [{'z': points[1].target['z']}]
    # next XY position: Z back to the first level, then XY
    nxt = points[5].target
    assert stops[5][2] == [{'z': nxt['z']}, {'x': nxt['x'], 'y': nxt['y']}]
    assert nxt['z'] == first['z']
    # last point -> home: Z toward 0 first, then XY
    assert stops[-1][0] == 'home' and stops[-1][1] is None
    assert stops[-1][2] == [{'z': 0.0}, {'x': 0.0, 'y': 0.0}]


def test_scan_moves_no_return():
    assert scan_moves(scan_points(make_cfg()), return_home=False)[-1][0] == '#19'


def test_dry_run_moves():
    cfg = make_cfg()
    lines = dry_run_lines(cfg, scan_points(cfg))
    moves = lines[lines.index('moves (after g28, from 0/0/0):') + 1:]
    assert len(moves) == 21
    assert moves[0].strip() == '#0: g1 x 50.5 5 10 + g1 y 35.5 5 10  then  g1 z 45 5 10'
    assert moves[1].strip() == '#1: g1 z -10 5 10'
    assert moves[5].strip() == '#5: g1 z 40 5 10  then  g1 x -1 5 10'
    assert moves[-1].strip().startswith('home: g1 z -5 5 10  then')
