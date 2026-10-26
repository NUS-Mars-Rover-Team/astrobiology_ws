from astrobiology_actuators.lights_commands import (
    LIGHTS,
    all_off_commands,
    light_command,
)


def test_lights_dict_keys():
    assert set(LIGHTS.keys()) == {'uv1', 'uv2', 'laser', 'halogen'}


def test_lights_firmware_names():
    assert LIGHTS['uv1'] == 'UV1'
    assert LIGHTS['uv2'] == 'UV2'
    assert LIGHTS['laser'] == 'LAS'
    assert LIGHTS['halogen'] == 'HAL'


def test_light_command_on():
    assert light_command('LAS', True) == 'LAS 1'
    assert light_command('UV1', True) == 'UV1 1'


def test_light_command_off():
    assert light_command('HAL', False) == 'HAL 0'
    assert light_command('UV2', False) == 'UV2 0'


def test_all_off_commands():
    cmds = all_off_commands()
    assert len(cmds) == 4
    assert all(cmd.endswith(' 0') for cmd in cmds)
    names_sent = {cmd.split()[0] for cmd in cmds}
    assert names_sent == {'UV1', 'UV2', 'LAS', 'HAL'}


def test_service_name_pattern():
    for name in LIGHTS:
        svc = f'astrobiology/{name}/set'
        assert svc.startswith('astrobiology/')
        assert svc.endswith('/set')
