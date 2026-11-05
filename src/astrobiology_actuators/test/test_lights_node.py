from astrobiology_actuators.lights_commands import (
    LIGHTS,
    all_off_commands,
    light_command,
)


def test_lights_dict_keys():
    assert set(LIGHTS.keys()) == {'uvm', 'uvc', 'laser', 'halogen'}


def test_lights_firmware_names():
    # Must match the commands in firmware/LEDdrill.ino
    assert LIGHTS['uvm'] == 'uvm'
    assert LIGHTS['uvc'] == 'uvc'
    assert LIGHTS['laser'] == 'laser'
    assert LIGHTS['halogen'] == 'halogen'


def test_light_command_on():
    assert light_command('laser', True) == 'laser on'
    assert light_command('uvm', True) == 'uvm on'


def test_light_command_off():
    assert light_command('halogen', False) == 'halogen off'
    assert light_command('uvc', False) == 'uvc off'


def test_all_off_commands():
    cmds = all_off_commands()
    assert len(cmds) == 4
    assert all(cmd.endswith(' off') for cmd in cmds)
    names_sent = {cmd.split()[0] for cmd in cmds}
    assert names_sent == {'uvm', 'uvc', 'laser', 'halogen'}


def test_service_name_pattern():
    for name in LIGHTS:
        svc = f'astrobiology/{name}/set'
        assert svc.startswith('astrobiology/')
        assert svc.endswith('/set')
