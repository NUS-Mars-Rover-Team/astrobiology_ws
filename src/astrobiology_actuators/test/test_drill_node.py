from astrobiology_actuators.drill_commands import START, STOP


def test_drill_firmware_commands():
    # Must match the commands in firmware/LAdrill.ino
    assert START == 'b'
    assert STOP == 's'


def test_service_name_pattern():
    for action in ('start', 'stop'):
        svc = f'astrobiology/drill/{action}'
        assert svc.startswith('astrobiology/drill/')
