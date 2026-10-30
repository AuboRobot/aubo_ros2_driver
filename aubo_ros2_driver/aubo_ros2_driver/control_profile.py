"""Select by remote controller/model family, never by the ROS host CPU."""
def resolve_update_rate(robot_type, profile, override, configured_rate):
    if profile not in ('auto', 'es', 'legacy', 'custom'):
        raise ValueError('controller_profile must be auto, es, legacy or custom')
    selected = 'es' if profile == 'auto' and robot_type.startswith('aubo_ES') else profile
    if override:
        rate = int(override)
    elif selected == 'es':
        rate = 100
    elif selected == 'legacy':
        rate = 200
    else:
        rate = int(configured_rate)
    if not 25 <= rate <= 500:
        raise ValueError('update_rate must be an integer between 25 and 500 Hz')
    return rate


def resolve_servo_period(value, rate):
    """Explicit auto pairs servoJoint execution time to the nominal call rate."""
    import math
    period = 1.0 / rate if value == 'auto' else float(value)
    if not math.isfinite(period) or not .002 <= period <= .02:
        raise ValueError('servo_period must be auto or a finite value in .002..02s')
    if 1.0 / rate >= 5.0 * period:
        raise ValueError('control period must leave margin within the 5*t watchdog')
    return period
