"""Run the compiled host DLL containing the production corexy.c."""
import ctypes
from pathlib import Path
import sys

cases = [
    'late contact after long travel, no current rise',
    'current over 200mA triggers despite reported encoder motion',
    'stale position while current/status replies continue',
    'one stationary motor, one moving motor',
    'start directly against wall',
    'same current packet cannot confirm repeatedly',
    'signed cumulative multi-turn position after packet gap',
    'axis motion uses equal RPM despite float coordinate residue',
    'manual stop disarms boundary detector',
    'full four-wall calibration with no B current replies',
    'retreat not executed must fail calibration',
    'full sweep starting close to corner',
    'malformed current packets are rejected',
    'current rise corroborated by actual slowdown',
    'missing positions prevents motion at preflight',
    '200ms feedback intervals remain valid',
    'single brief pause does not establish a wall',
    'diagonal probe refused before motion',
    'normal short move disarms at completion',
    'missing motor B position-command ACK prevents synchronous start',
    'position-reached response cannot substitute command ACK',
    'complete direction sequence with current-triggered retreats and bottom-left finish',
    '200mA is allowed, 201mA on B alone triggers after three new samples',
    'low sample resets confirmation, isolated spikes cannot retreat',
    'reproduce old unload-then-position sequence rejected with E2',
    'disabled motors are enabled and stopped before accepting retreat targets',
    'missing motor B stop ACK prevents synchronized movement',
    'driver rejects a target with E2: report failure without broadcasting',
    'delayed retreat positions recover and all four legs finish with acceleration',
    'all four legs complete with vendor acceleration and deceleration',
    'permanent loss of motor B retreat positions must stop with BR=3',
    'permanent loss of both retreat positions must stop with BR=3',
    'retreat toward the wall must fail actual displacement verification',
    '20ms polling stops sustained overcurrent contact within 80ms after cruise',
    'wall-loaded encoder reference must not misjudge a correctly executed 5mm retreat',
    'missing stopped-position baseline must prevent retreat startup',
]
dll = ctypes.CDLL(str(Path(__file__).with_name('boundary_harness.dll')))
dll.run_case.argtypes = [ctypes.c_int]
dll.run_case.restype = ctypes.c_int
failed = []
for i, name in enumerate(cases):
    ok = dll.run_case(i)
    print(('PASS' if ok else 'FAIL') + ': ' + name)
    if not ok:
        failed.append(name)
print('{} / {} passed'.format(len(cases) - len(failed), len(cases)))
dll.get_simulated_contact_to_stop_ms.restype = ctypes.c_uint32
print('Simulated contact-to-stop delay: {} ms (not a hardware measurement)'.format(
    dll.get_simulated_contact_to_stop_ms()))
sys.exit(bool(failed))
