"""Physical simulation of production corexy + HMI + asynchronous task scheduling."""
import ctypes
from pathlib import Path
import sys

cases = [
    'near-45-degree click follows one short diagonal with bounded path error',
    'FD ACK and model target cannot report arrival',
    'only latest pending click follows the completed task',
    'manual stop clears pending and requires recalibration',
    'current replies cannot mask missing motor B positions',
    'overcurrent protection remains active on a short move',
    'missing motor B FD ACK prevents any broadcast',
    'E2 rejection prevents any broadcast',
    'wrong direction stops without an automatic retreat',
    'one stalled motor stops both and clears queued target',
    'fresh but motionless positions cannot report completion',
    'same-position request still verifies fresh settled feedback',
    'cumulative creeping within tolerance resets the settling window',
    'missing final stop ACK prevents task completion',
    'missing preflight positions prevents motor startup',
    'model/actual mismatch at preflight revokes calibration',
    'revoking calibration in motion stops and clears queue',
    '200mA and absent B current do not invent a collision',
    'raw motion calls cannot override a tracked target',
    'single motor diagnostic verifies arrival and revokes calibration',
    'cached target-looking position cannot satisfy arrival',
    'self-test can rebuild calibration after a normal-motion fault',
    'position lifecycle survives tick counter wraparound',
    'actual HMI coordinate frame drives the mapped physical XY',
    'uncalibrated HMI frame cannot start a motor',
    'first post-zero position retains real displacement',
    'bus fault latches stop and forbids queued restart',
    'fractional Y pulse permits the next independent line task',
    'fractional X pulse permits completion of a direct line task',
]
dll = ctypes.CDLL(str(Path(__file__).with_name('boundary_harness.dll')))
dll.run_motion_case.argtypes = [ctypes.c_int]
dll.run_motion_case.restype = ctypes.c_int
failed = []
for i, name in enumerate(cases):
    ok = dll.run_motion_case(i)
    print(('PASS' if ok else 'FAIL') + ': ' + name)
    if not ok:
        failed.append((i, name))
print(f'{len(cases)-len(failed)} / {len(cases)} passed')
app_cases = [
    'Bluetooth initialization arms USART1 IRQ reception',
    'Bluetooth XY commands return immediately and report completion later',
    'S cancels current and pending targets including a queued command suffix',
    'S at final arrival clears pending target before it starts',
    'overlong command cannot execute its trailing suffix',
    'Bluetooth ring overflow stops rather than interpreting truncated commands',
    'Bluetooth UART error rearms RX and stops active motion',
    'historical CAN errors do not repeatedly revoke calibration',
    'new CAN send failure stops active and pending targets',
    'bus-off recovery stops and requires calibration',
    'same cumulative CAN error is recovered only once',
    'KEY1 cannot start a self-test over a tracked motion',
    'KEY2 uses asynchronous feedback-based arrival',
    'read-only Bluetooth commands do not recover or interrupt motion',
    'overflowing or fractional/garbage distance cannot start a motor',
    'ZERO during motion stops and clears pending clicks',
    'Bluetooth does not issue an early MOVE/DONE report',
    'S received by interrupt during final ACK wait precedes pending target',
    'USART2 interrupt through App_Poll drives mapped XY and reports arrival',
    'manual stop ACK failure latches a motion fault',
    'ZERO cannot clear counters after an unconfirmed stop',
    'four separate screen presses each complete then accept the next target',
    'twelve screen presses with fractional calibrated spans include all corners',
]
dll.run_app_case.argtypes = [ctypes.c_int]
dll.run_app_case.restype = ctypes.c_int
for i, name in enumerate(app_cases):
    ok = dll.run_app_case(i)
    print(('PASS' if ok else 'FAIL') + ': ' + name)
    if not ok:
        failed.append(('app', i, name))
print(f'{len(cases)+len(app_cases)-len(failed)} / {len(cases)+len(app_cases)} total passed')
line_cases = [f'quadrant/slope {i}: path stays on the single connecting line' for i in range(8)]
line_cases += [
    '45 degrees +X +Y: static B requires no FD ACK',
    '45 degrees +X -Y: static A requires no FD ACK',
    '45 degrees -X +Y: static A requires no FD ACK',
    '45 degrees -X -Y: static B requires no FD ACK',
    'near-45 degrees below: tiny slow-axis move with integer RPM',
    'near-45 degrees above: tiny reverse slow-axis move with integer RPM',
    'sub-millimeter diagonal remains finite and completes',
    'short diagonal uses a shared triangular ramp',
    'FD ACK loss mid-stream forbids the next broadcast and cancels pending',
    'transverse physical displacement faults with BR=4',
    'static motor position feedback remains mandatory',
    'received S takes priority over a due trajectory update',
    'expired trajectory deadline cannot send a late target',
    'stream sends finite absolute targets with bounded endpoint',
]
dll.run_line_case.argtypes=[ctypes.c_int]
dll.run_line_case.restype=ctypes.c_int
for i,name in enumerate(line_cases):
    ok=dll.run_line_case(i)
    print(('PASS' if ok else 'FAIL')+': '+name)
    if not ok: failed.append(('line',i,name))
print(f'{len(cases)+len(app_cases)+len(line_cases)-len(failed)} / {len(cases)+len(app_cases)+len(line_cases)} all motion cases passed')
marker_cases = [
    'no position packets cannot become valid actual coordinates',
    'one missing motor position cannot become valid actual coordinates',
    'signed real encoder packets convert to the measured XY',
    'freshness expires after 300ms independently for each motor',
    'actual snapshot freshness survives tick wraparound',
    'App reports moving actual position rather than commanded endpoint',
    'current replies cannot keep a stale green position valid',
    'S cancels pending motion and subsequent green reports are invalid',
    'motion fault hides green marker and clears pending motion',
    'periodic display requests coexist with arrival and stop after requests cease',
]
dll.run_marker_case.argtypes = [ctypes.c_int]
dll.run_marker_case.restype = ctypes.c_int
for i,name in enumerate(marker_cases):
    ok=dll.run_marker_case(i)
    print(('PASS' if ok else 'FAIL')+': '+name)
    if not ok: failed.append(('marker',i,name))
total=len(cases)+len(app_cases)+len(line_cases)+len(marker_cases)
print(f'{total-len(failed)} / {total} all motion/display cases passed')
sys.exit(bool(failed))
