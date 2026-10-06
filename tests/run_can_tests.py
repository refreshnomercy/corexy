import ctypes
from pathlib import Path
import sys

cases = [f'{kind} on mailbox {mailbox}: queued positions must resume' for kind in ('arbitration lost', 'transmission error') for mailbox in range(3)]
cases += [
    'queue reset retains old mailbox ownership; replacement FD starts at fragment zero',
    'failed discarded command cannot discard replacement Stop commands',
    'failed FD drops entire command instead of transmitting remainder',
    'mailbox busy recovers through foreground pump',
    'initial HAL Add failure recovers through foreground pump',
    'abort on mailbox 0 releases sender',
    'abort on mailbox 1 releases sender',
    'abort on mailbox 2 releases sender',
    'unrelated mailbox callbacks cannot advance owned command',
    'enqueue reset pump and error preserve caller interrupt mask',
    'second fragment HAL failure never replays partial FD',
    'invalid command length rejected before fragment packing',
    'absolute finite target with native ACC255 and sync flag uses correct CAN fragments',
]
dll = ctypes.CDLL(str(Path(__file__).with_name('can_transport_harness.dll')))
dll.transport_case.argtypes = [ctypes.c_int]
dll.transport_case.restype = ctypes.c_int
failed = []
for i, name in enumerate(cases):
    ok = dll.transport_case(i)
    print(('PASS' if ok else 'FAIL') + ': ' + name)
    if not ok:
        failed.append(name)
print(f'{len(cases)-len(failed)} / {len(cases)} passed')
sys.exit(bool(failed))
