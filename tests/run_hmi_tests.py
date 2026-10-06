"""Exercise the actual UART driver with split/noisy/queued frames and RX errors."""
import ctypes
import pathlib
import struct
import unittest

driver = ctypes.CDLL(str(pathlib.Path(__file__).with_name('hmi_harness.dll')))
driver.host_receive.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
driver.host_error.argtypes = [ctypes.c_uint8]
driver.host_take_ok.restype = ctypes.c_uint8
driver.host_length.restype = ctypes.c_uint16
driver.host_rearms.restype = ctypes.c_uint32
FRAME = bytes.fromhex('55 6F 6B FF FF FF')
driver.host_take_event.argtypes = [ctypes.POINTER(ctypes.c_uint16)] * 2
driver.host_take_event.restype = ctypes.c_uint8
driver.host_set_area.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_uint8]
driver.host_output.restype = ctypes.c_char_p
driver.host_map_touch.argtypes = [ctypes.c_uint16, ctypes.c_uint16,
                                 ctypes.c_float, ctypes.c_float,
                                 ctypes.POINTER(ctypes.c_float),
                                 ctypes.POINTER(ctypes.c_float)]
driver.host_map_touch.restype = ctypes.c_uint8
driver.host_targets.restype = ctypes.c_uint32
driver.host_target_x.restype = driver.host_target_y.restype = ctypes.c_float
driver.host_set_tick.argtypes = [ctypes.c_uint32]
driver.host_set_actual.argtypes = [ctypes.c_float, ctypes.c_float,
                                 ctypes.c_uint32, ctypes.c_uint32,
                                 ctypes.c_uint8, ctypes.c_uint8]
driver.host_tx_data.restype = ctypes.c_void_p
driver.host_tx_length.restype = ctypes.c_uint16
driver.host_tx_starts.restype = driver.host_tx_attempts.restype = ctypes.c_uint32
driver.host_tx_complete.argtypes = [ctypes.c_uint8]
driver.host_tx_result.argtypes = [ctypes.c_int]
driver.host_send_string.argtypes = driver.host_send_text.argtypes = [ctypes.c_char_p]
driver.host_send_string.restype = driver.host_send_text.restype = ctypes.c_uint8
driver.host_send_value.argtypes = [ctypes.c_int]
driver.host_send_value.restype = ctypes.c_uint8
driver.host_send_position.argtypes = [ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint8]
driver.host_send_position.restype = ctypes.c_uint8
driver.host_map_actual.argtypes = [ctypes.c_float] * 4 + [ctypes.POINTER(ctypes.c_uint16)] * 2
driver.host_map_actual.restype = ctypes.c_uint8
POSITION_REQUEST = bytes.fromhex('55 50 01 FF FF FF')


def tx_bytes():
    return ctypes.string_at(driver.host_tx_data(), driver.host_tx_length())


def position_reply(px, py, valid):
    return b''.join(command.encode('ascii') + b'\xff\xff\xff' for command in [
        'page0.gnew.val=0', f'page0.gpx.val={px}', f'page0.gpy.val={py}',
        f'page0.gok.val={valid}', 'page0.gnew.val=1'])


def request_position():
    receive(POSITION_REQUEST)
    driver.host_poll()
    driver.host_publish()


def map_actual(x, y, xm=161.0, ym=144.0):
    px, py = ctypes.c_uint16(999), ctypes.c_uint16(999)
    ok = driver.host_map_actual(x, y, xm, ym, ctypes.byref(px), ctypes.byref(py))
    return ok, px.value, py.value


def touch(x, y):
    return b'\x55\x54' + struct.pack('<HH', x, y) + b'\xff\xff\xff'


def take_event():
    x, y = ctypes.c_uint16(), ctypes.c_uint16()
    event = driver.host_take_event(ctypes.byref(x), ctypes.byref(y))
    return event, x.value, y.value


def output():
    return driver.host_output().decode('ascii')


def map_touch(px, py, xm=161.0, ym=144.0):
    x, y = ctypes.c_float(-1), ctypes.c_float(-1)
    ok = driver.host_map_touch(px, py, xm, ym, ctypes.byref(x), ctypes.byref(y))
    return ok, x.value, y.value


def receive(data, port=2):
    for byte in data:
        driver.host_receive(byte, port)


class HmiReceiveTests(unittest.TestCase):
    def setUp(self):
        driver.host_init()

    def test_initialization_selects_pa2_pa3_uart_and_irq(self):
        self.assertEqual(driver.host_rx_port(), 2)
        self.assertEqual(driver.host_enabled_irq(), 38)

    def test_one_press_one_reply(self):
        receive(FRAME)
        self.assertEqual(driver.host_take_ok(), 1)
        self.assertEqual(driver.host_take_ok(), 0)
        self.assertEqual(driver.host_rearms(), 7)

    def test_every_partial_frame_waits(self):
        for byte in FRAME[:-1]:
            receive(bytes([byte]))
            self.assertEqual(driver.host_take_ok(), 0)
        receive(FRAME[-1:])
        self.assertEqual(driver.host_take_ok(), 1)

    def test_boot_message_and_bad_frame_resync(self):
        receive(bytes.fromhex('00 00 00 FF FF FF 88 FF FF FF'))
        receive(bytes.fromhex('55 6F 6B FF FF 00') + FRAME)
        self.assertEqual(driver.host_take_ok(), 1)
        self.assertEqual(driver.host_take_ok(), 0)

    def test_consecutive_presses(self):
        receive(FRAME * 50)
        self.assertEqual(sum(driver.host_take_ok() for _ in range(51)), 50)

    def test_frame_crosses_ring_end(self):
        receive(b'\x00' * 498)
        self.assertEqual(driver.host_take_ok(), 0)
        receive(FRAME)
        self.assertEqual(driver.host_take_ok(), 1)

    def test_other_uart_is_ignored(self):
        for port in (1, 3):
            receive(FRAME, port=port)
            driver.host_error(port)
        self.assertEqual(driver.host_length(), 0)
        self.assertEqual(driver.host_rearms(), 1)

    def test_overrun_restarts_and_discards_partial_frame(self):
        receive(FRAME[:3])
        driver.host_error(2)
        self.assertEqual(driver.host_length(), 0)
        self.assertEqual(driver.host_rearms(), 5)
        receive(FRAME)
        self.assertEqual(driver.host_take_ok(), 1)

    def test_full_ring_recovers(self):
        receive(b'\x00' * 600)
        self.assertEqual(driver.host_length(), 500)
        self.assertEqual(driver.host_take_ok(), 0)
        receive(FRAME)
        self.assertEqual(driver.host_take_ok(), 1)

    def test_dequeue_preserves_interrupt_mask(self):
        receive(FRAME)
        driver.host_set_mask(1)
        self.assertEqual(driver.host_take_ok(), 1)
        self.assertEqual(driver.host_get_mask(), 1)

    def test_touch_decodes_two_byte_little_endian_coordinates(self):
        receive(touch(0x123, 0x201))
        self.assertEqual(take_event(), (2, 0x123, 0x201))
        self.assertEqual(take_event(), (0, 0, 0))

    def test_every_partial_touch_frame_waits(self):
        frame = touch(489, 439)
        for byte in frame[:-1]:
            receive(bytes([byte]))
            self.assertEqual(take_event(), (0, 0, 0))
        receive(frame[-1:])
        self.assertEqual(take_event(), (2, 489, 439))

    def test_mixed_ok_and_touch_events_keep_order(self):
        receive(FRAME + touch(20, 439) + touch(489, 20) + FRAME)
        self.assertEqual([take_event() for _ in range(5)],
                         [(1, 0, 0), (2, 20, 439), (2, 489, 20),
                          (1, 0, 0), (0, 0, 0)])

    def test_ff_coordinates_are_payload_not_terminators(self):
        receive(touch(0xffff, 0x12ff) + FRAME)
        self.assertEqual(take_event(), (2, 0xffff, 0x12ff))
        self.assertEqual(take_event(), (1, 0, 0))

    def test_bad_touch_tail_resynchronizes(self):
        receive(touch(100, 200)[:-1] + b'\x00' + touch(300, 400))
        self.assertEqual(take_event(), (2, 300, 400))

    def test_truncated_touch_before_new_frame_resynchronizes(self):
        receive(touch(100, 200)[:5] + touch(300, 400))
        self.assertEqual(take_event(), (2, 300, 400))

    def test_touch_frame_crosses_ring_end(self):
        receive(b'\x00' * 496)
        self.assertEqual(take_event(), (0, 0, 0))
        receive(touch(489, 439))
        self.assertEqual(take_event(), (2, 489, 439))

    def test_touch_error_restarts_reception(self):
        receive(touch(489, 439)[:7])
        driver.host_error(2)
        receive(touch(20, 20))
        self.assertEqual(take_event(), (2, 20, 20))

    def test_touch_dequeue_preserves_interrupt_mask(self):
        receive(touch(20, 20))
        driver.host_set_mask(1)
        self.assertEqual(take_event(), (2, 20, 20))
        self.assertEqual(driver.host_get_mask(), 1)

    def test_all_corners_map_to_calibrated_limits_with_y_flipped(self):
        for px, py, x, y in [(20, 439, 0, 0), (20, 20, 0, 144),
                              (489, 439, 161, 0), (489, 20, 161, 144)]:
            with self.subTest(pixel=(px, py)):
                ok, actual_x, actual_y = map_touch(px, py)
                self.assertEqual(ok, 1)
                self.assertAlmostEqual(actual_x, x, places=4)
                self.assertAlmostEqual(actual_y, y, places=4)

    def test_one_pixel_outside_each_edge_is_rejected(self):
        for px, py in [(19, 230), (490, 230), (255, 19), (255, 440)]:
            with self.subTest(pixel=(px, py)):
                self.assertEqual(map_touch(px, py), (0, -1, -1))

    def test_invalid_ranges_are_rejected(self):
        for xm, ym in [(0, 144), (161, 0), (-1, 144), (161, -1),
                       (float('nan'), 144)]:
            with self.subTest(area=(xm, ym)):
                self.assertEqual(map_touch(255, 230, xm, ym), (0, -1, -1))

    def test_uncalibrated_touch_reports_raw_pixels_without_target(self):
        receive(touch(255, 230))
        driver.host_poll()
        self.assertEqual(output(), 'HMI TOUCH PX=255 PY=230 -> TARGET UNAVAILABLE: '
                         'run KEY1 calibration\r\n')

    def test_outside_touch_reports_rejection(self):
        driver.host_set_area(161, 144, 1)
        receive(touch(490, 230))
        driver.host_poll()
        self.assertIn('PX=490 PY=230 -> REJECT: outside touch region', output())
        self.assertNotIn('TARGET X=', output())

    def test_interior_target_report_has_two_decimal_mm(self):
        driver.host_set_area(161, 144, 1)
        receive(touch(255, 230))
        driver.host_poll()
        self.assertEqual(output(), 'HMI TOUCH PX=255 PY=230 -> TARGET X=80.67mm '
                         'Y=71.83mm\r\n')
        self.assertEqual(driver.host_targets(), 1)
        self.assertAlmostEqual(driver.host_target_x(), 80.6716, places=3)
        self.assertAlmostEqual(driver.host_target_y(), 71.8282, places=3)

    def test_range_changes_and_revoked_calibration_are_not_cached(self):
        driver.host_set_area(161, 144, 1)
        receive(touch(489, 20))
        driver.host_poll()
        self.assertIn('X=161.00mm Y=144.00mm', output())
        driver.host_clear_output()
        driver.host_set_area(161, 144, 0)
        receive(touch(489, 20))
        driver.host_poll()
        self.assertNotIn('TARGET X=', output())
        driver.host_clear_output()
        driver.host_set_area(160.75, 143.25, 1)
        receive(touch(489, 20))
        driver.host_poll()
        self.assertIn('X=160.75mm Y=143.25mm', output())

    def test_calibrated_flag_with_invalid_range_reports_rejection(self):
        driver.host_set_area(0, 144, 1)
        receive(touch(20, 20))
        driver.host_poll()
        self.assertIn('REJECT: invalid calibrated range', output())

    def test_ok_and_touch_both_report_in_same_poll(self):
        driver.host_set_area(161, 144, 1)
        receive(FRAME + touch(20, 439) + FRAME)
        driver.host_poll()
        self.assertEqual(output(), 'ok\r\nHMI TOUCH PX=20 PY=439 -> TARGET '
                         'X=0.00mm Y=0.00mm\r\nok\r\n')

    def test_queued_touch_frames_are_each_reported_once(self):
        driver.host_set_area(161, 144, 1)
        receive(touch(20, 439) * 50)
        driver.host_poll()
        self.assertEqual(output().count('HMI TOUCH'), 50)
        self.assertEqual(driver.host_length(), 0)
        driver.host_poll()
        self.assertEqual(output().count('HMI TOUCH'), 50)

    def test_startup_reports_region_and_protocol(self):
        driver.host_hmi_init()
        self.assertIn('HMI POSITION: direct line, 50.00mm/s; latest pending click wins', output())
        self.assertIn('HMI REGION X=20 Y=20 W=470 H=420', output())


class ActualMarkerTests(unittest.TestCase):
    def setUp(self):
        driver.host_init()
        driver.host_set_area(161, 144, 1)
        driver.host_set_actual(0, 0, 0, 0, 1, 0)

    def test_request_fragmentation_and_event_order(self):
        for byte in POSITION_REQUEST[:-1]:
            receive(bytes([byte]))
            self.assertEqual(take_event(), (0, 0, 0))
        receive(POSITION_REQUEST[-1:] + touch(255, 230) + FRAME)
        self.assertEqual([take_event() for _ in range(3)],
                         [(3, 0, 0), (2, 255, 230), (1, 0, 0)])

    def test_bad_request_cannot_trigger_position_or_motion(self):
        receive(bytes.fromhex('55 50 00 FF FF FF'))
        driver.host_poll(); driver.host_publish()
        self.assertEqual(driver.host_tx_starts(), 0)
        self.assertEqual(driver.host_targets(), 0)

    def test_old_screen_gets_no_unsolicited_commands(self):
        driver.host_publish()
        receive(FRAME + touch(255, 230)); driver.host_poll(); driver.host_publish()
        self.assertEqual(driver.host_tx_starts(), 0)

    def test_actual_mapping_corners_and_center(self):
        for x, y, px, py in [(0, 0, 20, 439), (161, 0, 489, 439),
                              (0, 144, 20, 20), (161, 144, 489, 20),
                              (80.5, 72, 255, 229)]:
            self.assertEqual(map_actual(x, y), (1, px, py))

    def test_mapping_rejects_nonfinite_and_bad_range(self):
        for x, y, xm, ym in [(float('nan'), 0, 161, 144), (0, float('inf'), 161, 144),
                              (0, 0, 0, 144), (0, 0, float('inf'), 144),
                              (0, 0, 161, float('nan'))]:
            self.assertEqual(map_actual(x, y, xm, ym), (0, 999, 999))

    def test_half_mm_boundary_error_is_only_display_clamped(self):
        self.assertEqual(map_actual(-0.5, 144.5), (1, 20, 20))
        self.assertEqual(map_actual(-0.501, 0), (0, 999, 999))
        self.assertEqual(map_actual(161.501, 0), (0, 999, 999))

    def test_reply_is_actual_feedback_not_clicked_target(self):
        driver.host_set_actual(40.25, 36, 0, 0, 1, 0)
        receive(touch(489, 20))
        request_position()
        self.assertEqual(tx_bytes(), position_reply(137, 334, 1))
        self.assertEqual(driver.host_target_x(), 161)
        self.assertEqual(driver.host_target_y(), 144)

    def test_freshness_at_300ms_and_each_stale_motor(self):
        for age_a, age_b, valid in [(300, 300, 1), (301, 0, 0), (0, 301, 0)]:
            driver.host_init(); driver.host_set_area(161, 144, 1)
            driver.host_set_actual(0, 0, age_a, age_b, 1, 0)
            request_position()
            self.assertEqual(tx_bytes(), position_reply(20, 439, 1) if valid
                             else position_reply(0, 0, 0))

    def test_uncalibrated_position_has_no_green_marker(self):
        driver.host_set_area(161, 144, 0); request_position()
        self.assertEqual(tx_bytes(), position_reply(0, 0, 0))

    def test_no_position_packets_has_no_green_marker(self):
        driver.host_set_actual(0, 0, 0, 0, 0, 0); request_position()
        self.assertEqual(tx_bytes(), position_reply(0, 0, 0))

    def test_fault_hides_green_and_request_cannot_start_motors(self):
        driver.host_set_actual(0, 0, 0, 0, 1, 1); request_position()
        self.assertEqual(tx_bytes(), position_reply(0, 0, 0))
        self.assertEqual(driver.host_targets(), 0)

    def test_rate_limit_coalesces_and_returns_latest_position(self):
        request_position(); driver.host_tx_complete(2)
        driver.host_set_tick(99); request_position()
        self.assertEqual(driver.host_tx_starts(), 1)
        driver.host_set_actual(161, 144, 0, 0, 1, 0)
        driver.host_set_tick(100); driver.host_publish()
        self.assertEqual(driver.host_tx_starts(), 2)
        self.assertEqual(tx_bytes(), position_reply(489, 20, 1))

    def test_busy_keeps_inflight_bytes_and_retries_latest_sample(self):
        request_position(); first = tx_bytes()
        driver.host_set_tick(100); request_position()
        self.assertEqual(tx_bytes(), first)
        self.assertEqual(driver.host_tx_attempts(), 1)
        driver.host_set_actual(80.5, 72, 0, 0, 1, 0)
        driver.host_tx_complete(2); driver.host_publish()
        self.assertEqual(tx_bytes(), position_reply(255, 229, 1))

    def test_hal_busy_or_error_is_retried_without_waiting(self):
        for result in (1, 2):
            driver.host_init(); driver.host_set_area(161, 144, 1)
            driver.host_set_actual(0, 0, 0, 0, 1, 0)
            driver.host_tx_result(result); request_position()
            self.assertEqual(driver.host_tx_starts(), 0)
            driver.host_tx_result(0); driver.host_publish()
            self.assertEqual(tx_bytes(), position_reply(20, 439, 1))

    def test_rate_limit_survives_tick_wrap(self):
        driver.host_set_tick(0xfffffff0); driver.host_hmi_init()
        request_position(); driver.host_tx_complete(2)
        driver.host_set_tick(0x53); request_position()
        self.assertEqual(driver.host_tx_starts(), 1)
        driver.host_set_tick(0x54); driver.host_publish()
        self.assertEqual(driver.host_tx_starts(), 2)

    def test_discard_removes_old_request_and_invalidates_display(self):
        request_position(); driver.host_tx_complete(2)
        driver.host_set_tick(100)
        receive(POSITION_REQUEST); driver.host_poll()
        driver.host_discard()
        self.assertEqual(tx_bytes(), position_reply(0, 0, 0))
        driver.host_tx_complete(2); driver.host_publish()
        self.assertEqual(driver.host_tx_starts(), 2)

    def test_uart_owns_copy_and_ignores_other_uart_completion(self):
        text = ctypes.create_string_buffer(b'page 0')
        self.assertEqual(driver.host_send_string(text), 1)
        text.value = b'xxxxxx'
        self.assertEqual(tx_bytes(), b'page 0\xff\xff\xff')
        driver.host_tx_complete(1)
        self.assertEqual(driver.host_send_string(b'page 1'), 0)
        driver.host_tx_complete(2)
        self.assertEqual(driver.host_send_string(b'page 1'), 1)

    def test_rx_error_cannot_release_tx_buffer(self):
        driver.host_send_string(b'page 0'); first = tx_bytes()
        driver.host_error(2)
        self.assertEqual(driver.host_send_string(b'page 1'), 0)
        self.assertEqual(tx_bytes(), first)
        driver.host_tx_abort()
        self.assertEqual(driver.host_send_string(b'page 1'), 1)

    def test_oversized_command_never_sends_partial_packet(self):
        self.assertEqual(driver.host_send_string(b'x' * 190), 0)
        self.assertEqual(driver.host_tx_starts(), 0)
        self.assertEqual(driver.host_send_string(b'page 0'), 1)
        self.assertEqual(tx_bytes(), b'page 0\xff\xff\xff')

    def test_text_and_negative_int_frames_have_exact_terminators(self):
        self.assertEqual(driver.host_send_value(-2147483648), 1)
        self.assertEqual(tx_bytes(), b'n0.val=-2147483648\xff\xff\xff')
        driver.host_tx_complete(2)
        self.assertEqual(driver.host_send_text(b'Ready'), 1)
        self.assertEqual(tx_bytes(), b't0.txt="Ready"\xff\xff\xff')


if __name__ == '__main__':
    unittest.main(verbosity=2)
