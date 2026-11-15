import unittest

from my_algo.preflight import check, full_name, parse_rule

RULES = [parse_rule("/livox/lidar|/livox_lidar_publisher|1|1"),
         parse_rule("/commands/motor/speed|/exp_drive_mux,/joy_controller|1|2")]


class PreflightTest(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(check(RULES, {'/livox/lidar': ['/livox_lidar_publisher'],
                                       '/commands/motor/speed': ['/exp_drive_mux', '/joy_controller']}), [])

    def test_car2_namespace_publisher_rejected(self):
        p = check(RULES, {'/livox/lidar': ['/livox_lidar_publisher'],
                          '/commands/motor/speed': ['/exp_drive_mux', '/car2/vesc_driver_node']})
        self.assertTrue(any('/car2/vesc_driver_node' in x for x in p))

    def test_same_name_on_other_host_counted(self):
        # #2에서 같은 이름의 Livox 드라이버가 떠 있으면 발행자 2개 → 위반
        p = check(RULES, {'/livox/lidar': ['/livox_lidar_publisher', '/livox_lidar_publisher'],
                          '/commands/motor/speed': ['/exp_drive_mux']})
        self.assertTrue(any('발행자 2개' in x for x in p))

    def test_missing_publisher(self):
        p = check(RULES, {'/commands/motor/speed': ['/exp_drive_mux']})
        self.assertTrue(any('/livox/lidar' in x for x in p))

    def test_full_name_and_bad_rule(self):
        self.assertEqual(full_name('/', 'a'), '/a')
        self.assertEqual(full_name('/car2', 'b'), '/car2/b')
        with self.assertRaises(ValueError):
            parse_rule('/x|a')


if __name__ == '__main__':
    unittest.main()
