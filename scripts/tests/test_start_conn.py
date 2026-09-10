"""不导入启动脚本、不连接机器人，验证PTY等待的失败和超时行为。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class WaitForPortsTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / 'start_conn.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'wait_for_ports']
        self.assertEqual(len(functions), 1, '缺少可独立验证的PTY就绪等待函数')
        self.now = 0.0
        self.sleeps = 0
        def sleep(dt):
            self.now += dt
            self.sleeps += 1
        self.env = {'time': SimpleNamespace(monotonic=lambda: self.now, sleep=sleep)}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), self.env)

    def test_waits_until_all_ports_exist(self):
        port = SimpleNamespace(exists=lambda: self.sleeps >= 2)
        self.env['wait_for_ports']([port], [SimpleNamespace(poll=lambda: None)], timeout=1)
        self.assertEqual(self.sleeps, 2)

    def test_process_failure_does_not_wait_forever(self):
        with self.assertRaises(RuntimeError):
            self.env['wait_for_ports']([SimpleNamespace(exists=lambda: False)], [SimpleNamespace(poll=lambda: 1)])

    def test_missing_port_times_out(self):
        with self.assertRaises(TimeoutError):
            self.env['wait_for_ports']([SimpleNamespace(exists=lambda: False)], [SimpleNamespace(poll=lambda: None)], timeout=0.1)


if __name__ == '__main__':
    unittest.main()
