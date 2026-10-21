from pathlib import Path

root = Path.home() / 'vmdiag-app'
src = root / 'scripts/pulse1_first_bounded_motion_v3.py'
dst = root / 'scripts/pulse1_first_bounded_motion_v4.py'

if dst.exists():
    raise SystemExit(f'STOP: V4 già esistente: {dst}')
if not src.is_file():
    raise SystemExit(f'STOP: V3 non trovato: {src}')
s = src.read_text()

old_import = 'from ros_gz_interfaces.msg import WorldStatistics\n'
assert s.count(old_import) == 1, 'Import WorldStatistics inatteso'
s = s.replace(old_import, old_import + 'from ros_gz_interfaces.srv import ControlWorld\n', 1)
old_trace = "TRACE_PATH = Path('/tmp/pulse1_first_bounded_motion_v3.jsonl')"
assert s.count(old_trace) == 1, 'TRACE_PATH V3 inatteso'
s = s.replace(old_trace, "TRACE_PATH = Path('/tmp/pulse1_first_bounded_motion_v4.jsonl')", 1)

old_node = '        self.node = rclpy.create_node("pulse1_first_bounded_motion_probe")\n'
assert s.count(old_node) == 1, 'Creazione nodo inattesa'
s = s.replace(old_node,
              '        self.node = rclpy.create_node("pulse1_first_bounded_motion_v4_probe")\n'
              '        self.control_client = self.node.create_client(ControlWorld, WORLD_CONTROL)\n', 1)

start = s.index('    def step(self, n, first_x=None):')
end = s.index('\n\ndef run(probe):', start)
new_step = '''    def step(self, n, first_x=None):
        """ControlWorld ROS persistente; ritorna solo dopo ACK e target PAUSED.

        In stato armato non supera 50 step; in errore non ritenta
        automaticamente una richiesta che potrebbe essere ancora in corso.
        """
        self.require_paused()
        if not isinstance(n, int) or n < 1:
            raise RuntimeError(f"Conteggio step non valido: {n}")
        if first_x is not None and n > 50:
            raise RuntimeError("Oltre 50 step nella prova armata")
        if first_x is None and self.publishers("/pulse1/avoid_request"):
            raise RuntimeError("Trial publisher presente durante step disarmato")
        if not self.control_client.service_is_ready():
            if not self.control_client.wait_for_service(timeout_sec=8.0):
                raise RuntimeError("ControlWorld ROS non disponibile")

        start_i, _, start_t = self.stats
        target = start_i + n
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.multi_step = n
        began = time.monotonic()
        # Massimo 4 s per blocco armato; il warmup può richiedere più tempo.
        timeout_s = 4.0 if first_x is not None else 35.0
        future = self.control_client.call_async(req)
        ack_ok = False
        physics_ok = False
        try:
            while time.monotonic() - began < timeout_s:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if first_x is not None:
                    self.safety(first_x)
                if self.stats is not None:
                    iters, paused, _ = self.stats
                    if iters > target:
                        raise RuntimeError("Gazebo ha superato gli step richiesti")
                    if iters == target and paused is True:
                        physics_ok = True
                    elif physics_ok and paused is not True:
                        raise RuntimeError("Gazebo ripartito dopo PAUSED al target")
                if future.done() and not ack_ok:
                    response = future.result()
                    if response is None or response.success is not True:
                        raise RuntimeError("ControlWorld ROS senza ACK positivo")
                    ack_ok = True
                if ack_ok and physics_ok:
                    self.log('CONTROL_STEP', {
                        'n': n, 'start': start_i, 'target': target,
                        'wall_s': round(time.monotonic() - began, 3),
                        'armed_phase': first_x is not None,
                    })
                    return (self.stats[2] - start_t) / n
            raise RuntimeError(
                f"Timeout ControlWorld: target={target}, "
                f"ACK={ack_ok}, PAUSED_TARGET={physics_ok}, stats={self.stats}"
            )
        finally:
            # Non richiamare lo step: anche con timeout la richiesta lato
            # server potrebbe essere in corso. Il main esegue il controllo PAUSE.
            if not future.done():
                self.log('CONTROL_PENDING', {'target': target, 'armed': first_x is not None})
'''
s = s[:start] + new_step + s[end:]
# Logging separato per V4, senza sovrascrivere il log della V3.
s = s.replace('"/tmp/pulse1_first_bounded_motion.log"',
              '"/tmp/pulse1_first_bounded_motion_v4.log"')
compile(s, str(dst), 'exec')
dst.write_text(s)
print('V4 CREATA:', dst)
print('V3 INVARIATA:', src)
print('CONTROL_STEP persistente, cap armato 50 step, timeout armato 4 s')
print('Traccia: /tmp/pulse1_first_bounded_motion_v4.jsonl')
