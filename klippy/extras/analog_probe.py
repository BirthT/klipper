import logging
import pins
from . import manual_probe

HINT_TIMEOUT = """
"""
class AnalogProbe:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.config = config
        self.mcu_probe = mcu_probe
        self.last_state = False

        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        # Register z_virtual_endstop pin
        #TODO register_chipをs変更する
        self.printer.lookup_object('pins').register_chip('probe', self)
        # Register homing event handlers
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PROBE', self.cmd_PROBE,
                                    desc=self.cmd_PROBE_help)
        self.gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
                                    desc=self.cmd_QUERY_PROBE_help)
        self.gcode.register_command('PROBE_CALIBRATE', self.cmd_PROBE_CALIBRATE,
                                    desc=self.cmd_PROBE_CALIBRATE_help)
        self.gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
                                    desc=self.cmd_PROBE_ACCURACY_help)
        self.gcode.register_command('Z_OFFSET_APPLY_PROBE',
                                    self.cmd_Z_OFFSET_APPLY_PROBE,
                                    desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)
    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)
    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe
    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        return self.lift_speed
    def _probe(self, speed):
        pass
        return 0
    def _move(self, coord, speed):
        pass
    def _calc_mean(self, positions):
        pass
    def _calc_median(self, positions):
        pass
    def run_probe(self, gcmd):
        pass
    cmd_PROBE_help = "Probe Z-height at current XY position"
    def cmd_PROBE(self, gcmd):
        pass
    cmd_QUERY_PROBE_help = "Return the status of the z-probe"
    def cmd_QUERY_PROBE(self, gcmd):
        pass
    def get_status(self, eventtime):
        return {'name': self.name,
                'last_query': self.last_state,
                'last_z_result': self.last_z_result}
    cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"
    def cmd_PROBE_ACCURACY(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
                          " (samples=%d retract=%.3f"
                          " speed=%.1f lift_speed=%.1f)\n"
                          % (pos[0], pos[1], pos[2],
                             sample_count, sample_retract_dist,
                             speed, lift_speed))
        # Probe bed sample_count times
        self.multi_probe_begin()
        positions = []
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed)
            positions.append(pos)
            # Retract
            liftpos = [None, None, pos[2] + sample_retract_dist]
            self._move(liftpos, lift_speed)
        self.multi_probe_end()
        # Calculate maximum, minimum and average values
        max_value = max([p[2] for p in positions])
        min_value = min([p[2] for p in positions])
        range_value = max_value - min_value
        avg_value = self._calc_mean(positions)[2]
        median = self._calc_median(positions)[2]
        # calculate the standard deviation
        deviation_sum = 0
        for i in range(len(positions)):
            deviation_sum += pow(positions[i][2] - avg_value, 2.)
        sigma = (deviation_sum / len(positions)) ** 0.5
        # Show information
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f" % (
            max_value, min_value, range_value, avg_value, median, sigma))
    def probe_calibrate_finalize(self, kin_pos):
        if kin_pos is None:
            return
        z_offset = self.probe_calibrate_z - kin_pos[2]
        self.gcode.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, z_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'z_offset', "%.3f" % (z_offset,))
    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"
    def cmd_PROBE_CALIBRATE(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Perform initial probe
        lift_speed = self.get_lift_speed(gcmd)
        curpos = self.run_probe(gcmd)
        # Move away from the bed
        self.probe_calibrate_z = curpos[2]
        curpos[2] += 5.
        self._move(curpos, lift_speed)
        # Move the nozzle over the probe point
        curpos[0] += self.x_offset
        curpos[1] += self.y_offset
        self._move(curpos, self.speed)
        # Start manual probe
        manual_probe.ManualProbeHelper(self.printer, gcmd,
                                       self.probe_calibrate_finalize)
    def cmd_Z_OFFSET_APPLY_PROBE(self,gcmd):
        offset = self.gcode_move.get_status()['homing_origin'].z
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_calibrate = self.z_offset - offset
            self.gcode.respond_info(
                "%s: z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer."
                % (self.name, new_calibrate))
            configfile.set(self.name, 'z_offset', "%.3f" % (new_calibrate,))
    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"

# Endstop wrapper
class AnalogProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = 0.0 #config.getfloat('z_offset')

        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        # endstop用のADCを定義する
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=False, can_pullup=False)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        # = ppins.setup_pin('endstop', pin)

        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = 'OFF'
    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    #def multi_probe_begin(self):
    #    if self.stow_on_each_sample:
    #        return
    #    self.multi = 'FIRST'
    #def multi_probe_end(self):
    #    if self.stow_on_each_sample:
    #        return
    #    self.raise_probe()
    #    self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.raise_probe()
    def get_position_endstop(self):
        return self.position_endstop

def load_config(config):
    ap=AnalogProbeEndstopWrapper(config)
    #TODO 別名でs保存する
    config.get_printer().add_object('probe', AnalogProbe(config, ap))
    return ap