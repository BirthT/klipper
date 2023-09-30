import logging
import chelper
from mcu import MCU, MCU_trsync
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
        #TODO register_chipを変更する
        self.printer.lookup_object('pins').register_chip('analog_probe', self)

        # Register homing event handlers
        #self.printer.register_event_handler("homing:homing_move_begin",
        #                                    self._handle_homing_move_begin)
        #self.printer.register_event_handler("homing:homing_move_end",
        #                                    self._handle_homing_move_end)
        #self.printer.register_event_handler("homing:home_rails_begin",
        #                                    self._handle_home_rails_begin)
        #self.printer.register_event_handler("homing:home_rails_end",
        #                                    self._handle_home_rails_end)
        #self.printer.register_event_handler("gcode:command_error",
        #                                    self._handle_command_error)
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')
        #self.gcode.register_command('PROBE', self.cmd_PROBE,
        #                            desc=self.cmd_PROBE_help)
        #self.gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
        #                            desc=self.cmd_QUERY_PROBE_help)
        #self.gcode.register_command('PROBE_CALIBRATE', self.cmd_PROBE_CALIBRATE,
        #                            desc=self.cmd_PROBE_CALIBRATE_help)
        #self.gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
        #                            desc=self.cmd_PROBE_ACCURACY_help)
        #self.gcode.register_command('Z_OFFSET_APPLY_PROBE',
        #                            self.cmd_Z_OFFSET_APPLY_PROBE,
        #                            desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)
        self.gcode.register_command('QUERY_ANALOG_PROBE', self.cmd_QUERY_ANALOG_PROBE,
                                    desc=self.cmd_QUERY_ANALOG_PROBE_help)
    #def _handle_homing_move_begin(self, hmove):
    #    if self.mcu_probe in hmove.get_mcu_endstops():
    #        self.mcu_probe.probe_prepare(hmove)
    #def _handle_homing_move_end(self, hmove):
    #    if self.mcu_probe in hmove.get_mcu_endstops():
    #        self.mcu_probe.probe_finish(hmove)
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe
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
    def get_status(self, eventtime):
        return {'name': self.name,
                'last_query': self.last_state}
    
    cmd_QUERY_ANALOG_PROBE_help = "Get Analog Probe value"
    def cmd_QUERY_ANALOG_PROBE(self,gcmd):
        (get_value, get_time) = self.mcu_probe.get_value()
        gcmd.respond_info("analog probe: t=%f,  v=%f" % (get_time, get_value))

# Endstop wrapper
TRSYNC_TIMEOUT = 0.025
TRSYNC_SINGLE_MCU_TIMEOUT = 0.250
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

        self.min_adc = config.getfloat('min_adc', -9999999.9,
                                        minval=-9999999.9)
        self.max_adc = config.getfloat('max_adc', 99999999.9,
                                        above=self.min_adc)
        self.endstop_trigger = config.getfloat('endstop_trigger', 0.5)


        # endstop用のADCを定義する
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=False, can_pullup=False)
        self.mcu = pin_params['chip']
        self.mcu_endstop = self.mcu.setup_pin('adc', pin_params)

        self.oid = self.mcu.create_oid()

        ADC_SAMPLE_TIME = 0.001
        ADC_SAMPLE_COUNT = 8
        ADC_REPORT_TIME = 0.01
        self.mcu_endstop.setup_minmax(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT)
        self.mcu_endstop.setup_adc_callback(ADC_REPORT_TIME, self.adc_callback)

        self.real_trigger_val = int(self.endstop_trigger / self.mcu_endstop._inv_max_adc)

        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(self.mcu, self._trdispatch)]

        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

        cmd_queue = self._trsyncs[0].get_command_queue()
        self._home_cmd = self.mcu.lookup_command(
            "analog_endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
            " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c tirgger_val=%u",
            cq=cmd_queue)


        # Wrappers
        #self.get_mcu = self.mcu_endstop.get_mcu
        #self.add_stepper = self.mcu_endstop.add_stepper
        #self.get_steppers = self.mcu_endstop.get_steppers
        #self.home_start = self.mcu_endstop.home_start
        #self.home_wait = self.mcu_endstop.home_wait
        #self.query_endstop = self.mcu_endstop.query_endstop
    
    def get_value(self):
        return self.mcu_endstop.get_last_value()
    
    def adc_read(self):
        (_v, _t) = self.mcu_endstop.get_last_value()
        return _v

    def adc_callback(self, read_time, read_value):
        # read sensor value
        pass
    
    def get_mcu(self):
        return self.mcu
    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self.mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")
    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]    
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        clock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.print_time_to_clock(print_time+rest_time) - clock
        self._rest_ticks = rest_ticks
        reactor = self.mcu.get_printer().get_reactor()
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        self._home_cmd.send(
            [self.oid, clock, self.mcu.seconds_to_clock(sample_time),
             sample_count, rest_ticks, triggered, 
             etrsync.get_oid(), etrsync.REASON_ENDSTOP_HIT, self.real_trigger_val], reqclock=clock)
        return self._trigger_completion
    def home_wait(self, home_end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self.mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        self._home_cmd.send([self.oid, 0, 0, 0, 0, 0, 0, 0, 0])
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self.mcu.is_fileoutput():
            return home_end_time
        
        return home_end_time
        #ffi_main, ffi_lib = chelper.get_ffi()
        #ffi_lib.trdispatch_stop(self._trdispatch)
        #res = [trsync.stop() for trsync in self._trsyncs]
        #if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
        #    return -1.
        #if res[0] != etrsync.REASON_ENDSTOP_HIT:
        #    return 0.
        #if self._mcu.is_fileoutput():
        #    return home_end_time
        #params = self._query_cmd.send([self.oid])
        #next_clock = self.mcu.clock32_to_clock64(params['next_clock'])
        #return self.mcu.clock_to_print_time(next_clock - self._rest_ticks)

    def query_endstop(self, print_time):
        _res = self.adc_read()
        params = 0 #default open
        if _res < self.endstop_trigger: #trigered
            params = 1
        return params

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('y'):
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
    #def get_position_endstop(self):
    #    return self.position_endstop

def load_config(config):
    ap=AnalogProbeEndstopWrapper(config)
    #TODO 別名で保存する
    config.get_printer().add_object('analog_probe', AnalogProbe(config, ap))
    return ap