"""
Extension of SHESHA-Simulator to accommodate for Matlab controlled, user-defined control scripts
"""
from shesha.sim.simulator import Simulator


# new packages
import shesha.util.mat_log as mat_log
import shesha.util.z_aberration as z_aber
import shesha.util.t_control as t_ctrl
import shesha.util.ud_control as ud_ctrl
import shesha.config as conf


class SimulatorUDC(Simulator):
    """
    The SimulatorUDC class is self sufficient for running a COMPASS simulation
    Initializes and run a COMPASS simulation
    """

    def __init__(self, filepath: str = None, use_DB: bool = False) -> None:
        """
        Initializes a Simulator instance

        :parameters:
            filepath: (str): (optional) path to the parameters file
            use_DB: (bool): (optional) flag to use dataBase system
        """
        Simulator.__init__(self, filepath, use_DB)

    def init_sim(self) -> None:
        """
        Initializes the simulation by creating all the sutra objects that will be used
        """

        Simulator.__init_sim(self)
        self._ud_control_init()

    def _mat_logging_init(self):
        """
        Initializes the logging of variables in a mat-file
        """
        mat_log.mat_file_init(log_mat=self.config.p_matlog.log_mat,
                              mat_file_name=self.config.p_matlog.mat_file_name,
                              mat_file_dir=self.config.p_matlog.mat_file_dir,
                              mat_var=self.config.p_matlog.mat_var,
                              bool_log_init=self.config.p_matlog.log_init)

        # save variables (after initialization) in mat-file
        if self.config.p_matlog.log_mat and self.config.p_matlog.log_init:
            mat_log.mat_file_write(log_mat=self.config.p_matlog.log_mat,
                                   mat_file_name=self.config.p_matlog.mat_file_name,
                                   mat_file_dir=self.config.p_matlog.mat_file_dir,
                                   mat_num_dig=self.config.p_matlog.mat_num_dig,
                                   mat_var=self.config.p_matlog.mat_var,
                                   mat_cmds=self.config.p_matlog.mat_cmds,
                                   index=-1, sim=self)

    def _z_aberration_init(self):
        """
        Initializes the custom zernike aberrations (in pupil)
        """
        z_aber.init_z_aber(self)

    def _t_control_init(self):
        """
        Initializes the Time Control utility (enabling oversampling)
        """
        t_ctrl.t_ctrl_init(self)

    def _ud_control_init(self):
        """
        Initializes the User Defined Control utility
        """
        ud_ctrl.ud_ctrl_init(self)

    def next(self, *, move_atmos: bool = True, see_atmos: bool = True, nControl: int = 0,
             tar_trace: Iterable[int] = None, wfs_trace: Iterable[int] = None,
             do_control: bool = True, apply_control: bool = True) -> None:
        '''
        Iterates the AO loop, with optional parameters

        :parameters:
             move_atmos: (bool): move the atmosphere for this iteration, default: True

             nControl: (int): Controller number to use, default 0 (single control configurations)

             tar_trace: (None or list[int]): list of targets to trace. None equivalent to all.

             wfs_trace: (None or list[int]): list of WFS to trace. None equivalent to all.

             apply_control: (bool): (optional) if True (default), apply control on DMs
        '''
        if tar_trace is None and self.tar is not None:
            tar_trace = range(self.config.p_target.ntargets)
        if wfs_trace is None and self.wfs is not None:
            wfs_trace = range(len(self.config.p_wfss))

        if move_atmos and self.atm is not None:
            self.atm.move_atmos()

        if (
                self.config.p_controllers is not None and
                self.config.p_controllers[nControl].type == scons.ControllerType.GEO):
            for t in tar_trace:
                if see_atmos:
                    self.tar.raytrace(t, b"atmos", atmos=self.atm)
                else:
                    self.tar.reset_phase(t)
                self.tar.raytrace(t, b"telncpa", tel=self.tel, ncpa=1)
                self.tar.raytrace(t, b"dm", dms=self.dms)
                self.rtc.do_control_geo(nControl, self.dms, self.tar, t)
                self.rtc.apply_control(nControl, self.dms)
        else:
            for t in tar_trace:
                if see_atmos:
                    self.tar.raytrace(t, b"atmos", atmos=self.atm)
                else:
                    self.tar.reset_phase(t)
                self.tar.raytrace(t, b"dm", tel=self.tel, dms=self.dms, ncpa=1)
            for w in wfs_trace:

                if see_atmos:
                    self.wfs.raytrace(w, b"atmos", tel=self.tel, atmos=self.atm, ncpa=1)
                else:
                    self.wfs.raytrace(w, b"telncpa", tel=self.tel, rst=1, ncpa=1)

                if not self.config.p_wfss[w].openloop and self.dms is not None:
                    self.wfs.raytrace(w, b"dm", dms=self.dms)
                self.wfs.comp_img(w)
        if do_control and self.config.p_t_ctrl.os_dec <= 1 and not self.config.p_ud_ctrl.use_udc:
            self.rtc.do_centroids(nControl)
            self.rtc.do_control(nControl)
            self.rtc.do_clipping(0, -1e5, 1e5)
            if apply_control:
                self.rtc.apply_control(nControl, self.dms)

        self.iter += 1

    def loop(self, n: int = 1, monitoring_freq: int = 100, save_index: int = 0,
             phase_index: int = 1, **kwargs):
        """
        Perform the AO loop for n iterations

        :parameters:
            n: (int): (optional) Number of iteration that will be done
            monitoring_freq: (int): (optional) Monitoring frequency [frames]
            save_index: (int): (optional) start index for mat-file-logging
            phase_index: (int): (optional) start index of phasescreen aberrations
        """
        self.config.p_z_ab.set_phase_index(phase_index)
        self.config.p_matlog.set_save_index(save_index)

        print("----------------------------------------------------")
        print("iter# | S.E. SR | L.E. SR | ETR (s) | Framerate (Hz)")
        print("----------------------------------------------------")
        t0 = time.time()
        t1 = time.time()
        if n == -1:
            i = 0
            while (True):
                self.next(**kwargs)
                if ((i + 1) % monitoring_freq == 0):
                    self.print_strehl(monitoring_freq, time.time() - t1, i, i)
                    t1 = time.time()
                i += 1

        for i in range(n):
            # Iterate optical system
            self.next(**kwargs)

            # additional commands executed after every iteration
            self.loop_addition(i + 1)

            # show simulation progress
            if ((i + 1) % monitoring_freq == 0):
                self.print_strehl(monitoring_freq, time.time() - t1, i, n)
                t1 = time.time()

        t1 = time.time()
        print(" loop execution time:", t1 - t0, "  (", n, "iterations), ", (t1 - t0) / n,
              "(mean)  ", n / (t1 - t0), "Hz")

        self.end_addition()

    def loop_addition(self, it_index):
        """
        Additional code executed after every simulation step
        """

        # update of Time control
        if self.config.p_t_ctrl.os_dec > 1:
            # cumulate wfs images
            t_ctrl.add_sub_images(self)

            # upload wfs images to GPU
            t_ctrl.upload_images(self)

            # set modulation points for next sub-cycle
            t_ctrl.update_pyr_mod(self)

        # update of User Defined Control
        if self.config.p_ud_ctrl.use_udc:
            # evaluate mirror commands
            ud_ctrl.eval_ud_ctrl(self)

            # apply mirror commands
            ud_ctrl.update_mirror_shape(self)

        # update loop_it and main_cnt after for next sub-cycle
        t_ctrl.update_idx(self)

        # update of Zernike aberration
        if self.config.p_z_ab.include_zab and (it_index % self.config.p_z_ab.dec == 0):

            # calculate and set phase-screen for spupil
            if (self.config.p_z_ab.include_path == 1) or (self.config.p_z_ab.include_path == 3):
                self.tel.set_phase_ab_M1(z_aber.calc_phase_screen(
                    cube=self.config.p_z_ab.zcube_spup,
                    coeff=self.config.p_z_ab.coeff_series[self.config.p_z_ab.phase_index, :],
                    n_zpol=self.config.p_z_ab.num_zpol))

            # calculate and set phase-screen for mpupil
            if (self.config.p_z_ab.include_path == 2) or (self.config.p_z_ab.include_path == 3):
                self.tel.set_phase_ab_M1_m(z_aber.calc_phase_screen(
                    cube=self.config.p_z_ab.zcube_mpup,
                    coeff=self.config.p_z_ab.coeff_series[self.config.p_z_ab.phase_index, :],
                    n_zpol=self.config.p_z_ab.num_zpol))

            self.config.p_z_ab.phase_index += 1

        # save variables in mat-file
        if self.config.p_matlog.log_mat:
            mat_log.mat_file_write(log_mat=self.config.p_matlog.log_mat,
                                   mat_file_name=self.config.p_matlog.mat_file_name,
                                   mat_file_dir=self.config.p_matlog.mat_file_dir,
                                   mat_num_dig=self.config.p_matlog.mat_num_dig,
                                   mat_var=self.config.p_matlog.mat_var,
                                   mat_cmds=self.config.p_matlog.mat_cmds,
                                   index=self.config.p_matlog.save_index, sim=self)

            self.config.p_matlog.save_index += 1

    def end_addition(self):
        """
        Additional code executed after simulation
        """
        # write log-file for MATLAB ud controller
        ud_ctrl.write_log(self)

        # disconnect Python from MATLAB
        if self.config.p_ud_ctrl.engine != None:
            self.config.p_ud_ctrl.engine.rmpath(self.config.p_ud_ctrl.udc_file_dir, nargout=0)
            self.config.p_ud_ctrl.engine.quit()

def load_config_from_file(sim_class, filepath: str) -> None:
    """
    Load the parameters from the parameters file
    :parameters:
        filepath: (str): path to the parameters file
    """
    sim_class.loaded = False
    sim_class.is_init = False
    filename = filepath.split('/')[-1]
    if (len(filepath.split('.')) > 1 and filepath.split('.')[-1] != "py"):
        raise ValueError("Config file must be .py")

    pathfile = filepath.split(filename)[0]
    if (pathfile not in sys.path):
        sys.path.insert(0, pathfile)

    print("loading: %s" % filename.split(".py")[0])
    sim_class.config = __import__(filename.split(".py")[0])
    del sys.modules[sim_class.config.__name__]  # Forced reload
    sim_class.config = __import__(filename.split(".py")[0])

    # exec("import %s as wao_config" % filename.split(".py")[0])
    sys.path.remove(pathfile)

    # Set missing config attributes to None
    if not hasattr(sim_class.config, 'p_loop'):
        sim_class.config.p_loop = None
    if not hasattr(sim_class.config, 'p_geom'):
        sim_class.config.p_geom = None
    if not hasattr(sim_class.config, 'p_tel'):
        sim_class.config.p_tel = None
    if not hasattr(sim_class.config, 'p_atmos'):
        sim_class.config.p_atmos = None
    if not hasattr(sim_class.config, 'p_dms'):
        sim_class.config.p_dms = None
    if not hasattr(sim_class.config, 'p_target'):
        sim_class.config.p_target = None
    if not hasattr(sim_class.config, 'p_wfss'):
        sim_class.config.p_wfss = None
    if not hasattr(sim_class.config, 'p_centroiders'):
        sim_class.config.p_centroiders = None
    if not hasattr(sim_class.config, 'p_controllers'):
        sim_class.config.p_controllers = None

    # Set missing custom config attributes to default values
    if not hasattr(sim_class.config, 'p_t_ctrl'):
        sim_class.config.p_t_ctrl = conf.Param_t_control()
    if not hasattr(sim_class.config, 'p_matlog'):
        sim_class.config.p_matlog = conf.Param_mat_logger()
    if not hasattr(sim_class.config, 'p_z_ab'):
        sim_class.config.p_z_ab = conf.Param_z_aberration()
    if not hasattr(sim_class.config, 'p_ud_ctrl'):
        sim_class.config.p_ud_ctrl = conf.Param_ud_control()

    if not hasattr(sim_class.config, 'simul_name'):
        sim_class.config.simul_name = None

    sim_class.loaded = True
