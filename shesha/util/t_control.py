"""
Time Control utility (enables oversampling)

Introduce Time Oversampling to simulation

@author: neureuther
"""

import numpy as np
import time

import shesha.config as conf
from shesha.constants import CONST

#%%
def calc_dec(main_ctime, sub_ctime, tol = 1e-10):
    """
    Loads 
    
    :parameters:
        main_ctime: (float): Time of one main-cycle
        sub_ctime: (float): Time of one sub-cycle
        tol: (float): acceptable numeric tolerance between calculated and rounded
                      decimation

    :return:
        dec: (int): Decimation from sub-cycle to main-cycle 
    
    :typical call:
    calc_dec(main_ctime = self.config.p_t_ctrl.main_ctime,
             sub_ctime = self.config.p_t_ctrl.sub_ctime,
             tol = 1e-12)
    """
    
    # ratio of main- and sub-cycle-time
    dec = np.float64( main_ctime / sub_ctime )
    
    # check if sub-cycle-time is smaller than main-cycle-time
    if dec < 1.0:
        raise ArithmeticError("sub-cycle-time must be smaller than main-cycle-time.")
    
    # check if main-cycle-time is not a multiple of sub-cycle-time
    if np.abs( dec - np.round(dec) ) > tol:
        raise ArithmeticError("main-cycle-time is not a multiple of sub-cycle-time.")
    
    return int( np.round(dec) )

#%%
def calc_pyr_mod(p_t_ctrl: conf.Param_t_control, p_wfss: list, tel_diam: float):
    """ Calculate the pyramid modulation points and their scales

    :parameters:
        p_t_ctrl: (Param_t_control): 
        p_wfss: (list of Param_wfs): list of wfs parameters
        tel_diam: (float): diameter of telescope
        ampl: (float): new amplitude in units of lambda/D
    """
    # allocate memory
    cx = np.zeros(shape = (len(p_wfss), p_t_ctrl.os_dec * p_t_ctrl.pyr_dec), dtype = np.float32)
    cy = np.zeros(shape = (len(p_wfss), p_t_ctrl.os_dec * p_t_ctrl.pyr_dec), dtype = np.float32)
    scale = np.zeros(shape = (len(p_wfss),), dtype = np.float32)
    
    pyr_npts = p_t_ctrl.os_dec * p_t_ctrl.pyr_dec
    
    # calc cx and cy for every wavefront-sensor
    for k in range( len(p_wfss) ):
        
        # check if current wfs is a Shack Hartmann sensor
        if p_wfss[k].type.decode("utf-8") == "sh":
            continue
        
        # retrieve amplitude
        ampl = p_wfss[k].pyr_ampl
        
        # scale quantum pixel size
        pixsize = p_wfss[k]._qpixsize * CONST.ARCSEC2RAD
        
        # calc scaleing factor
        scale_fact = 2 * np.pi / p_wfss[k]._Nfft * (p_wfss[k].Lambda * 1e-6 / tel_diam) / pixsize * ampl
        
        # calc x-positions of the modulation points
        cx[k,:] = scale_fact * np.sin((np.arange(pyr_npts, dtype=np.float32)) * 2. * np.pi / pyr_npts)
        
        # calc y-positions of the modulation points
        cy[k,:] = scale_fact * np.cos((np.arange(pyr_npts, dtype=np.float32)) * 2. * np.pi / pyr_npts)
        
        # scale of the modulation points
        scale[k] = p_wfss[k].Lambda * 1e-6 / tel_diam * ampl * 180. / np.pi * 3600.
    
    return [cx, cy, scale]

#%% 
def t_ctrl_init(sim):
    """ Initializes the Time Control utility (enabling oversampling)
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # only initialize if os_dec > 1
    if sim.config.p_t_ctrl.os_dec > 1:
        
        # calc main-cycle time
        sim.config.p_t_ctrl.set_main_ctime(
                sim.config.p_t_ctrl.os_dec * sim.config.p_loop.ittime)
        
        # set sub-cycle time
        sim.config.p_t_ctrl.set_sub_ctime(sim.config.p_loop.ittime)
        
        # set number of current sub-cycle
        sim.config.p_t_ctrl.set_loop_it(0)
        
        # set number of following sub-cycles
        sim.config.p_t_ctrl.set_main_cnt(sim.config.p_t_ctrl.os_dec - 1)
        
        # init list of matrices containing the cumulated wfs-images
        sim.config.p_t_ctrl.init_wfs_images( len(sim.config.p_wfss) )
        
        #nit list of support-variables (flux_per_sub, halfxy)
        sim.config.p_t_ctrl.init_flux_per_sub( len(sim.config.p_wfss) )
        sim.config.p_t_ctrl.init_halfxy( len(sim.config.p_wfss) )
        
        # fill list of matrices containing the cumulated wfs-images
        # fill list of support-variables (flux_per_sub, halfxy)
        for k in range( len(sim.config.p_wfss) ):
            
            # calculate flux_per_sub
            sim.config.p_t_ctrl.set_single_flux_per_sub(idx = k, 
                content = sim.config.p_wfss[k]._fluxPerSub.T[np.where(sim.config.p_wfss[k]._isvalid > 0)].copy() )
            
            # check if wfs is a Shack-Hartmann sensor
            if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
                
                # fill wfs-image-matrix
                sim.config.p_t_ctrl.set_single_wfs_image( idx = k, 
                        content = np.zeros(sim.wfs.get_bincube(k).shape, dtype = np.float32) )
                
                # fill halfxy
                sim.config.p_t_ctrl.set_single_halfxy(idx = k,
                    content = sim.config.p_wfss[k]._halfxy )
            
            # check if wfs is a pyramid sensor
            elif sim.config.p_wfss[k].type.decode("utf-8") == "pyrhr":
                
                # fill wfs-image-matrix 
                sim.config.p_t_ctrl.set_single_wfs_image( idx = k, 
                        content = np.zeros(sim.wfs.get_pyrimg(k).shape, dtype = np.float32) )
                
                # fill halfxy
                sim.config.p_t_ctrl.set_single_halfxy(idx = k,
                    content = np.exp(1j * sim.config.p_wfss[k]._halfxy).astype(np.complex64).T.copy() )
                
                # if there is a pyramid sensor, then bool_pyr is set to True
                sim.config.p_t_ctrl.set_bool_pyr(True)
            
            # anything else (this should not happen)
            else:
                raise TypeError("Oops, something strange happened. It seems" + 
                                "that one wfs is neither sh nor pyrhr.")
        
        # init list of matrices containing the mirror-cmds
        sim.config.p_t_ctrl.init_ctrl_cmds( len(sim.config.p_dms) )
        
        # fill list of matrices containing the mirror-cmds
        for k in range( len(sim.config.p_dms) ):
            
            # fill matrix
            sim.config.p_t_ctrl.set_single_ctrl_cmd( idx = k, 
                content = np.zeros((sim.config.p_t_ctrl.os_dec, sim.config.p_dms[k]._ntotact), 
                                   dtype = np.float32) )
        
        # calculate modulation points for pyramid wfs
        # calc modulation points and their scales
        [cx, cy, scale] = calc_pyr_mod(
                p_t_ctrl = sim.config.p_t_ctrl, 
                p_wfss = sim.config.p_wfss,
                tel_diam = sim.config.p_tel.diam)
        
        # set modulation points
        sim.config.p_t_ctrl.set_mod_cx(cx)
        sim.config.p_t_ctrl.set_mod_cy(cy)
        
        # set scale of modulation points
        sim.config.p_t_ctrl.set_mod_scale(scale)
        
        # initialize modulation points for every wfs
        for k in range( len(sim.config.p_wfss) ):
            
            # check if current wfs is a Shack Hartmann sensor
            if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
                continue
            
            # set modulation points in wfs-object
            sim.wfs.init_arrays(k, sim.config.p_wfss[k]._phasemap, sim.config.p_wfss[k]._hrmap,
                    sim.config.p_t_ctrl.get_single_halfxy(k), 
                    sim.config.p_t_ctrl.get_single_flux_per_sub(k),
                    sim.config.p_wfss[k]._validsubsx, sim.config.p_wfss[k]._validsubsy,
                    sim.config.p_wfss[k]._istart + 1, sim.config.p_wfss[k]._jstart + 1,
                    sim.config.p_wfss[k]._binmap, sim.config.p_wfss[k]._ftkernel,
                    sim.config.p_t_ctrl.mod_cx[k, 0 : sim.config.p_t_ctrl.pyr_dec], 
                    sim.config.p_t_ctrl.mod_cy[k, 0 : sim.config.p_t_ctrl.pyr_dec], 
                    sim.config.p_wfss[k]._sincar, sim.config.p_wfss[k]._submask)
        
        # set seed for readout noise
        # set "random" seed (based on unix time; default option)
        if sim.config.p_t_ctrl.noise_seed == 0:
            np.random.seed( np.uint32(time.time()) )
            
        # set user-defined seed
        else:
            np.random.seed(sim.config.p_t_ctrl.noise_seed)
        
    # print status of time control
    print()
    print("*-------------------------------")
    print("TIME CONTROL")
    
    # if os_dec <= 1 then there is no time oversampling
    if sim.config.p_t_ctrl.os_dec <= 1:
        print("status: disabled")
    
    # there is time oversampling
    else:
        print("status: enabled")
        print("Time decimation: " + str(sim.config.p_t_ctrl.os_dec) )
        print("Main cycle: " + str(sim.config.p_t_ctrl.main_ctime * 1000) + " ms" )
        print("Sub cycle: " + str(sim.config.p_t_ctrl.sub_ctime * 1000) + " ms" )
                    
        # print status of wfs
        if sim.config.p_t_ctrl.bool_pyr:
            print("Pyramid wfs: present")
            print("Pyramid decimation: " + str(sim.config.p_t_ctrl.pyr_dec))
        else:
            print("Pyramid wfs: not present")
        
        # print warnings of initialization
        if len(sim.config.p_t_ctrl.warnings) == 0:
             print("Warnings: none")
        else:
            print("Warnings: ")
            for k in range( len(sim.config.p_t_ctrl.warnings) ):
                print("   " + sim.config.p_t_ctrl.warnings[k])
    
    print("*-------------------------------")

#%%
def add_sub_images(sim):
    """ Add sensor images after every sub-cycle
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # reset cumulated images if main-cycle is over
    if sim.config.p_t_ctrl.main_cnt == (sim.config.p_t_ctrl.os_dec - 1):
        for k in range( len(sim.config.p_t_ctrl.wfs_images) ):
            sim.config.p_t_ctrl.wfs_images[k].fill(0.0)
    
        
    # readout every sensor and store cumulated image
    for k in range( len(sim.config.p_wfss) ):
        
        # READOUT
        # check if wfs is a Shack-Hartmann sensor
        if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
            img_temp = sim.wfs.get_bincube(k)
        
        # check if wfs is a pyramid sensor
        elif sim.config.p_wfss[k].type.decode("utf-8") == "pyrhr":
            img_temp = sim.wfs.get_pyrimg(k)
            
        # anything else (this should not happen)
        else:
            raise TypeError("Oops, something strange happened. It seems" + 
                            "that one wfs is neither sh nor pyrhr.")
        
        # CUMULATE
        img_temp = sim.config.p_t_ctrl.get_single_wfs_image(k) + img_temp
        
        # ADDITION OF READOUT NOISE (at the end of main-cycle)
        if sim.config.p_t_ctrl.main_cnt == 0:
            
            # check if wfs is a Shack-Hartmann sensor
            if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
                img_temp = img_temp + np.random.normal(loc = 0.0, 
                        scale = sim.config.p_t_ctrl.elec_noise[k], 
                        size = sim.wfs.get_bincube(k).shape).astype(np.float32).round()
            
            # check if wfs is a pyramid sensor
            elif sim.config.p_wfss[k].type.decode("utf-8") == "pyrhr":
                img_temp = img_temp + np.random.normal(loc = 0.0, 
                        scale = sim.config.p_t_ctrl.elec_noise[k], 
                        size = sim.wfs.get_pyrimg(k).shape).astype(np.float32).round()
        
        # STORAGE OF IMAGE
        sim.config.p_t_ctrl.set_single_wfs_image(k, img_temp)

#%%
def update_idx(sim):
    """ Update loop_it and main_cnt after every time step
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # update number of the next sub-cycle
    sim.config.p_t_ctrl.loop_it += 1
    
    # update number of sub-cycles until next main-cycle for next sub-cycle
    if sim.config.p_t_ctrl.main_cnt > 0:
        sim.config.p_t_ctrl.main_cnt -= 1
    else:
        sim.config.p_t_ctrl.main_cnt = sim.config.p_t_ctrl.os_dec - 1
    
#%%
def update_pyr_mod(sim):
    """ Update modulation points (cx and cy) of pyramid wfs
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # 
    idx = (sim.config.p_t_ctrl.os_dec - 1) - sim.config.p_t_ctrl.main_cnt + 1
    
    if idx == sim.config.p_t_ctrl.os_dec:
        idx = 0
    
    #print(str(sim.config.p_t_ctrl.loop_it + 1) + "  " + str(idx))
    
    # update modulation points for every wfs
    for k in range( len(sim.config.p_wfss) ):
        
        # check if current wfs is a Shack Hartmann sensor
        if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
            continue
        
        # set modulation points in wfs-object
        sim.wfs.init_arrays(k, sim.config.p_wfss[k]._phasemap, sim.config.p_wfss[k]._hrmap,
            sim.config.p_t_ctrl.get_single_halfxy(k), 
            sim.config.p_t_ctrl.get_single_flux_per_sub(k),
            sim.config.p_wfss[k]._validsubsx, sim.config.p_wfss[k]._validsubsy,
            sim.config.p_wfss[k]._istart + 1, sim.config.p_wfss[k]._jstart + 1,
            sim.config.p_wfss[k]._binmap, sim.config.p_wfss[k]._ftkernel,
            sim.config.p_t_ctrl.mod_cx[k, sim.config.p_t_ctrl.pyr_dec * idx : sim.config.p_t_ctrl.pyr_dec * (idx+1)],
            sim.config.p_t_ctrl.mod_cy[k, sim.config.p_t_ctrl.pyr_dec * idx : sim.config.p_t_ctrl.pyr_dec * (idx+1)], 
            sim.config.p_wfss[k]._sincar, sim.config.p_wfss[k]._submask)

    
#%%
def upload_images(sim):
    """ Load cumulated images onto the GPU
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # upload cumulated images only if simulation is in main-cycle
    if sim.config.p_t_ctrl.main_cnt == 0:
            
        # upload all cumulated images to GPU
        for k in range( len(sim.config.p_wfss) ):
            
            # check if wfs is a Shack-Hartmann sensor
            if sim.config.p_wfss[k].type.decode("utf-8") == "sh":
                sim.wfs.set_bincube(k, sim.config.p_t_ctrl.get_single_wfs_image(k))
            
            # check if wfs is a pyramid sensor
            elif sim.config.p_wfss[k].type.decode("utf-8") == "pyrhr":
                sim.wfs.set_pyrimg(k, sim.config.p_t_ctrl.get_single_wfs_image(k))
                
            # anything else (this should not happen)
            else:
                raise TypeError("Oops, something strange happened. It seems" + 
                                "that one wfs is neither sh nor pyrhr.")
    