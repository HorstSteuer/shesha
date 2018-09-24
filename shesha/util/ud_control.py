"""
User Defined Control utility

Introduce user defined controller to simulation

@author: neureuther
"""

import io
import matlab.engine
import sys
import os
import numpy as np
import time


#%%
def ud_ctrl_init(sim):
    """ Initializes the User Defined Control utility
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    
    # if UD Control is used
    if sim.config.p_ud_ctrl.use_udc:
        
        # Common initialization
        # Begining of status message
        print()
        print("*-------------------------------")
        print("USER DEFINED CONTROL")
        print("status: enabled")
        print("ctrl script name: " + str(sim.config.p_ud_ctrl.udc_file_name)) 
        print("ctrl script directory: " + str(sim.config.p_ud_ctrl.udc_file_dir))
                
               
        # Python specific initialization
        if sim.config.p_ud_ctrl.udc_session == "python":
            
            # add directory of udc to python search path
            sys.path.append(sim.config.p_ud_ctrl.udc_file_dir)
            
            # load udc file
            sim.config.p_ud_ctrl.udc_mod = __import__(sim.config.p_ud_ctrl.udc_file_name)
            
            # construct string to call the ud python controller
            sim.config.p_ud_ctrl.set_func_call("sim.config.p_ud_ctrl.udc_mod." + 
                sim.config.p_ud_ctrl.udc_file_name + "(sim)")
            
            # specific status message
            print("ctrl script languge: Python")
            
        # MATLAB specific initialization
        else:
            
            # initialize ouput stream
            sim.config.p_ud_ctrl.mat_output = io.StringIO()
            
            # construct string to call the ud MATLAB controller
            call_str = "sim.config.p_ud_ctrl.engine." + sim.config.p_ud_ctrl.udc_file_name + "("
            
            # append COMPASS commands to call-string
            for k in range( len(sim.config.p_ud_ctrl.func_param) ):
                
                # seperate inputs with commas
                if k > 0:
                    call_str = call_str + ", "
                
                # append COMPASS commands
                call_str = call_str + sim.config.p_ud_ctrl.func_param[k]
            
            # append parameter-dictionary
            call_str = call_str + ", sim.config.p_ud_ctrl.param, nargout = 1, " + \
                "stdout = sim.config.p_ud_ctrl.mat_output)"
                
            # add some important params to parameter-dictionary
            sim.config.p_ud_ctrl.param.update(
                {"it_time": sim.config.p_loop.ittime,
                 "t_dec": sim.config.p_t_ctrl.os_dec,
                 "n_act": [x._ntotact for x in sim.config.p_dms]})
            
            # save constructed call-string
            sim.config.p_ud_ctrl.set_func_call(call_str)
            
            # opening MATLAB
            # new MATLAB instance
            if sim.config.p_ud_ctrl.udc_session == "new matlab":
                
                # initial status message
                print("MATLAB is starting ...", end="\r")
                
                # start new MATLAB instance
                sim.config.p_ud_ctrl.engine = matlab.engine.start_matlab("-desktop")
            
            # connect to single running MATLAB instance
            elif sim.config.p_ud_ctrl.udc_session == "find matlab":
                
                # find all running MATLAB instances
                res = matlab.engine.find_matlab()
                
                # check if only one MATLAB session is running
                if len(res) != 1:
                    raise RuntimeError("The number of running MATLAB sessions is != 1. " + 
                                       "Currently there are " + str(len(res)) + " running.")
                
                # connect to running MATLAB session
                sim.config.p_ud_ctrl.engine = matlab.engine.connect_matlab(res[0])
            
            # connect to one specified MATLAB instance
            else:
                
                # find all running MATLAB instances
                res = matlab.engine.find_matlab()
                
                # check if specified MATLAB instance is running
                if not any(x == sim.config.p_ud_ctrl.udc_session for x in res):
                    raise RuntimeError("The specified MATLAB instance is not running.")
                
                # connect to running MATLAB session
                sim.config.p_ud_ctrl.engine = matlab.engine.connect_matlab(
                                                    sim.config.p_ud_ctrl.udc_session)
                
            # add directory of udc to MATLAB search path
            sim.config.p_ud_ctrl.engine.addpath(sim.config.p_ud_ctrl.udc_file_dir, nargout=0)
            
            # specific status message
            print("ctrl script languge: MATLAB")
           
    
        # End of status message
        # timing according to time control util
        if sim.config.p_t_ctrl.os_dec > 1:
            print("timing: controlled by TIME CONTROL")
        else:
            print("timing: every sim step")
        
        # application of controller commands to mirrors
        if sim.config.p_ud_ctrl.cmd_apply:
            print("command application: enabled")
        else:
            print("command application: disabled")
        
        print("*-------------------------------")
        
    else:
        print()
        print("*-------------------------------")
        print("USER DEFINED CONTROL")
        print("status: disabled")
        print("*-------------------------------")

#%%
def eval_ud_ctrl(sim):
    """ Evaluate the User Defined Controller.
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # if sim is in main cycle evaluate the ud controller
    if sim.config.p_t_ctrl.main_cnt == 0:
        
        # do centroiding on all wfs-images
        sim.rtc.do_centroids(0)
        
        # evaluate ud controller
        cmds_list = eval(sim.config.p_ud_ctrl.func_call)
        
        # convert MATLAB matrices to numpy matrices
        if sim.config.p_ud_ctrl.engine != None:
            cmds_list = [np.array(x, dtype = np.float32) for x in cmds_list]
        
        # store mirror-cmds/-positions
        sim.config.p_t_ctrl.set_ctrl_cmds(cmds_list)


#%%
def write_log(sim):
    """ Write messages for MATLAB ud controller to a log-file.
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # write log-file only for MATLAB ud controller
    if sim.config.p_ud_ctrl.engine != None:
        
        # create full dir and name of log-file
        file_name = sim.config.p_ud_ctrl.udc_file_dir + "log.txt"
        
        # delete older log-file
        if os.path.isfile(file_name):
            os.remove(file_name)
            
        # write MATLAB messages to file
        with open(file_name, "w") as text_file:
            
            # file header
            text_file.write("LOG FILE FOR A COMPASS SIMULATION EMPOLYING A MATLAB CONTROLLER \n")
            text_file.write(" \n")
            
            text_file.write("time of simulation: " + 
                            time.strftime("%A %d.%m.%Y %H:%M:%S", time.localtime(time.time())) + 
                            " \n")
            
            text_file.write("simulation successfully finished \n")
            text_file.write(" \n")
            text_file.write("messages MATLAB outputed during the simulation: \n")
            text_file.write(" \n")
            
            # MATLAB messages
            text_file.write(sim.config.p_ud_ctrl.mat_output.getvalue())
        
#%%
def update_mirror_shape(sim):
    """ Update and apply the mirror-commands/-positions.
    
    :parameters:
        sim: (Simulator) : simulator object (in simulator.py = self)
    """
    # check whether mirror-cmds/-positions are applied
    if sim.config.p_ud_ctrl.cmd_apply:
        
        # index of mirror command for the next sub cycle
        idx = sim.config.p_t_ctrl.os_dec - sim.config.p_t_ctrl.main_cnt
        
        if idx == sim.config.p_t_ctrl.os_dec:
            idx = 0
        
        # retrieve mirror-cmds/-positions
        cur_cmds = [None] * len(sim.config.p_dms)
        
        for k in range( len(sim.config.p_dms) ):
            cur_cmds[k] = sim.config.p_t_ctrl.get_single_ctrl_cmd(k, idx)
        
        # set mirror-cmds/-positions in the rtc-object
        sim.rtc.set_com(0, np.hstack(cur_cmds))
        
        # limit mirror deflection
        sim.rtc.do_clipping(0, -1e5, 1e5)
        
        # apply mirror-cmds/-positions
        sim.rtc.apply_control(0, sim.dms)
        