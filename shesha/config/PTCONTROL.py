"""
Param_t_control class definition
Parameters for Time Control utility (enables oversampling)
@author: neureuther
"""

import numpy as np

#################################################
# P-Class (parametres) Time Control
#################################################
class Param_t_control:
    
    #%%
    #################################################
    # DEFINITION
    #################################################
    def __init__(self):
        """
        Parameters to configure the time control
        os_dec          Decimation from sub-cycle to main-cycle
                        (= main_ctime / sub_ctime) (int)
        main_ctime      Time of one main-cycle/-step (float)
        sub_ctime       Time of one sub-cycle/-step (float)
        loop_it         Number of the current sub-cycle (int)
        wfs_images      List of matrices containing the cumulated wfs-images 
                        since the last main-cycle (list of np.ndarrays[dtype=np.float32])
        ctrl_cmds       List of matrices containing the mirror-cmds/-positions
                        until the next main-cycle (list of np.ndarrays[dtype=np.float32])
                        1st row of matrix = mirror-cmd for 1st sub-cycle after main-cycle
                        2nd row of matrix = mirror-cmd for 2nd sub-cycle after main-cycle
        main_cnt        Number of following sub-cycles until next main-cycle (int)
                        0 = this cycle is a main-cycle
                        1 = next cylce is a main-cycle
        bool_pyr        Boolean if one/several pyramid wfs are simulated (boolean)
        pyr_dec         Modulation points per sub-cycle (int)
        mod_cx          x-positions of the modulation points (only for pyramid 
                        wfs, np.ndarrays[dtype=np.float32])
                        every row represents the positions for one wfs
        mod_cy          y-positions of the modulation points (only for pyramid 
                        wfs, np.ndarrays[dtype=np.float32])
                        every row represents the positions for one wfs
        mod_scale       Vector of scales of the modulation points
        elec_noise      Vector of electronic-noise-stds for all wfs 
                        (np.ndarrays[dtype=float])
        noise_seed      Seed for the wfs-noises (int)
        flux_per_sub    List containing the fluxPerSub for each wfs (every entry = 
                        fluxPerSub for one wfs; list of np.ndarrays[dtype=np.float32])
        halfxy          List containing the halfxy for each wfs (every entry = 
                        halfxy for one wfs; list of np.ndarrays[dtype=np.float32])
        warnings        List containing all warnings raised during initialization
        """
        self.__os_dec = 1
        self.__main_ctime = 0.0
        self.__sub_ctime = 0.0
        self.__loop_it = 0
        self.__wfs_images = None
        self.__ctrl_cmds = None
        self.__main_cnt = 0
        self.__bool_pyr = False
        self.__pyr_dec = 0
        self.__mod_cx = None
        self.__mod_cy = None
        self.__mod_scale = None
        self.__elec_noise = None
        self.__noise_seed = 0
        self.__flux_per_sub = None
        self.__halfxy = None
        self.__warnings = []
        
    
    #%%
    #################################################
    # SET-COMMANDS
    #################################################
    def set_os_dec(self, dec):
        """ Set decimation from sub-cycle to main-cycle
        
        :parameters:
            dec: (int) : decimation from sub-cycle to main-cycle
        """
        if (type(dec) != int) and (type(dec) != np.int32) and (type(dec) != np.int64):
            raise TypeError("os_dec must be a POSITIVE integer.")
        if dec <= 0:
            raise TypeError("os_dec must be a POSITIVE integer.")
        if dec > 50:
            self.__warnings.append("os_dec seems to be very large. Make sure that this is correct!") 
        
        self.__os_dec = int(dec)
        
    os_dec = property(lambda x: x.__os_dec, set_os_dec)
    
    #%%
    def set_main_ctime(self, time):
        """ Set time of one main-cycle/-step
        
        :parameters:
            time: (float) : time of one main-cycle/-step
        """
        if (type(time) != float) and (type(time) != np.float64):
            raise TypeError("main_ctime must be a POSITIVE 64-bit float.")
        if time <= 0:
            raise TypeError("main_ctime must be a POSITIVE 64-bit float.")
        
        self.__main_ctime = np.float64(time)
        
    main_ctime = property(lambda x: x.__main_ctime, set_main_ctime)
    
    #%%
    def set_sub_ctime(self, time):
        """ Set time of one sub-cycle/-step
        
        :parameters:
            time: (float) : time of one sub-cycle/-step
        """
        if (type(time) != float) and (type(time) != np.float64):
            raise TypeError("sub_ctime must be a POSITIVE 64-bit float.")
        if time <= 0:
            raise TypeError("sub_ctime must be a POSITIVE 64-bit float.")
        
        self.__sub_ctime = np.float64(time)
        
    sub_ctime = property(lambda x: x.__sub_ctime, set_sub_ctime)
    
    #%%
    def set_loop_it(self, it):
        """ Set number of the current sub-cycle
        
        :parameters:
            it: (int) : number of the current sub-cycle
        """
        if (type(it) != int) and (type(it) != np.int32) and (type(it) != np.int64):
            raise TypeError("loop_it must be a integer.")
        if it < 0:
            raise TypeError("loop_it must be a NON-NEGATIVE integer.")
        
        self.__loop_it = int(it)
    
    loop_it = property(lambda x: x.__loop_it, set_loop_it)
    
    #%%
    def init_wfs_images(self, size):
        """ Initializes list of matrices containing the cumulate wfs-images
        (one list entry for each cumulates image)
        
        :parameters:
            size: (int) : number of wfs
        """
        if (type(size) != int) and (type(size) != np.int64) and (type(size) != np.int32):
            raise TypeError("The number of wfs must be a POSITIVE int.")
        if size <= 0:
            raise TypeError("The number of wfs must be a POSITIVE int.")
        
        self.__wfs_images = [None]*int(size)
    
    #%%
    def set_wfs_images(self, content):
        """ Set complete list of matrices containing the cumulated wfs-images
        
        :parameters:
            content: (list of np.ndarrays[ndim = 2/3, dtype=np.float32]) : 
                list of matrices containing the cumulated wfs-images
        """
        # check if content seems ok
        if not all(type(x) == np.ndarray for x in content):
            raise TypeError("input for wfs_images must be a list of numpy.ndarray.")
        if not all((len(x.shape) == 2) or (len(x.shape) == 3) for x in content):
            raise TypeError("input for wfs_images must be a list of ndarray with exactly 2 or 3 dimensions.\n" +
                            "ndim = 2 for pyramid wfs \n" + "ndim = 3 for SH wfs")
        if not all(x.dtype == np.float32 for x in content):
             raise TypeError("input for wfs_images must be a list of np.ndarrays[ndim = 2/3, dtype=np.float32].")
         
        self.__wfs_images = content
        
    wfs_images = property(lambda x: x.__wfs_images, set_wfs_images)
    
    #%%
    def set_single_wfs_image(self, idx, content):
        """ Set one element of list of matrices containing the cumulated wfs-images
        
        :parameters:
            idx: (int) : index of the wfs-image to be set
            content: (np.ndarrays[ndim = 2/3, dtype=np.float32]) : cumulated wfs-image
        """
        # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for wfs_image must be a int.")
        if (idx < 0) or (idx > (len(self.__wfs_images)-1)):
            raise TypeError("index for wfs_image exceeds list-length.")
            
        # check if content seems ok
        if type(content) != np.ndarray:
            raise TypeError("input for wfs_image must be a numpy.ndarray.")
        if (len(content.shape) != 3) and (len(content.shape) != 2):
            raise TypeError("input for wfs_image must be a ndarray with exactly 2 or 3 dimensions.\n" +
                            "ndim = 2 for pyramid wfs \n" + "ndim = 3 for SH wfs")
        if content.dtype != np.float32:
             raise TypeError("input for wfs_image must be a np.ndarrays[ndim = 2/3, dtype=np.float32].")
        
        self.__wfs_images[idx] = content
    
    #%%
    def get_single_wfs_image(self, idx):
        """ Get one element of list of matrices containing the cumulated wfs-images
        
        :parameters:
            idx: (int) : index of the wfs-image to be get
        """
         # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for wfs_image must be a int.")
        if (idx < 0) or (idx > (len(self.__wfs_images)-1)):
            raise TypeError("index for wfs_image exceeds list-length.")
        
        return self.__wfs_images[idx]
    
    #%%
    def init_ctrl_cmds(self, size):
        """ Initializes list of matrices containing the mirror-cmds/-positions
        (one list entry for each mirror)
        
        :parameters:
            size: (int) : number of mirrors
        """
        if (type(size) != int) and (type(size) != np.int64) and (type(size) != np.int32):
            raise TypeError("The number of mirrors must be a POSITIVE int.")
        if size <= 0:
            raise TypeError("The number of mirrors must be a POSITIVE int.")
        
        self.__ctrl_cmds = [None]*int(size)
    
    #%%
    def set_ctrl_cmds(self, content):
        """ Set complete list of matrices containing the mirror-cmds/-positions
        
        :parameters:
            content: (list of np.ndarrays[ndim = 2, dtype=np.float32]) : 
                list of vector containing the mirror-cmds/-positions
        """
        # check if content seems ok
        if not all(type(x) == np.ndarray for x in content):
            raise TypeError("input for mirror-cmds must be a list of numpy.ndarray.")
        if not all(len(x.shape) == 2 for x in content):
            raise TypeError("input for mirror-cmds must be a list of ndarray with exactly 2 dimension.")
        if not all(x.dtype == np.float32 for x in content):
             raise TypeError("input for mirror-cmds must be a list of np.ndarrays[ndim = 2, dtype=np.float32].")
         
        self.__ctrl_cmds = content
        
    ctrl_cmds = property(lambda x: x.__ctrl_cmds, set_ctrl_cmds)
    
    #%%
    def set_single_ctrl_cmd(self, idx, content):
        """ Set one element of list of matrices containing the mirror-cmds/-positions
        
        :parameters:
            idx: (int) : index of the mirror-cmd to be set
            content: (np.ndarrays[ndim = 2, dtype=np.float32]) : mirror-cmd
        """
        # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for ctrl_cmds must be a int.")
        if (idx < 0) or (idx > (len(self.__ctrl_cmds)-1)):
            raise TypeError("index for ctrl_cmds exceeds list-length.")
            
        # check if content seems ok
        if type(content) != np.ndarray:
            raise TypeError("input for ctrl_cmds must be a numpy.ndarray.")
        if len(content.shape) != 2:
            raise TypeError("input for ctrl_cmds must be a ndarray with exactly 2 dimensions.")
        if len(content[:,0]) != self.__os_dec:
            raise TypeError("input for ctrl_cmds must be a ndarray with number of rows = os_dec.")
        if content.dtype != np.float32:
             raise TypeError("input for ctrl_cmds must be a np.ndarrays[ndim = 2, dtype=np.float32].")
        
        self.__ctrl_cmds[idx] = content
    
    #%%
    def get_single_ctrl_cmd(self, idx_l, idx_r):
        """ Get one element of list of matrices containing the mirror-cmds/-positions
        
        :parameters:
            idx_l: (int) : list-index of the mirror-cmd to be get
            idx_r: (int) : row-index of the mirror-cmd to be get
        """
        # check if idx_l is ok
        if (type(idx_l) != int) and (type(idx_l) != np.int64) and (type(idx_l) != np.int32):
            raise TypeError("list-index for ctrl_cmds must be a int.")
        if (idx_l < 0) or (idx_l > (len(self.__ctrl_cmds)-1)):
            raise TypeError("list-index for ctrl_cmds exceeds list-length.")
        
        # check if idx_r is ok
        if (type(idx_r) != int) and (type(idx_r) != np.int64) and (type(idx_r) != np.int32):
            raise TypeError("row-index for ctrl_cmds must be a int.")
        if (idx_r < 0) or (idx_r > (self.__os_dec-1)):
            raise TypeError("row-index for ctrl_cmds exceeds number of matrix rows.")
        
        return self.__ctrl_cmds[idx_l][idx_r,:]
    
    #%%
    def set_main_cnt(self, num):
        """ Set number of following sub-cycles until next main-cycle
        
        :parameters:
            num: (int) : number of following sub-cycles until next main-cycle
        """
        if (type(num) != int) and (type(num) != np.int64) and (type(num) != np.int32):
            raise TypeError("main_cnt must be a NON-NEGATIVE integer.")
        if (num < 0) or (num > (self.__os_dec - 1)) :
            raise TypeError("main_ctime must be a NON-NEGATIVE integer and can't be larger than os_dec.")
        
        self.__main_cnt = int(num)
    
    main_cnt = property(lambda x: x.__main_cnt, set_main_cnt)
    
    #%%
    def set_bool_pyr(self, b_pyr):
        """ Set if one/several pyramid wfs are simulated 
        
        :parameters:
            b_pyr: (bool)) : one/several pyramid wfs are simulated 
        """
        if type(b_pyr) != bool:
            raise TypeError("bool_pyr must be a boolean")
        
        self.__bool_pyr = b_pyr
    
    bool_pyr = property(lambda x: x.__bool_pyr, set_bool_pyr)
    
    #%%
    def set_pyr_dec(self, dec):
        """ Set modulation points per sub-cycle
        
        :parameters:
            dec: (int) : modulation points per sub-cycle
        """
        if (type(dec) != int) and (type(dec) != np.int32) and (type(dec) != np.int64):
            raise TypeError("pyr_dec must be a POSITIVE integer.")
        if dec <= 1:
            raise TypeError("pyr_dec must be a POSITIVE integer larger than 1.")
        if dec > 10:
            self.__warnings.append("pyr_dec seems to be very large. Make sure that this is correct!")
        
        self.__pyr_dec = int(dec)
        
    pyr_dec = property(lambda x: x.__pyr_dec, set_pyr_dec)
    
    #%% 
    def set_mod_cx(self, cx):
        """ Set x-positions of the modulation points (only for pyramid wfs)
        
        :parameters:
            cx: (np.ndarrays[dim=2, dtype=np.float32]) : x-positions of mod. points
        """
        # check if content seems ok
        if type(cx) != np.ndarray:
            raise TypeError("cx must be a numpy.ndarray.")
        if len(cx.shape) != 2:
            raise TypeError("cx must be a ndarray with exactly 2 dimensions.")
        if len(cx[0,:]) != (self.__os_dec * self.__pyr_dec):
            raise TypeError("cx must be a ndarray with number of cols = os_dec * pyr_dec.")
        if cx.dtype != np.float32:
             raise TypeError("cx must be a np.ndarrays[ndim=2, dtype=np.float32].")
         
        self.__mod_cx = cx
    
    mod_cx = property(lambda x: x.__mod_cx, set_mod_cx)
    
    #%% 
    def set_mod_cy(self, cy):
        """ Set y-positions of the modulation points (only for pyramid wfs)
        
        :parameters:
            cy: (np.ndarrays[dim=2, dtype=np.float32]) : y-positions of mod. points
        """
        # check if content seems ok
        if type(cy) != np.ndarray:
            raise TypeError("cy must be a numpy.ndarray.")
        if len(cy.shape) != 2:
            raise TypeError("cy must be a ndarray with exactly 2 dimensions.")
        if len(cy[0,:]) != (self.__os_dec * self.__pyr_dec):
            raise TypeError("cy must be a ndarray with number of cols = os_dec * pyr_dec.")
        if cy.dtype != np.float32:
             raise TypeError("cy must be a np.ndarrays[ndim=2, dtype=np.float32].")
         
        self.__mod_cy = cy
    
    mod_cy = property(lambda x: x.__mod_cy, set_mod_cy)
    
    #%%
    def set_mod_scale(self, scale):
        """ Set vector of scales of the modulation points
        
        :parameters:
            scale: (np.ndarrays[ndim=1, dtype=np.float32]) : Vector of scales of the
                                                             modulation points
        """
        if type(scale) != np.ndarray:
            raise TypeError("scale must be a numpy.ndarray.")
        if len(scale.shape) != 1:
            raise TypeError("scale must be a ndarray with exactly 1 dimensions.")
        if scale.dtype != np.float32:
             raise TypeError("scale must be a np.ndarrays[ndim=1, dtype=np.float32].")
        
        self.__mod_scale = scale
        
    mod_scale = property(lambda x: x.__mod_scale, set_mod_scale)
    
    #%%
    def set_elec_noise(self, content):
        """ Set vector of electronic-noise-stds for all wfs 
        
        :parameters:
            content: (np.ndarrays[ndim=1, dtype=float]) : vector of
                    electronic-noise-stds for all wfs 
        
        """
        if (type(content) != np.ndarray) and (type(content) != list):
            raise TypeError("elec_noise must be a numpy.ndarray or a list.")
        if not all(isinstance(x, (int, float, np.float32, np.float64, np.int32, np.int64)) for x in content):
            raise TypeError("elec_noise may only consists out of int and floats.")
        
        temp = np.asarray(content, dtype = float)
        
        if len(temp.shape) != 1:
            raise TypeError("elec_noise must be a ndarray with exactly 1 dimension.")
         
        self.__elec_noise = temp

    elec_noise = property(lambda x: x.__elec_noise, set_elec_noise)
    
    #%%
    def set_noise_seed(self, seed):
        """ Set seed for the wfs-noises
        
        :parameters:
            seed: (int) : Seed for the wfs-noises
        
        """
        if (type(seed) != int) and (type(seed) != np.int32) and (type(seed) != np.int64):
            raise TypeError("noise_seed must be a POSITIVE integer.")
        if seed <= 0:
            raise TypeError("noise_seed must be a POSITIVE integer.")
        
        self.__noise_seed = int(seed)
    
    noise_seed = property(lambda x: x.__noise_seed, set_noise_seed)
    
    #%%
    def init_flux_per_sub(self, size):
        """ Initializes list of fluxPerSub
        (one list entry for each wfs)
        
        :parameters:
            size: (int) : number of wfs
        """
        if (type(size) != int) and (type(size) != np.int64) and (type(size) != np.int32):
            raise TypeError("The number of wfs must be a POSITIVE int.")
        if size <= 0:
            raise TypeError("The number of wfs must be a POSITIVE int.")
        
        self.__flux_per_sub = [None]*int(size)
    
    #%%
    def set_flux_per_sub(self, content):
        """ Set complete list of fluxPerSub
        
        :parameters:
            content: (list of np.ndarrays[ndim = 1, dtype=np.float32]) : 
                list of fluxPerSub
        """
        # check if content seems ok
        if not all(type(x) == np.ndarray for x in content):
            raise TypeError("input for flux_per_sub must be a list of numpy.ndarray.")
        if not all(len(x.shape) == 1 for x in content):
            raise TypeError("input for flux_per_sub must be a list of ndarray with exactly 1 dimension.")
        if not all(x.dtype == np.float32 for x in content):
             raise TypeError("input for flux_per_sub must be a list of np.ndarrays[ndim = 1, dtype=np.float32].")
         
        self.__flux_per_sub = content
        
    flux_per_sub = property(lambda x: x.__flux_per_sub, set_flux_per_sub)
    
    #%%
    def set_single_flux_per_sub(self, idx, content):
        """ Set one element of list of fluxPerSub
        
        :parameters:
            idx: (int) : index of list resp. corresponding wfs
            content: (np.ndarrays[ndim = 1, dtype=np.float32]) : fluxPerSub
        """
        # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for flux_per_sub must be a int.")
        if (idx < 0) or (idx > (len(self.__flux_per_sub)-1)):
            raise TypeError("index for flux_per_sub exceeds list-length.")
        
        # check if content seems ok
        if type(content) != np.ndarray:
            raise TypeError("input for flux_per_sub must be a numpy.ndarray.")
        if len(content.shape) != 1:
            raise TypeError("input for flux_per_sub must be a ndarray with exactly 1 dimension.")
        if content.dtype != np.float32:
             raise TypeError("input for flux_per_sub must be a np.ndarrays[ndim = 1, dtype=np.float32].")
        
        self.__flux_per_sub[idx] = content
    
    #%%
    def get_single_flux_per_sub(self, idx):
        """ Get one element of list of fluxPerSub
        
        :parameters:
            idx: (int) : index of list resp. corresponding wfs
        """
         # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for flux_per_sub must be a int.")
        if (idx < 0) or (idx > (len(self.__flux_per_sub)-1)):
            raise TypeError("index for flux_per_sub exceeds list-length.")
        
        return self.__flux_per_sub[idx]
    
    #%%
    def init_halfxy(self, size):
        """ Initializes list of halfxy
        (one list entry for each wfs)
        
        :parameters:
            size: (int) : number of wfs
        """
        if (type(size) != int) and (type(size) != np.int64) and (type(size) != np.int32):
            raise TypeError("The number of wfs must be a POSITIVE int.")
        if size <= 0:
            raise TypeError("The number of wfs must be a POSITIVE int.")
        
        self.__halfxy = [None]*int(size)
    
    #%%
    def set_halfxy(self, content):
        """ Set complete list of halfxy
        
        :parameters:
            content: (list of np.ndarrays[ndim = 2]) : 
                list of halfxy
        """
        # check if content seems ok
        if not all(type(x) == np.ndarray for x in content):
            raise TypeError("input for halfxy must be a list of numpy.ndarray.")
        if not all(len(x.shape) == 2 for x in content):
            raise TypeError("input for halfxy must be a list of ndarray with exactly 2 dimensions.")
         
        self.__halfxy = content
        
    halfxy = property(lambda x: x.__halfxy, set_halfxy)
    
    #%%
    def set_single_halfxy(self, idx, content):
        """ Set one element of list of halfxy
        
        :parameters:
            idx: (int) : index of list resp. corresponding wfs
            content: (np.ndarrays[ndim = 2]) : halfxy
        """
        # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for halfxy must be a int.")
        if (idx < 0) or (idx > (len(self.__halfxy)-1)):
            raise TypeError("index for halfxy exceeds list-length.")
        
        # check if content seems ok
        if type(content) != np.ndarray:
            raise TypeError("input for halfxy must be a numpy.ndarray.")
        if len(content.shape) != 2:
            raise TypeError("input for flux_per_sub must be a ndarray with exactly 2 dimensions.")
        
        self.__halfxy[idx] = content
    
    #%%
    def get_single_halfxy(self, idx):
        """ Get one element of list of halfxy
        
        :parameters:
            idx: (int) : index of list resp. corresponding wfs
        """
         # check if idx is ok
        if (type(idx) != int) and (type(idx) != np.int64) and (type(idx) != np.int32):
            raise TypeError("index for halfxy must be a int.")
        if (idx < 0) or (idx > (len(self.__halfxy)-1)):
            raise TypeError("index for halfxy list-length.")
        
        return self.__halfxy[idx]
    
    #%%
    # List containing all warnings raised during initialization
    warnings = property(lambda x: x.__warnings)
    