"""
Param_mat_logger class definition
Parameters for Mat Logger utility
@author: neureuther
"""

import numpy as np

#################################################
# P-Class (parametres) Mat Logger
#################################################
class Param_mat_logger:
    
    #%%
    #################################################
    # DEFINITION
    #################################################
    def __init__(self):
        """
        Parameters to configure the saving of variables to a mat-File (Mat Logger)
        log_mat         if true: variables are saved to mat-file (bool)
        log_init        if true: variables after initialization are saved (bool)
        mat_file_dir    directory of (to be created) mat-file (string)
        mat_file_name   file-name of (to be created) mat-file (string)
        mat_num_dig     number of digits used in the mat-file variables, must be
                        larger than niter (int)
        mat_var         list of variable names used in (to be created) mat-file
                        (list of strings)
        mat_cmds        list of COMPASS commands used to retrieve desiered data
                        (list of strings)
        save_index      Index for mat-file-logging (int)
        """
        self.__log_mat = False
        self.__log_init = False
        self.__mat_file_dir = "default"
        self.__mat_file_name = "COMPASS_sim.mat"
        self.__mat_num_dig = 4
        self.__mat_var = []
        self.__mat_cmds = []
        self.__save_index = 0
    
    
    #%%
    #################################################
    # SET-COMMANDS
    #################################################
    def set_log_mat(self, log_bool):
        """ Set if data will be logged in a mat-file
        
        :parameters:
            log_bool: (bool) : bool determining if data will be logged in a mat-file
        """
        if type(log_bool) != bool:
            raise TypeError("log_mat must be a boolean (True or False)")
        
        self.__log_mat = log_bool
        
    log_mat = property(lambda x: x.__log_mat, set_log_mat)
    
    #%%
    def set_log_init(self, init_bool):
        """ Set if data after initialization will be logged in the mat-file
        
        :parameters:
            init_bool: (bool) : bool determining if data after initialization are
                                logged in a mat-file
        """
        if type(init_bool) != bool:
            raise TypeError("log_init must be a boolean (True or False).")
        
        self.__log_init = init_bool
        
    log_init = property(lambda x: x.__log_init, set_log_init)
    
    #%%
    def set_mat_file_dir(self, file_dir):
        """ Set the directory of (to be created) mat-file
        
        :parameters:
            file_dir: (string) : directory of (to be created) mat-file
        """
        if type(file_dir) != str:
            raise TypeError("mat_file_dir must be a string.")
        
        self.__mat_file_dir = file_dir
    
    mat_file_dir = property(lambda x: x.__mat_file_dir, set_mat_file_dir)
    
    #%%
    def set_mat_file_name(self, file_name):
        """ Set the file-name of (to be created) mat-file
        
        :parameters:
            file_name: (string) : file-name of (to be created) mat-file
        """
        if type(file_name) != str:
            raise TypeError("mat_file_name must be a string.")
        
        self.__mat_file_name = file_name
    
    mat_file_name = property(lambda x: x.__mat_file_name, set_mat_file_name)
    
    #%%
    def set_mat_num_dig(self, num_dig):
        """ Set the number of digits used in the mat-file variables
        
        :parameters:
            num_dig: (int) : number of digits used in the mat-file variables
        """
        if type(num_dig) != int:
            raise TypeError("mat_num_dig must be a POSITIVE integer.")
        if num_dig <= 0:
            raise TypeError("mat_num_dig must be a POSITIVE integer.")
        
        self.__mat_num_dig = num_dig
    
    mat_num_dig = property(lambda x: x.__mat_num_dig, set_mat_num_dig)
    
    #%%
    def set_mat_var(self, mat_var):
        """ Set the list of variable names used in (to be created) mat-file
        
        :parameters:
            mat_var: (list of strings) : list of variable names used in (to be
                                         created) mat-file
        """
        if (not mat_var) and (self.__log_mat):
            raise TypeError("mat_var must be list of strings.")
        
        for k in range(len(mat_var)):
            if type(mat_var[k]) != str:
                raise TypeError("All elements of mat_var must be a string.")
        
        self.__mat_var = mat_var
    
    mat_var = property(lambda x: x.__mat_var, set_mat_var)
    
    #%%       
    def set_mat_cmds(self, mat_cmds):
        """
        
        """
        if (not mat_cmds) and (self.__log_mat):
            raise TypeError("mat_cmds must be list of strings")
        
        for k in range(len(mat_cmds)):
            if type(mat_cmds[k]) != str:
                raise TypeError("All elements of mat_cmds must be a string.")
        
        if (len(mat_cmds) != len(self.__mat_var)) and (self.__log_mat):
            raise TypeError("mat_cmds and mat_var must be lists of the same length.")
        
        self.__mat_cmds = mat_cmds
    
    mat_cmds = property(lambda x: x.__mat_cmds, set_mat_cmds)
    
    #%%
    def set_save_index(self, idx):
        """ Set index for mat-file-logging
        
        :parameters:
            idx: (integer) : Index for mat-file-logging
        """
        if (type(idx) != int) and (type(idx) != np.int64):
            raise TypeError("save_index must be a NON-NEGATIVE integer.")
        if idx < 0:
            raise TypeError("save_index must be a NON-NEGATIVE integer.")
        
        self.__save_index = int(idx)
    
    save_index = property(lambda x: x.__save_index, set_save_index)