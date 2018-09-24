"""
Param_user_defined_control class definition
Parameters for User Defined Control utility
@author: neureuther
"""

#################################################
# P-Class (parametres) User Defined Control
#################################################
class Param_ud_control:
    
    #%%
    #################################################
    # DEFINITION
    #################################################
    def __init__(self):
        """
        Parameters to configure the use of user defined control-laws
        use_udc         if true: ud control-law is used (bool)
        udc_file_dir    directory of the script containing the ud controller (string)
        udc_file_name   file-name of the script containing the ud controller (string)
        udc_session     Session used to execute the ud controller (string)
                        "python" = Python
                        "new matlab" = new MATLAB session
                        "find matlab" = find shared MATLAB session
                        arbitrary = connect to shared MATLAB with specified name
        mat_output      Output of the MATLAB function implementing the ud 
                        controller (io.StringIO)
        cmd_apply       if true: mirror cmds calculated by ud controller are 
                        applied to the COMPASS simulation (bool)
        param           Dictionary containing all relevant "simulation configs"
                        for MATLAB (dictionary)
        engine          MATLAB engine used to interact with MATLAB (MATLAB 
                        engine object)
        udc_mode        Loaded python module containing the user definded 
                        controller function (module)
        func_param      COMPASS commands providing the inputs for the MATLAB
                        ud controller (list of strings)
        func_call       Python call of the ud controller (string)
        buffer          Object member intended to store variables between ud
                        controller runs (specified by user)
        """
        self.__use_udc = False
        self.__udc_file_dir = "default"
        self.__udc_file_name = "contoller.m"
        self.__udc_session  = None # 0
        self.mat_output = None
        self.__cmd_apply = True
        self.__param = None
        self.engine = None
        self.udc_mod = None
        self.__func_param = []
        self.__func_call = None
        self.buffer = None
        
        
    #%%
    #################################################
    # SET-COMMANDS
    #################################################
    def set_use_udc(self, use_bool):
        """ Set if user defined control-law is used
        
        :parameters:
            use_bool: (bool) : bool determining if ud control-law is used
        """
        if type(use_bool) != bool:
            raise TypeError("use_udc must be a boolean (True or False)")
        
        self.__use_udc = use_bool
        
    use_udc = property(lambda x: x.__use_udc, set_use_udc)
    
    #%%
    def set_udc_file_dir(self, file_dir):
        """ Set the directory of the script containing the ud controller
        
        :parameters:
            file_dir: (string) : directory of script containing ud controller
        """
        if type(file_dir) != str:
            raise TypeError("udc_file_dir must be a string.")
        
        self.__udc_file_dir = file_dir
    
    udc_file_dir = property(lambda x: x.__udc_file_dir, set_udc_file_dir)
    
    #%%
    def set_udc_file_name(self, file_name):
        """ Set the file-name of the script containing the ud controller
        
        :parameters:
            file_name: (string) : file-name of script containing ud controller
        """
        if type(file_name) != str:
            raise TypeError("udc_file_name must be a string.")
        
        self.__udc_file_name = file_name
    
    udc_file_name = property(lambda x: x.__udc_file_name, set_udc_file_name)
    
    #%%
    def set_udc_session (self, session):
        """ Set the session used to execute the ud controller ("python" = Python,
        "new matlab" = new MATLAB session, arbitrary = connect to shared MATLAB
        with specified name)
        
        :parameters:
            session: (str) : session used to execute the ud controller
        """
        if type(session) != str:
            raise TypeError("udc_session must be an string.")
        
        self.__udc_session = session
    
    udc_session = property(lambda x: x.__udc_session, set_udc_session)
    
    #%%
    def set_cmd_apply(self, ap_bool):
        """ Set if mirror cmds calculated by ud controller are applied to the
        COMPASS simulation
        
        :parameters:
            ap_bool: (bool) : bool determining if mirror cmds calculated by ud
                              controller are applied
        """
        if type(ap_bool) != bool:
            raise TypeError("cmd_apply must be a boolean (True or False).")
        
        self.__cmd_apply = ap_bool
        
    cmd_apply = property(lambda x: x.__cmd_apply, set_cmd_apply)
    
    #%%
    def set_param(self, dictionary):
        """ Set the dictionary containing all relevant "simulation configs"
        for MATLAB
        
        :parameters:
            dictionary: (dict) : containing all relevant "simulation configs"
        """
        if type(dictionary) != dict:
            raise TypeError("param must be a python dictionary.")
        
        if not all(type(x) == str for x in list(dictionary.keys())):
            raise TypeError("All keys in the param-dictionary must be strings.")
        
        self.__param = dictionary
    
    param = property(lambda x: x.__param, set_param)
    
    #%%
    def set_func_param(self, content):
        """ Set the list of COMPASS commands providing the inputs for the MATLAB
        ud controller
        
        :parameters:
            content: (list of strings) : list of COMPASS commands providing the
                                        inputs for the MATLAB ud controller
        """
        if not all(type(x) == str for x in content):
            raise TypeError("All items of func_param must be strings.")
        
        self.__func_param = content
    
    func_param = property(lambda x: x.__func_param, set_func_param)
    
    #%%
    def set_func_call(self, content):
        """ Set python-call-string of the ud controller
        
        :parameters:
            content: (string) : python-call-string of the ud controller
        """
        if type(content) != str:
            raise TypeError("func_call must be a string.")
        
        self.__func_call = content
    
    func_call = property(lambda x: x.__func_call, set_func_call)
    