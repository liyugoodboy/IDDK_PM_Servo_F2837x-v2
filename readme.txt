
Major changes to this project from the earlier release (v1.0) are:
	- Addition of EnDAT absolute encoder interface library and code
        - Addition of BiSS-C encoder interfce library and code
        - Bug fix in over current protection algorithm
          - new variable 'curLimit' added, which the user can set up at actuals
           set to 10.0 if protection needed at 10A
        - Default code set for COLD control GND configuration
           (SHUNT currents and voltages can be sensed in HOT config only)
        - device support library changed from v150 to v170
          - TrigRegs changed to SyncSocRegs        
