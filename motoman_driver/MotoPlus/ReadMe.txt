There are various possible compilation for this project.

MpRosFs100
----------
The version is good for all FS100 software version.
It is the default compile option so make sure the neither "DX100" or "DX100FTP" keyword are defined in the Controller.h file.

MpRosDx100
----------
Is for controller version DS1.63.XX()-03 which is normally the recommended version.
It has file access API (uses ParameterExtraction_DX100.mpLib).
Note that the expanded memory option board is required to use these API.
Make sure to define the keyword "DX100" to compile this version. (Normally defined in the Controller.h file)
  
MpRosDx100Ftp
-------------
Is for other controller version that support MotoPlus (-03 or -14) which maybe required because of the need of other functions not supported by the version DS1.63.XX()-03.
It doesn't have the file access API and uses the FTP function to access the parameters(uses ParameterExtraction_DX100_FTP.mpLib).
Make sure to define the keyword "DX100" to compile this version. (Normally defined in the Controller.h file)
 
