//SetupValidation.h
//

#define	MOTOROS_SETUPERROR_ALARMCODE	8003

typedef enum
{
	MOTOROS_SETUP_OK = 0,	//All is good
	
	MOTOROS_SETUP_RS0,		//Set RS000 = 2

	MOTOROS_SETUP_S2C541,	//Set S2C541 = 0
	MOTOROS_SETUP_S2C542,	//Set S2C542 = 0
	MOTOROS_SETUP_S2C1100,	//Set S2C1100 = 1
	//MOTOROS_SETUP_S2C1101,	//Set S2C1101 = 1 (recommended, but optional)
	MOTOROS_SETUP_S2C1103,	//Set S2C1103 = 2
	MOTOROS_SETUP_S2C1117,	//Set S2C1117 = 1
	MOTOROS_SETUP_S2C1119,	//Set S2C1119 = 0 (optionally can be =2 to allow telnet connections)

	MOTOROS_SETUP_NotCompatibleWithPFL,	//Uninstall the PFL driver for the HC-10

	//For all other error codes, please contact Yaskawa Motoman
	//to have the MotoROS Runtime functionality enabled on
	//your robot controller.
	MOTOROS_SETUP_ContactYaskawaMotoman_1 = 100,
	MOTOROS_SETUP_ContactYaskawaMotoman_2,
	MOTOROS_SETUP_ContactYaskawaMotoman_3,
	MOTOROS_SETUP_ContactYaskawaMotoman_4,
	MOTOROS_SETUP_ContactYaskawaMotoman_5,
	MOTOROS_SETUP_ContactYaskawaMotoman_6,
	MOTOROS_SETUP_ContactYaskawaMotoman_7,
	MOTOROS_SETUP_ContactYaskawaMotoman_8,
	MOTOROS_SETUP_ContactYaskawaMotoman_9,
	MOTOROS_SETUP_ContactYaskawaMotoman_10,
	MOTOROS_SETUP_ContactYaskawaMotoman_11,
	MOTOROS_SETUP_ContactYaskawaMotoman_12
} MOTOROS_SETUP_CODES;

// Verify most of the setup parameters of the robot controller.
// Please note that some parameters cannot be checked, such as
// the parameter(s) which enable this task to run.
extern MOTOROS_SETUP_CODES ValidateMotoRosSetupParameters();

