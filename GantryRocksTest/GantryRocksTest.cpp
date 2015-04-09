// GantryRocksTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <nyceapi.h>
#include <rocksapi.h>
#include <Windows.h>

#define NUM_AXES 4

const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3", "DEF_AXIS_4" };

int _tmain(int argc, _TCHAR* argv[])
{
	NYCE_STATUS nyceStatus = NYCE_OK;
	ROCKS_MECH  m_mech;
	SAC_AXIS    m_sacAxis[ NUM_AXES ];
	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccPars;
	ROCKS_KIN_INV_PARS kinPars;

	if(NyceError(NyceInit( NYCE_SIM)))
	{
		printf("NyceInit error! \n");
		printf( NyceGetStatusString(nyceStatus));
		printf("\n");
		system("pause");
		return 0;
	}

	// Connect to all axes
	// -------------------
	for ( int ax = 0; ax < NUM_AXES; ax++ )
	{
			nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacConnect( axName[ ax ], &m_sacAxis[ ax ] );
			nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacInitialize( m_sacAxis[ ax ], SAC_USE_FLASH );
			nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacSynchronize( m_sacAxis[ ax ], SAC_REQ_INITIALIZE, 10 );
	}
	if (NyceError( nyceStatus ))
	{
		printf("Connect error! \n");
		printf( NyceGetStatusString(nyceStatus));
		printf("\n");
		goto term;
	}

	// Bring all axes to the ready state and home all axes
	// ---------------------------------------------------
	SAC_CONFIGURE_AXIS_PARS axisPars;
	for (int ax = 0; ax < NUM_AXES; ax++)
	{
		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacGetAxisConfiguration( m_sacAxis[ ax ], &axisPars );

		if ( axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
		{
			nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacAlignMotor( m_sacAxis[ ax ] ) ;
			nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacSynchronize( m_sacAxis[ ax ], SAC_REQ_ALIGN_MOTOR, 10 ) ;
		}

		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacLock( m_sacAxis[ ax ] );
		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacSynchronize( m_sacAxis[ ax ], SAC_REQ_LOCK, 10 ) ;

		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacHome( m_sacAxis[ ax ] );
		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacSynchronize( m_sacAxis[ ax ], SAC_REQ_HOMING_COMPLETED, 10 ) ;

	}
	if (NyceError( nyceStatus ))
	{
		printf("Prepare error! \n");
		goto stop;
	}

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = NUM_AXES;      // X1, X2, Y and Z
	m_mech.dof[ 0 ] = TRUE;     // X
	m_mech.dof[ 1 ] = TRUE;     // Y
	m_mech.dof[ 2 ] = TRUE;     // Z
	m_mech.dof[ 3 ] = FALSE;    // Rx
	m_mech.dof[ 4 ] = FALSE;    // Ry
	m_mech.dof[ 5 ] = FALSE;    // Rz
	for ( int ax = 0; ax < NUM_AXES; ax++ )
	{
		m_mech.jointAxisId[ ax ] = m_sacAxis[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDefineGantry( &m_mech, ROCKS_GANTRY_X );

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = 500.0;
	sineAccPars.maxAcceleration = 5000.0;
	sineAccPars.splineTime = 0.01;
	sineAccPars.center[ 0 ] = 300.0;
	sineAccPars.center[ 1 ] = 0.0;
	sineAccPars.angle = 360.0;
	sineAccPars.plane = ROCKS_PLANE_XY;
	sineAccPars.maxNrOfSplines =  0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	// Get current position
	// --------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinGantryPosition( &m_mech, sineAccPars.startPos );

	// Get path splines
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccPars );

	// Apply inverse kinematics to get joint splines
	// ---------------------------------------------
	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseGantry( &m_mech, &kinPars );

	// Feed splines to the joints
	// --------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );

	// Synchronize on motion complete
	// ------------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE);

	// Delete mechanism
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechDelete( &m_mech );
	if (NyceError( nyceStatus ))
	{
		printf("Rocks error! \n");
		printf( NyceGetStatusString(nyceStatus));
		printf("\n");
	}

stop:	
	nyceStatus = NYCE_OK;
	for (int ax = 0; ax <  NUM_AXES; ++ax)
	{
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : SacShutdown(m_sacAxis[ax]);
		nyceStatus =  NyceError( nyceStatus ) ? nyceStatus : SacSynchronize( m_sacAxis[ ax ], SAC_REQ_SHUTDOWN, 10 ) ;
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : SacDisconnect(m_sacAxis[ax]);
	}
	if (NyceError( nyceStatus ))
	{
		printf("SacStop error! \n");
		printf( NyceGetStatusString(nyceStatus));
		printf("\n");
	}

term:
	if (NyceError( NyceTerm() ))
	{
		printf("Termination error! \n");
		printf( NyceGetStatusString(nyceStatus));
		printf("\n");
	}

	system("pause");
	return 0;
}