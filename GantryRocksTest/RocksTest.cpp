// GantryRocksTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <nyceapi.h>
#include <sacapi.h>
#include <nhiapi.h>
#include <rocksapi.h>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

#define NUM_NODE 1
const char *noName[ NUM_NODE ] = { "NY4112_node"};
NHI_NODE noId[ NUM_NODE ];
BOOL noCon[ NUM_NODE ];

#define NUM_AXES 4
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3", "DEF_AXIS_4"};
SAC_AXIS axId[ NUM_AXES ];
BOOL axCon[ NUM_AXES ];

NYCE_STATUS nyceStatus;
BOOL isErrAtCallback = FALSE;

uint32_t varIndexs[NUM_AXES * 2];
double vars[NUM_AXES * 2];

void HandleError(const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(nyceStatus)<<"\n"<<endl;
}

void TermAxis(void)
{
	int ax;
	SAC_STATE sacState = SAC_NO_STATE;
	SAC_SPG_STATE sacSpgState;
	nyceStatus = NYCE_OK;
	for (ax = 0; ax < NUM_AXES; ax++ )
	{
		if (axCon[ax] != TRUE) continue;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadState(axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus) && sacState == SAC_MOVING)
		{
			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacQuickStop(axId[ ax ]);
			if (NyceSuccess(nyceStatus))
			{
				nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 10 );
				if (NyceError(nyceStatus))
				{
					HandleError(axName[ ax ]);
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_RESET, 10 );
				}
			}
		}
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacShutdown(axId[ ax ]);
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDisconnect(axId[ ax ]);
		if(NyceError(nyceStatus)) HandleError(axName[ ax ]);
		else axCon[ax] = FALSE;
	}
}

BOOL InitAxis(void)
{
	int ax;
	SAC_SPG_STATE sacSpgState;
	SAC_STATE sacState;
	SAC_CONFIGURE_AXIS_PARS axisPars;
	nyceStatus = NYCE_OK;
	for (ax = 0; ax < NUM_AXES; ax++ )
	{
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacConnect( axName[ ax ], &axId[ ax ] );
		if ( NyceSuccess(nyceStatus)) axCon[ax] = TRUE;

		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadState( axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus))
		{
			switch (sacState)
			{
			case SAC_IDLE:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacInitialize( axId[ ax ], SAC_USE_FLASH );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_INITIALIZE, 10 );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacGetAxisConfiguration( axId[ ax ], &axisPars );

				if ( NyceSuccess(nyceStatus) && axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
				{
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacAlignMotor( axId[ ax ] );
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_ALIGN_MOTOR, 10 );
				}
			case SAC_INACTIVE:
			case SAC_FREE:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacLock( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_LOCK, 10 );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );
				break;
			case SAC_MOVING:
				printf("Waiting the motion stop...");
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 30 );
			default:
				break;
			}
		}
		
		if(NyceError(nyceStatus))
		{
			HandleError(axName[ ax ]);
			TermAxis();
			return FALSE;
		}
	}

	return TRUE;
}

void TermNode(void)
{
	int no;
	nyceStatus = NYCE_OK;
	for(no = 0; no < NUM_NODE; ++no)
	{
		if (noCon[no] != TRUE) continue;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiDisconnect(noId[no]);
		if(NyceError(nyceStatus)) HandleError(noName[no]);
		else noCon[no] = FALSE;
	}
}

BOOL InitNode(void)
{
	int no;
	nyceStatus = NYCE_OK;
	for (no = 0; no < NUM_NODE; ++no)
	{
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiConnect(noName[no],&noId[no]);
		if (NyceSuccess(nyceStatus) ) noCon[no] = TRUE;
		if (NyceError(nyceStatus) )
		{
			HandleError(noName[no]);
			TermNode();
			return FALSE;
		}
	}
	return TRUE;
}

int _tmain(int argc, _TCHAR* argv[])
{
	nyceStatus = NYCE_OK;
	ROCKS_MECH  m_mech;
	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccPars;
	ROCKS_KIN_INV_PARS kinPars;

	printf("Begin...\n");
	nyceStatus = NyceInit(NYCE_SIM);
	if (NyceError(nyceStatus))
	{
		HandleError("System");
		goto end;
	}
	if (!InitAxis()) goto end;

#ifdef NT
	Sleep(500);//Some trouble will be happened without this code while using the simulation system.
			   //--For example, the error "spline buffer empty" occur.
#endif // NT

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = NUM_AXES; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( int ax = 0; ax < NUM_AXES; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );
//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDefineGantry( &m_mech, ROCKS_GANTRY_X );

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = 100.0;
	sineAccPars.maxAcceleration = 1000.0;
	sineAccPars.splineTime = 0.01;
	sineAccPars.center[ 0 ] = 300.0;
	sineAccPars.center[ 1 ] = 0.0;
	sineAccPars.angle = M_PI * 4;
	sineAccPars.plane = ROCKS_PLANE_XY;
	sineAccPars.maxNrOfSplines =  0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	// Get current position
	// --------------------
//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinGantryPosition( &m_mech, sineAccPars.startPos );
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinCartesianPosition( &m_mech, sineAccPars.startPos );

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

//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseGantry( &m_mech, &kinPars );
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

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
		printf(NyceGetStatusString(nyceStatus));
		printf("\n");
	}

end:
	TermAxis();
	TermNode();
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();
	if (NyceError(nyceStatus))
	{
		HandleError("Host");
	}
	printf("End.\n");

	system("pause");
	return 0;
}