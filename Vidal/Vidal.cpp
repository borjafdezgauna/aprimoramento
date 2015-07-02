// Boukhezzar.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "Vidal.h"

#ifndef __USE_OLD_BLADED_INTERFACE__
	#include "../External Controller/ExternalControllerApi.h"
	#pragma comment (lib,"../ExternalControllerApi.lib")
	using namespace GHTurbineInterface;
#endif


#define NINT(a) ((a) >= 0.0 ? (int)((a)+0.5) : (int)((a)-0.5))

extern "C"{

static double GenSpeedF;	//Filtered HSS (generator) speed, rad/s.
static double IntSpdErr; //Current integral of speed error w.r.t. time, rad.
static double LastGenTrq; //Commanded electrical generator torque the last time the controller was called, N-m.
static double LastTime; //Last time this DLL was called, sec.
static double LastTimePC; //Last time the pitch  controller was called, sec.
static double LastTimeVS; //Last time the torque controller was called, sec.
static double PitCom[3]; //Commanded pitch of each blade the last time the controller was called, rad.
static double LastGenSpeedF;




double sgn(double value)
{
	if (value<0.0) return -1.0;
	else if (value>0.0) return 1.0;

	return 0.0;
}

#ifdef __USE_OLD_BLADED_INTERFACE__
#define RATED_POWER 5296610.0
#define INITIAL_TORQUE 40000.0

void __declspec( dllexport ) __cdecl DISCON (float *avrSWAP //inout
												, int *aviFAIL //inout
												, char *accINFILE //in
												, char* avcOUTNAME //in
												, char *avcMSG) //inout
#else
	int __declspec( dllexport ) __cdecl CONTROLLER (const turbine turbine_id)
#endif
{
   //Local Variables:

	double BlPitch[3]; //Current values of the blade pitch angles, rad.
	double ElapTime; //Elapsed time since the last call to the controller, sec.
	double CornerFreq= 1.570796; //Corner frequency (-3dB point) in the recursive, single-pole, low-pass filter, rad/s. -- chosen to be 1/4 the blade edgewise natural frequency ( 1/4 of approx. 1Hz = 0.25Hz = 1.570796rad/s)
	double GenSpeed;		//Current  HSS (generator) speed, rad/s.
	double GenTrq;	//Electrical generator torque, N-m.

	double OnePlusEps= 1.0 + DBL_EPSILON; //The number slighty greater than unity in single precision.
	double PC_DT;  //Communication interval for pitch  controller, sec.
	double PC_KI= 0.008068634; //Integral gain for pitch controller at rated pitch (zero), (-).
	double PC_KK= 0.1099965; //Pitch angle where the the derivative of the aerodynamic power w.r.t. pitch has increased by a factor of two relative to the derivative at rated pitch (zero), rad.
	double PC_KP= 0.01882681; //Proportional gain for pitch controller at rated pitch (zero), sec.
	double PC_MaxPit= 1.570796; //Maximum pitch setting in pitch controller, rad.
	double PC_MaxRat= 0.1396263; //Maximum pitch  rate (in absolute value) in pitch  controller, rad/s.
	double PC_MinRat= -PC_MaxRat;
	double PC_MinPit= 0.0; //Minimum pitch setting in pitch controller, rad.
	double PC_RefSpd= 122.9096; //Desired (reference) HSS speed for pitch controller, rad/s.
	double PC_RefRotorSpd= 1.267109037;//Rated rotor speed for pitch controller, rad/s

	double PitRate[3]; //Pitch rates of each blade based on the current pitch angles and current pitch command, rad/s.
	double R2D= 57.295780; //Factor to convert radians to degrees.
	double RPS2RPM= 9.5492966; //Factor to convert radians per second to revolutions per minute.
	double SpdErr; //Current speed error, rad/s.
	double Time; //Current simulation time, sec.
	double VS_DT; //Communication interval for torque controller, sec.
	double VS_MaxRat= 15000.0; //Maximum torque rate (in absolute value) in torque controller, N-m/s.
	double VS_MaxTq= 47402.91; //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_MinTq= 0.0; //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double RotorSpeed;

	int iStatus; //A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
	int numBl; //Number of blades, (-).

#ifdef __USE_OLD_BLADED_INTERFACE__
   //Load variables from calling program (See Appendix A of Bladed User's Guide):
	iStatus= NINT (avrSWAP[1 -1]);
	NumBl= NINT(avrSWAP[61 -1]);
	BlPitch[1 -1] = (double) avrSWAP[4 -1];
	BlPitch[2 -1] = (double) avrSWAP[33 -1];
	BlPitch[3 -1] = (double) avrSWAP[34 -1];
	GenSpeed     =       avrSWAP[20 -1];
	HorWindV     =       avrSWAP[27 -1];
	Time         =       avrSWAP[2 -1];
	RotorSpeed= avrSWAP[21-1];
	VS_DT= 0.0125;
	PC_DT= 0.0125;
   //Initialize *aviFAIL to 0:


	*aviFAIL      = 0;
#else
	numBl= GetNumberOfBlades(turbine_id);
	if (GetPitchControl(turbine_id)==0) //collective pitch control?
	{
		BlPitch[0]= GetCollectivePitchAngle(turbine_id);
		BlPitch[1]= BlPitch[0];
		BlPitch[2]= BlPitch[0];
	}
	else
	{
		BlPitch[0]= GetMeasuredPitchAngle (turbine_id,0);
		BlPitch[1]= GetMeasuredPitchAngle (turbine_id,1);
		BlPitch[2]= GetMeasuredPitchAngle (turbine_id,2);
	}
	GenSpeed= GetMeasuredGeneratorSpeed(turbine_id);
	Time= GHTurbineInterface::GetCurrentTime(turbine_id);
	VS_DT= GetCommunicationInterval(turbine_id);
	PC_DT= GetCommunicationInterval(turbine_id);
	RotorSpeed= GetMeasuredRotorSpeed(turbine_id);
	
	//assuming same limits for all blades
	PC_MaxPit= GetMaximumPitchAngle(turbine_id,0);
	PC_MinPit= GetMinimumPitchAngle(turbine_id,0);
	PC_MaxRat= GetMaximumPitchRate(turbine_id,0);
	PC_MinRat= GetMinimumPitchRate(turbine_id,0);
	PC_RefSpd= GetReferenceGeneratorSpeedAboveRated(turbine_id);
	PC_RefRotorSpd= GetReferenceGeneratorTorqueAboveRated(turbine_id);
	VS_MaxRat= 15000; //max torque rate
	VS_MaxTq= 14743.3; //max torque
	VS_MinTq= 0; //min torque

	iStatus= (int) (Time!=0.0); //iStatus is 0 if Time==0.0, 1 otherwise
#endif
	if ( iStatus == 0 )  //.TRUE. if we're on the first call to the DLL
	{
		//Inform users that we are using this user-defined routine:
#ifdef __USE_OLD_BLADED_INTERFACE__
		*aviFAIL  = 1;
		//strcpy_s(ErrMsg,512,"Running with Vidal's torque and pitch controller. (C implementation by Borja Fernandez-Gauna)");
#endif
		GenSpeedF  = GenSpeed;                       //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		for (int i= 0; i<3; i++)
			PitCom[i]= BlPitch[i];   //This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call

		IntSpdErr  = 0.0;//PitCom[1 -1]/( GK*PC_KI );          //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero

		LastTime   = Time; //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		LastTimePC = Time - PC_DT; //This will ensure that the pitch  controller is called on the first pass 
		LastTimeVS = Time - VS_DT; //This will ensure that the torque controller is called on the first pass 

#ifdef __USE_OLD_BLADED_INTERFACE__
		LastGenTrq= INITIAL_TORQUE;
#else
		LastGenTrq= GetMeasuredGeneratorTorque(turbine_id);
#endif

		LastGenSpeedF= GenSpeedF;
	}
	if ( iStatus >= 0 /* &&  *aviFAIL >= 0 */ /* NO ERROR CHECKING SO THIS MAKES NO SENSE JUST YET*/ )
	{
/*
	   //Abort if the user has not requested a pitch angle actuator (See Appendix A
	   //  of Bladed User's Guide):
	   if ( (int) (avrSWAP[10 -1]) != 0 )
	   {
		   //.TRUE. if a pitch angle actuator hasn't been requested
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"Pitch angle actuator not requested.");
	   }*/

	   //Set unused outputs to zero (See Appendix A of Bladed User's Guide):
#ifdef __USE_OLD_BLADED_INTERFACE__
	   avrSWAP[36 -1] = 0.0; //Shaft brake status: 0=off
	   avrSWAP[41 -1] = 0.0; //Demanded yaw actuator torque
	   avrSWAP[46 -1] = 0.0; //Demanded pitch rate (Collective pitch)
	   avrSWAP[48 -1] = 0.0; //Demanded nacelle yaw rate
	   avrSWAP[65 -1] = 0.0; //Number of variables returned for logging
	   avrSWAP[72 -1] = 0.0; //Generator start-up resistance
	   avrSWAP[79 -1] = 0.0; //Request for loads: 0=none
	   avrSWAP[80 -1] = 0.0; //Variable slip current status
	   avrSWAP[81 -1] = 0.0; //Variable slip current demand
#endif
	   //Filter the HSS (generator) speed measurement:
	   //NOTE: This is a very simple recursive, single-pole, low-pass filter with
	   //      exponential smoothing.



	   //Update the coefficient in the recursive formula based on the elapsed time
	   //  since the last call to the controller:
	   //CURRENTLY DISABLED. TO ENABLE, UNCOMMENT NEXT LINE
	   //Alpha     = exp( ( LastTime - Time )*CornerFreq );

	   //Apply the filter: CURRENTLY DISABLED. TO ENABLE, UNCOMMENT NEXT LINE
	   GenSpeedF = GenSpeed;
			//( 1.0 - Alpha )*GenSpeed + Alpha*GenSpeedF;

	   double ratedPower, power, powerError;
#ifdef __USE_OLD_BLADED_INTERFACE__
		power= avrSWAP[14 -1];
		ratedPower= RATED_POWER;
		
#else 
		power= GetMeasuredElectricalPowerOutput(turbine_id);
		ratedPower= GetDemandedPower(turbine_id);
#endif
		powerError= power - ratedPower;

		//Variable-speed torque control:
		ElapTime = Time - LastTimeVS;

		double d_T_g;
		double d_omega_g;

///IT MIGHT BE WORTH CHANGING THESE PARAMETERS IF IT DOESN'T WORK PROPERLY
		double A= 1.0;
		double K_ALPHA= ratedPower*.01;

	   if ( (ElapTime+0.001) >= VS_DT )
	   {
		   //d(Tg)/dt= (-1/omega_r)*(T_g*(a*omega_r+d_omega_r)-a*P_setpoint + K_alpha*sgn(P_e-P_setpoint))

			d_omega_g= (GenSpeedF-LastGenSpeedF)/ElapTime;

			d_T_g= (-1.0/GenSpeedF)*(LastGenTrq*(A*GenSpeedF+d_omega_g)
				-A*ratedPower + K_ALPHA*sgn(powerError));

			d_T_g= min(max(d_T_g,-VS_MaxRat),VS_MaxRat);
				
			GenTrq= LastGenTrq + d_T_g*ElapTime;

			//Saturate the commanded torque using the maximum torque limit:

			GenTrq  = max(VS_MinTq,min( GenTrq, VS_MaxTq ));   //Saturate the command using the maximum torque limit


			//Reset the values of LastTimeVS and LastGenTrq to the current values:

			LastTimeVS = Time;
			LastGenTrq = GenTrq;
			LastGenSpeedF= GenSpeedF;
		}


	   //Set the generator contactor status, avrSWAP(35), to main (high speed) 
	   //  variable-speed generator, the torque override to yes, and command the
	   //  generator torque (See Appendix A of Bladed User's Guide):
#ifdef __USE_OLD_BLADED_INTERFACE__
	   avrSWAP[35 -1] = 1.0;          //Generator contactor status: 1=main (high speed) variable-speed generator
	   avrSWAP[56 -1] = 0.0;          //Torque override: 0=yes
	   avrSWAP[47 -1] = (float) LastGenTrq;   //Demanded generator torque
#else
		SetGeneratorContactor(turbine_id, 1); //Generator contactor status: 1= main
		SetTorqueOverrideStatus(turbine_id, 0); //Torque override: 0=on
		SetDemandedGeneratorTorque(turbine_id, LastGenTrq);
#endif




	   //Pitch control:

	   //Compute the elapsed time since the last call to the controller:

	   ElapTime = Time - LastTimePC;


	   //Only perform the control calculations if the elapsed time is greater than
	   //  or equal to the communication interval of the pitch controller:
	   //NOTE: Time is scaled by OnePlusEps to ensure that the controller is called
	   //      at every time step when PC_DT = DT, even in the presence of
	   //      numerical precision errors.

	   if ( (ElapTime+0.001) >= PC_DT )
	   {
			//Compute the current speed error and its integral w.r.t. time; saturate the
			//  integral term using the pitch angle limits:

			SpdErr    = RotorSpeed - PC_RefRotorSpd;                                 //Current speed error
			IntSpdErr = IntSpdErr + SpdErr*ElapTime;                           //Current integral of speed error w.r.t. time

		  
			//d(beta)= 0.5*K_p*error_omega*(1.0+sgn(error_omega))
			//	+ K_i*state->getContinuousState(DIM_integrative_omega_r_error);*/
			double KI= 0.0;
			double KP= 0.1;

			//Saturate the integral term using the pitch angle limits, converted to integral speed error limits
			if (KI!=0.0)
				IntSpdErr = min( max( IntSpdErr, PC_MinPit/KI ), PC_MaxPit/KI);


			double dBeta= //0.5*KP*SpdErr*(max(min(0.5,SpdErr*10.0),-0.5)+1.0); //in the simulink vidal sent me (?)
				KP*SpdErr + KI*IntSpdErr;						//typical PI
				//0.5*KP*SpdErr*(1.0+sgn(SpdErr));//+KI*IntSpdErr; // in the original paper
			dBeta= min( max( dBeta, PC_MinRat ), PC_MaxRat );
			//Saturate the overall commanded pitch using the pitch rate limit:
			//NOTE: Since the current pitch angle may be different for each blade
			//      (depending on the type of actuator implemented in the structural
			//      dynamics model), this pitch rate limit calculation and the
			//      resulting overall pitch angle command may be different for each
			//      blade.

			for (int k=0; k<numBl; k++) //Loop through all blades
			{
				PitCom[k]= BlPitch[k] + dBeta*ElapTime;
				PitCom[k]= min(max(PitCom[k],PC_MinPit),PC_MaxPit);
				PitRate[k] = ( PitCom[k] - BlPitch[k ] )/ElapTime; //Pitch rate of blade K (unsaturated)
				PitRate[k] = min( max( PitRate[k], -PC_MaxRat ), PC_MaxRat ); //Saturate the pitch rate of blade K using its maximum absolute value
				PitCom[k] = BlPitch[k] + PitRate[k]*ElapTime; //Saturate the overall command of blade K using the pitch rate limit

				PitCom[k]  = min( max( PitCom[k ], PC_MinPit ), PC_MaxPit ); //Saturate the overall command using the pitch angle limits         
			}

			LastTimePC = Time;
	   }   
	      
	      
	   //Set the pitch override to yes and command the pitch demanded from the last
	   //  call to the controller (See Appendix A of Bladed User's Guide):
#ifdef __USE_OLD_BLADED_INTERFACE__
	   avrSWAP[55 -1] = 0.0;       //Pitch override: 0=yes

	   avrSWAP[42 -1] = (float)PitCom[1 -1]; //Use the command angles of all blades if using individual pitch
	   avrSWAP[43 -1] = (float)PitCom[2 -1]; //"
	   avrSWAP[44 -1] = (float)PitCom[3 -1]; //"

	   avrSWAP[45 -1] = (float)PitCom[1 -1]; //Use the command angle of blade 1 if using collective pitch
#else
		if (GetPitchControl(turbine_id)==0) //collective pitch control?
			SetDemandedCollectivePitchRate(turbine_id,PitRate[0]);
		else
		{
			for (int i= 0; i<numBl; i++)
				SetDemandedPitchAngle(turbine_id,i,PitRate[i]);
		}
	   SetPitchOverrideStatus(turbine_id,0); //pitch override: 0= on
#endif

	   //Reset the value of LastTime to the current value:
	   LastTime = Time;
	}

	//strcpy_s(avcMSG , msgLen-1,ErrMsg);//C_NULL_CHAR, avcMSG )
#ifdef __USE_OLD_BLADED_INTERFACE__
	return GH_DISCON_SUCCESS;
#else
	return 0;
#endif
}

}