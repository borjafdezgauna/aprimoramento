// Jonkman.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "Jonkman.h"

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
static double VS_Slope15;//Torque/speed slope of region 1 1/2 cut-in torque ramp , N-m/(rad/s).
static double VS_Slope25;//Torque/speed slope of region 2 1/2 induction generator, N-m/(rad/s).
static double VS_SySp; //Synchronous speed of region 2 1/2 induction generator, rad/s.
static double VS_TrGnSp; //Transitional generator speed (HSS side) between regions 2 and 2 1/2, rad/s.


#ifdef __USE_OLD_BLADED_INTERFACE__
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

	double Alpha; //Current coefficient in the recursive, single-pole, low-pass filter, (-).
	double BlPitch[3]; //Current values of the blade pitch angles, rad.
	double ElapTime; //Elapsed time since the last call to the controller, sec.
	double CornerFreq= 1.570796; //Corner frequency (-3dB point) in the recursive, single-pole, low-pass filter, rad/s. -- chosen to be 1/4 the blade edgewise natural frequency ( 1/4 of approx. 1Hz = 0.25Hz = 1.570796rad/s)
	double GenSpeed;		//Current  HSS (generator) speed, rad/s.

	double GenTrq;	//Electrical generator torque, N-m.
	double GK;	//Current value of the gain correction factor, used in the gain scheduling law of the
				//pitch controller, (-).

	double PC_DT;  //Communication interval for pitch  controller, sec.
	double PC_KI= 0.008068634; //Integral gain for pitch controller at rated pitch (zero), (-).
	double PC_KK= 0.1099965; //Pitch angle where the the derivative of the aerodynamic power w.r.t. pitch has increased by a factor of two relative to the derivative at rated pitch (zero), rad.
	double PC_KP= 0.01882681; //Proportional gain for pitch controller at rated pitch (zero), sec.
	double PC_MaxPit= 1.570796; //Maximum pitch setting in pitch controller, rad.
	double PC_MaxRat= 0.1396263; //Maximum pitch  rate (in absolute value) in pitch  controller, rad/s.
	double PC_MinRat= -PC_MaxRat;
	double PC_MinPit= 0.0; //Minimum pitch setting in pitch controller, rad.
	double PC_RefSpd= 122.9096; //Desired (reference) HSS speed for pitch controller, rad/s.

	double PitComI; //Integral term of command pitch, rad.
	double PitComP; //Proportional term of command pitch, rad.
	double PitComT; //Total command pitch based on the sum of the proportional and integral terms, rad.
	double PitRate[3]; //Pitch rates of each blade based on the current pitch angles and current pitch command, rad/s.
	double R2D= 57.295780; //Factor to convert radians to degrees.
	double RPS2RPM= 9.5492966; //Factor to convert radians per second to revolutions per minute.
	double SpdErr; //Current speed error, rad/s.
	double Time; //Current simulation time, sec.
	double TrqRate; //Torque rate based on the current and last torque commands, N-m/s.
	double VS_CtInSp= 70.16224; //Transitional generator speed (HSS side) between regions 1 and 1 1/2, rad/s.
	double VS_DT; //Communication interval for torque controller, sec.
	double VS_MaxRat= 15000.0; //Maximum torque rate (in absolute value) in torque controller, N-m/s.
	double VS_MaxTq= 47402.91; //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_MinTq= 0.0; //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_Rgn2K= 0.24; //Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2.
	double VS_Rgn2Sp= 115.71521; //Transitional generator speed (HSS side) between regions 1 1/2 and 2, rad/s.
	double VS_Rgn3MP= -0.017453; //Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, rad. -- chosen to be 1.0 degree above PC_MinPit
	double VS_RtGnSp= 157.08; //Rated generator speed (HSS side), rad/s. -- chosen to be 99% of PC_RefSpd
	double VS_RtPwr= 5296610.0; //Rated generator generator power in Region 3, Watts. -- chosen to be 5MW divided by the electrical generator efficiency of 94.4%
	double VS_SlPc= 10.0;//Rated generator slip percentage in Region 2 1/2, %.

	int numBl;
	int iStatus; //A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.


#ifdef __USE_OLD_BLADED_INTERFACE__
   //Load variables from calling program (See Appendix A of Bladed User's Guide):
	iStatus= NINT (avrSWAP[1 -1]);
	NumBl= NINT(avrSWAP[61 -1]);
	BlPitch[1 -1] = (double) avrSWAP[4 -1];
	BlPitch[2 -1] = (double) avrSWAP[33 -1];
	BlPitch[3 -1] = (double) avrSWAP[34 -1];
	GenSpeed     =       avrSWAP[20 -1];
	Time         =       avrSWAP[2 -1];
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

	//assuming same limits for all blades
	PC_MaxPit= GetMaximumPitchAngle(turbine_id,0);
	PC_MinPit= GetMinimumPitchAngle(turbine_id,0);
	PC_MaxRat= GetMaximumPitchRate(turbine_id,0);
	PC_MinRat= GetMinimumPitchRate(turbine_id,0);
	PC_RefSpd= GetReferenceGeneratorSpeedAboveRated(turbine_id);
	VS_MaxRat= 15000; //max torque rate
	VS_MaxTq= 14743.3; //max torque
	VS_MinTq= 0; //min torque

	iStatus= (int) (Time!=0.0); //iStatus is 0 if Time==0.0, 1 otherwise
#endif


   //Read any External Controller Parameters specified in the User Interface
   //  and initialize variables:

	if ( iStatus == 0 )  //.TRUE. if we're on the first call to the DLL
	{
		//Inform users that we are using this user-defined routine:
#ifdef __USE_OLD_BLADED_INTERFACE__
		*aviFAIL  = 1;
		//strcpy_s(ErrMsg,512,"Running with torque and pitch control of the NREL offshore 5MW baseline wind turbine from DISCON.dll as written by J. Jonkman of NREL/NWTC for use in the IEA Annex XXIII OC3 studies. (reimplemented in C by Borja Fernandez-Gauna)");
#else
		VS_RtPwr= GetDemandedPower(turbine_id);
#endif
		//Determine some torque control parameters not specified directly:

	   VS_SySp    = VS_RtGnSp/( 1.0 +  0.01*VS_SlPc );
	   VS_Slope15 = ( VS_Rgn2K*VS_Rgn2Sp*VS_Rgn2Sp )/( VS_Rgn2Sp - VS_CtInSp );
	   VS_Slope25 = ( VS_RtPwr/VS_RtGnSp           )/( VS_RtGnSp - VS_SySp   );
	   if ( VS_Rgn2K == 0.0 )  //.TRUE. if the Region 2 torque is flat, and thus, the denominator in the ELSE condition is zero
		  VS_TrGnSp = VS_SySp;
	   else                          //.TRUE. if the Region 2 torque is quadratic with speed
		  VS_TrGnSp = ( VS_Slope25 - sqrt( VS_Slope25*( VS_Slope25 - 4.0*VS_Rgn2K*VS_SySp ) ) )/( 2.0*VS_Rgn2K );
	   

/* NO ERROR CHECKING FOR NOW!!!!!!!!!!!!
	   //Check validity of input parameters:

		if ( CornerFreq <= 0.0 )
		{ 
			ReportInfoMessage(turbine_id, "Set pitch");
			return GH_DISCON_ERROR;

			*aviFAIL = -1;
			strcpy_s(ErrMsg, 512, "CornerFreq must be greater than zero.");
		}

		if ( VS_DT     <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_DT must be greater than zero.");
		}

		if ( VS_CtInSp <  0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_CtInSp must not be negative.");
		}

		if ( VS_Rgn2Sp <= VS_CtInSp )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_Rgn2Sp must be greater than VS_CtInSp.");
		}

		if ( VS_TrGnSp <  VS_Rgn2Sp )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_TrGnSp must not be less than VS_Rgn2Sp.");
		}

		if ( VS_SlPc   <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_SlPc must be greater than zero.");
		}

		if ( VS_MaxRat <= 0.0 )
		{
			*aviFAIL =  -1;
			strcpy_s(ErrMsg,512,"VS_MaxRat must be greater than zero.");
		}

		if ( VS_RtPwr  <  0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_RtPwr must not be negative.");
		}

		if ( VS_Rgn2K  <  0.0 ) 
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_Rgn2K must not be negative.");
		}

		if ( VS_Rgn2K*VS_RtGnSp*VS_RtGnSp > VS_RtPwr/VS_RtGnSp )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_Rgn2K*VS_RtGnSp^2 must not be greater than VS_RtPwr/VS_RtGnSp.");
		}

		if ( VS_MaxTq < VS_RtPwr/VS_RtGnSp )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"VS_RtPwr/VS_RtGnSp must not be greater than VS_MaxTq.");
		}

		if ( PC_DT <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_DT must be greater than zero.");
		}

		if ( PC_KI <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_KI must be greater than zero.");
		}

		if ( PC_KK <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_KK must be greater than zero.");
		}

		if ( PC_RefSpd <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_RefSpd must be greater than zero.");
		}

		if ( PC_MaxRat <= 0.0 )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_MaxRat must be greater than zero.");
		}

		if ( PC_MinPit >= PC_MaxPit )
		{
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"PC_MinPit must be less than PC_MaxPit.");
		}
*/


		//Initialize the SAVEd variables:
		//NOTE: LastGenTrq, though SAVEd, is initialized in the torque controller
		//      below for simplicity, not here.

		GenSpeedF  = GenSpeed;                       //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		for (int i= 0; i<3; i++) PitCom[i]= BlPitch[i];   //This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
		GK         = 1.0/( 1.0 + PitCom[1 -1]/PC_KK );   //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero
		IntSpdErr  = PitCom[1 -1]/( GK*PC_KI );          //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero

		LastTime   = Time; //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		LastTimePC = Time - PC_DT; //This will ensure that the pitch  controller is called on the first pass 
		LastTimeVS = Time - VS_DT; //This will ensure that the torque controller is called on the first pass 

	}
	if ( iStatus >= 0 /* &&  *aviFAIL >= 0 */ /* NO ERROR CHECKING SO THIS MAKES NO SENSE JUST YET*/ )
	{
	   //Main control calculations:
/*
	   //Abort if the user has not requested a pitch angle actuator (See Appendix A
	   //  of Bladed User's Guide):

	   if ( (int) (avrSWAP[10 -1]) != 0 )
	   {
		   //.TRUE. if a pitch angle actuator hasn't been requested
			*aviFAIL = -1;
			strcpy_s(ErrMsg,512,"Pitch angle actuator not requested.");
	   }
*/
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

	   Alpha     = exp( ( LastTime - Time )*CornerFreq );
	   //Apply the filter:
	   GenSpeedF = ( 1.0 - Alpha )*GenSpeed + Alpha*GenSpeedF;
	   //Variable-speed torque control:
	   //Compute the elapsed time since the last call to the controller:
	   ElapTime = Time - LastTimeVS;


	   //Only perform the control calculations if the elapsed time is greater than
	   //  or equal to the communication interval of the torque controller:
	   //NOTE: Time is scaled by OnePlusEps to ensure that the contoller is called
	   //      at every time step when VS_DT = DT, even in the presence of
	   //      numerical precision errors.

	   if (  (ElapTime+0.001) >= VS_DT )
	   {
	   //Compute the generator torque, which depends on which region we are in:

		  if ( (   GenSpeedF >= VS_RtGnSp ) || (  PitCom[1 -1] >= VS_Rgn3MP ) )   //We are in region 3 - power is constant
			 GenTrq = VS_RtPwr/GenSpeedF;
		  else if ( GenSpeedF <= VS_CtInSp )                                      //We are in region 1 - torque is zero
			 GenTrq = 0.0;
		  else if ( GenSpeedF <  VS_Rgn2Sp )                                      //We are in region 1 1/2 - linear ramp in torque from zero to optimal
			 GenTrq = VS_Slope15*( GenSpeedF - VS_CtInSp );
		  else if ( GenSpeedF <  VS_TrGnSp )                                      //We are in region 2 - optimal torque is proportional to the square of the generator speed
			 GenTrq = VS_Rgn2K*GenSpeedF*GenSpeedF;
		  else                                                                       //We are in region 2 1/2 - simple induction generator transition region
			 GenTrq = VS_Slope25*( GenSpeedF - VS_SySp   );


			//Saturate the commanded torque using the maximum torque limit:
			GenTrq  = max(VS_MinTq,min( GenTrq, VS_MaxTq ));   //Saturate the command using the maximum torque limit


			//Saturate the commanded torque using the torque rate limit:
			if ( iStatus == 0 )  LastGenTrq = GenTrq; //Initialize the value of LastGenTrq on the first pass only

			TrqRate = ( GenTrq - LastGenTrq )/ElapTime; //Torque rate (unsaturated)
			TrqRate = min( max( TrqRate, -VS_MaxRat ), VS_MaxRat ); //Saturate the torque rate using its maximum absolute value
			GenTrq  = LastGenTrq + TrqRate*ElapTime;                 //Saturate the command using the torque rate limit


			//Reset the values of LastTimeVS and LastGenTrq to the current values:
			LastTimeVS = Time;
			LastGenTrq = GenTrq;
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
			//Compute the gain scheduling correction factor based on the previously
			//  commanded pitch angle for blade 1:
			GK = 1.0/( 1.0 + PitCom[0]/PC_KK );

			//Compute the current speed error and its integral w.r.t. time; saturate the
			//  integral term using the pitch angle limits:
			SpdErr    = GenSpeedF - PC_RefSpd;                                 //Current speed error
			IntSpdErr = IntSpdErr + SpdErr*ElapTime;                           //Current integral of speed error w.r.t. time
			//Saturate the integral term using the pitch angle limits, converted to integral speed error limits
			IntSpdErr = min( max( IntSpdErr, PC_MinPit/( GK*PC_KI ) ), PC_MaxPit/( GK*PC_KI ));
		  


			//Compute the pitch commands associated with the proportional and integral
			//  gains:

			PitComP   = GK*PC_KP*   SpdErr; //Proportional term
			PitComI   = GK*PC_KI*IntSpdErr; //Integral term (saturated)


			//Superimpose the individual commands to get the total pitch command;
			//  saturate the overall command using the pitch angle limits:

			PitComT   = PitComP + PitComI;                                     //Overall command (unsaturated)
			PitComT   = min( max( PitComT, PC_MinPit ), PC_MaxPit );           //Saturate the overall command using the pitch angle limits


			//Saturate the overall commanded pitch using the pitch rate limit:
			//NOTE: Since the current pitch angle may be different for each blade
			//      (depending on the type of actuator implemented in the structural
			//      dynamics model), this pitch rate limit calculation and the
			//      resulting overall pitch angle command may be different for each
			//      blade.

		  for (int k=1; k<=numBl; k++) //Loop through all blades
		  {
			 PitRate[k -1] = ( PitComT - BlPitch[k -1] )/ElapTime; //Pitch rate of blade K (unsaturated)
			 PitRate[k -1] = min( max( PitRate[k -1], PC_MinRat ), PC_MaxRat ); //Saturate the pitch rate of blade K using its maximum absolute value
			 PitCom [k -1] = BlPitch[k -1] + PitRate[k -1]*ElapTime; //Saturate the overall command of blade K using the pitch rate limit

			 PitCom[k -1]  = min( max( PitCom[k -1], PC_MinPit ), PC_MaxPit ); //Saturate the overall command using the pitch angle limits         
		  }

	   //Reset the value of LastTimePC to the current value:

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