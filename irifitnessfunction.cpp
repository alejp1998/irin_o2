#include "irifitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::CIriFitnessFunction(const char* pch_name, 
                                                                 CSimulator* pc_simulator, 
                                                                 unsigned int un_collisions_allowed_per_epuck)
    :
    CFitnessFunction(pch_name, pc_simulator)
{

	/* Check number of robots */
	m_pcSimulator = pc_simulator;
	TEpuckVector* pvecEpucks=m_pcSimulator->GetEpucks();
	
	if ( pvecEpucks->size() == 0 )
	{
		printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
		fflush(stdout);
		exit(0);
	}
	else if  (pvecEpucks->size()>1)
	{
		printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
	}
    
	m_pcEpuck=(*pvecEpucks)[0];

	m_unNumberOfSteps = 0;
	m_fComputedFitness = 0.0;
	
}

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::~CIriFitnessFunction(){
}
/******************************************************************************/
/******************************************************************************/
int state = 0;

double CIriFitnessFunction::GetFitness()
{    

	/* If you need to check the collisions of the robot, here are the total number of 
	 * collisions done by the robot in the simulations */
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	/* Get the fitness divided by the number of steps */
	double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(coll,100)/100.0)));

	/* If fitness less than 0, put it to 0 */
	if ( fit < 0.0 ) fit = 0.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CIriFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{
	/* See Evolutionary Robotics Book */
	/* This is the function to be implemented */
	/* f = V * ( 1 - sqrt(Delta(v)) ) * (1 - i)
	 * V relates to the maximum speed
	 * Delta(v) relates to the movement on the same direction
	 * i relates to the maximum sensor value
	 */

	/* Get actual SPEED of the left and right wheel */
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));
	
	/* Eval SENSORS */

	/* Where the Max PROXIMITY sensor will be stored*/
	double maxProxSensorEval 		= 0.0;
	/* Where the Max LIGHT sensor will be stored*/
	double maxLightSensorEval 		= 0.0;
	/* whre the BATTERY will be stored */
	double* battery;
	
	double proxS1	=	0.0;
	double proxS2	=	0.0;
	double proxS5	=	0.0;
	double proxS6	=	0.0;
	double lightS0	=	0.0;
	double lightS7	=	0.0;

	/* Auxiluar variables */
	unsigned int unThisSensorsNumberOfInputs; 
	double* pfThisSensorInputs; 
	
	/* Go in all the sensors */
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		/* Check type of sensor */
		switch ( (*i)->GetType() )
		{
			/* If sensor is PROXIMITY */
			case SENSOR_PROXIMITY:
				/* Get the number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxProxSensorEval )
					{	
						/* Store maximum value */
						maxProxSensorEval = pfThisSensorInputs[j];
					}
					//Store sensors 1 & 2 values
					if (j==1)
						proxS1 = pfThisSensorInputs[j];
					else if (j==2)
						proxS2 = pfThisSensorInputs[j];
					else if (j==5)
						proxS5 = pfThisSensorInputs[j];
					else if (j==6)
						proxS6 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is LIGHT */
			case SENSOR_REAL_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxLightSensorEval )
					{	
						/* Store maximum value */
						maxLightSensorEval = pfThisSensorInputs[j];
					}
					//Store sensors 0 & 7 values
					if (j==0)
						lightS0 = pfThisSensorInputs[j];
					else if (j==7)
						lightS7 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is BATTERY */
			case SENSOR_BATTERY:
         		battery = (*i)->GetComputedSensorReadings();
				break;
		}
	}
	
	/* FROM HERE YOU NEED TO CREATE YOUR FITNESS */	

	//Nos aseguramos de que va hacia delante
	double goForwardEval = 0.0;
	if(leftSpeed>0.5 && rightSpeed>0.5){
		goForwardEval = 1.0;
	}

	//Distancia a la pared que preferimos
	double proxUmb = 0.7;
	//Cuanto más se parezca a proxUmb la media de ambos sensores, mayor fitness
	double proxFit = 1 - abs(( 0.5*proxS5 + 0.5*proxS6 ) - proxUmb);

	//Distancia giro sobre luz
	double lightUmb = 0.85;
	//Ver luz por los sensores delanteros aporta puntuacion
	double lightFit = fmin(1.0,( (1/2)*lightS0 + (1/2)*lightS7 )/lightUmb);

	//Parametros fitness
  	double fitness = 0.0;
  	double coef1 = 0.2;
  	double coef2 = 0.8;
  	double coef3 = 0.0;

  	//Escogemos manera de evaluarlo segun la bateria que tenga
  	double batteryUmb = 0.3;
  	if(battery[0]<batteryUmb){
  		state = 1;
  		coef1 = 0.2;
  		coef2 = 0.8;
  		coef3 = 0.0;
  	}else if(battery[0]>(1-batteryUmb)){
  		state = 0;
  		coef1 = 0.2;
  		coef2 = 0.8;
  		coef3 = 0.0;
  	}

  	//Evaluamos comportamiento cada step
  	switch(state){
  		case 0:
  			fitness = coef1*maxSpeedEval;
			fitness += coef2*proxFit;
  			break;

  		case 1:
  			fitness = coef1*maxSpeedEval;
  			fitness += coef2*lightFit;
  			break;
  	}
  	//Nos aseguramos de que va hacia delante
  	fitness *= goForwardEval;
	
	/* TO HERE YOU NEED TO CREATE YOUR FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/
