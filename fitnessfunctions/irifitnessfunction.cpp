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

double CIriFitnessFunction::GetFitness()
{    

	/* If you need to check the collisions of the robot, here are the total number of 
	 * collisions done by the robot in the simulations */
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	/* Get the fitness divided by the number of steps */
	double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(coll,50)/50.0)));

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
	double maxSpeedEval = fmax(fabs(leftSpeed),fabs(rightSpeed));
	/* Eval same directio partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval SENSORS */

	/* Where the Max PROXIMITY sensor will be stored*/
	double maxProxSensorEval = 0.0;
	/* Where the Max LIGHT sensor will be stored*/
	double maxYellowEval	= 0.0;
	double maxBlueEval		= 0.0;
	double maxRedEval		= 0.0;

	/* Where particular sensors will be stored*/	
	double yellowS0	=	0.0;
	double yellowS7	=	0.0;
	double redS1	=	0.0;
	double redS2	=	0.0;
	double blueS5	=	0.0;
	double blueS6	=	0.0;

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
					if ( pfThisSensorInputs[j] > maxYellowEval )
					{	
						/* Store maximum value */
						maxYellowEval = pfThisSensorInputs[j];
					}
					//Store sensors 0 & 7 values
					if (j==0)
						yellowS0 = pfThisSensorInputs[j];
					else if (j==7)
						yellowS7 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is BLUE LIGHT */
			case SENSOR_REAL_BLUE_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{

					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxBlueEval )
					{	
						/* Store maximum value */
						maxBlueEval = pfThisSensorInputs[j];
					}
					//Store sensors 0 & 7 values
					if (j==5)
						blueS5 = pfThisSensorInputs[j];
					else if (j==6)
						blueS6 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is RED LIGHT */
			case SENSOR_REAL_RED_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{

					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxRedEval )
					{	
						/* Store maximum value */
						maxRedEval = pfThisSensorInputs[j];
					}
					//Store sensors 0 & 7 values
					if (j==1)
						redS1 = pfThisSensorInputs[j];
					else if (j==2)
						redS2 = pfThisSensorInputs[j];
				}
				break;
		}
	}
	
	/* FROM HERE YOU NEED TO CREATE YOUR FITNESS */	

	/*INTENTO DE SEGUIMIENTO PAREDES Y DESPUES BATERIA
	//Distancia a la pared que preferimos
	double proxFit, lightFit = 0.0;
	double proxUmb = 0.75;

	//Cuanto más se parezca a proxUmb la media de ambos sensores, mayor fitness
	proxFit = 1 - abs(( 0.5*proxS1 + 0.5*proxS2 ) - proxUmb);
	lightFit = (lightS0 + lightS7)/2;

	//Parametros fitness
  	double fitness = 0.0;
  	double coef1 = 0.25;
  	double coef2 = 0.75;
  	double coef3 = 0.0;

  	//Escogemos manera de evaluarlo segun la bateria que tenga
  	double batteryUmb = 0.3;
  	if(battery[0]<batteryUmb){
  		state = 1;
  		coef1 = 0.2;
  		coef2 = 0.3;
  		coef3 = 0.5;
  	}else if(battery[0]>(1-batteryUmb)){
  		state = 0;
  		coef1 = 0.25;
  		coef2 = 0.75;
  		coef3 = 0.0;
  	}

  	//Evaluamos comportamiento cada step
  	switch(state){
  		//Rodear paredes mientras pierde bateria
  		case 0:
  			fitness = coef1*maxSpeedEval*sameDirectionEval;
			fitness += coef2*proxFit;
  			break;

  		//Ir hacia la luz para recargar bateria	
  		case 1:
  			fitness = coef1*maxSpeedEval*sameDirectionEval;
  			fitness += coef2*lightFit;
  			fitness += coef3*fmin(1.0,battery[0]/batteryUmb);
  			break;
  	}
  	FINAL DE INTENTO SEGUIMIENTO PAREDES*/

  	/*INTENTO DE ORBITA A DISTANCIAS DISTANCIAS Y SENTIDOS SOBRE LUCES*/
	//Distancias de giro
	double blueUmb = 0.5;
	double redUmb  = 0.5;

	//Parametros fitness
  	double fitness = 0.0;
  	double coef1 = 0.25;
  	double coef2 = 0.75;

  	//Acercarse a la luz amarilla
  	double yellowFit = (yellowS0 + yellowS7)/2;

  	//Rodear luces azul y roja
  	double blueFit = 1 - abs(( blueS5 + blueS6 )/2 - blueUmb);
  	double redFit = 1 - abs(( redS1 + redS2 )/2 - redUmb);


  	//Calculo de fitness segun luz encendida
  	if(maxYellowEval>0.0){
  		fitness = coef1*maxSpeedEval*sameDirectionEval;
  		fitness += coef2*yellowFit;
  	}else if(maxBlueEval>0.0){
  		fitness = coef1*maxSpeedEval;
  		fitness += coef2*blueFit;
  	}else if(maxRedEval>0.0){
  		fitness = coef1*maxSpeedEval;
  		fitness += coef2*redFit;
  	}
	/* TO HERE YOU NEED TO CREATE YOUR FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/
