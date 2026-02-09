#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize(); //Already happens when processed

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity =  -(Target.Position - Agent.GetPosition());
	Steering.LinearVelocity.Normalize(); //Already happens when processed

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	if (FirstCall)
	{
		MaxSpeed = Agent.GetMaxLinearSpeed();
		FirstCall = false;
	}
	
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize(); //Already happens when processed
	
	//set speed
	float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	if (Distance < SlowRadius)
	{
		if (Distance < TargetRadius)
		{
			Agent.SetMaxLinearSpeed(0.f);
		}
		else
		{
			float Interpolation{ (Distance - TargetRadius) / (SlowRadius - TargetRadius) };
			Agent.SetMaxLinearSpeed(MaxSpeed * Interpolation);
		}
	}
	else
	{
		Agent.SetMaxLinearSpeed(MaxSpeed);
	}

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.AngularVelocity = Target.AngularVelocity - Agent.GetRotation();

	//Add debug rendering for grades :3

	return Steering;
}