
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	SteeringOutput TempSteering = {};
	
	for (WeightedBehavior Behavior : WeightedBehaviors)
	{
		TempSteering = Behavior.pBehavior->CalculateSteering(DeltaT, Agent);
		BlendedSteering.LinearVelocity += TempSteering.LinearVelocity.GetSafeNormal() * Behavior.Weight;
		BlendedSteering.AngularVelocity += TempSteering.AngularVelocity * Behavior.Weight;
	}
	
	BlendedSteering.LinearVelocity.Normalize();
	
	//ViewDir
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition(), 0.f),
		FVector(Agent.GetPosition() + Agent.GetLinearVelocity() * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Green);
	
	//TargetDir
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition(), 0.f),
		FVector(Agent.GetPosition() + BlendedSteering.LinearVelocity * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Red);

	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If none of the behavior return a valid output, last behavior is returned
	return Steering;
}