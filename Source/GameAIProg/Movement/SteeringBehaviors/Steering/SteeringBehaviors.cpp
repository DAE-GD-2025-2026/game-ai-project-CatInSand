#include "SteeringBehaviors.h"

#include "MeshPaintVisualize.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	FVector2D Direction{ (Target.Position - Agent.GetPosition()).GetSafeNormal() };
	if (Distance > 1.f)
	{
		Steering.LinearVelocity = Direction;
	}
	
	//Debug
	FVector2D AgentViewDir{ Agent.GetLinearVelocity().GetSafeNormal() };
	
	//ToTarget
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + Direction.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + Direction.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Red);
	
	//ViewDir
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + AgentViewDir.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + AgentViewDir.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Green);

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	FVector2D Direction{ -(Target.Position - Agent.GetPosition()).GetSafeNormal() };
	Steering.LinearVelocity = Direction;
	
	//Debug
	FVector2D AgentViewDir{ Agent.GetLinearVelocity().GetSafeNormal() };

	//Away from target
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + Direction.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + Direction.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Red);
	
	//ViewDir
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + AgentViewDir.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + AgentViewDir.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Green);

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

	float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	FVector2D Direction{ (Target.Position - Agent.GetPosition()).GetSafeNormal() };
	Steering.LinearVelocity = Direction;
	
	//set speed
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

	//Debug
	FVector2D AgentViewDir{ Agent.GetLinearVelocity().GetSafeNormal() };

	//Away from target
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + Direction.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + Direction.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Red);
	
	//ViewDir
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		FVector(Agent.GetPosition().X + AgentViewDir.X * Agent.GetMaxLinearSpeed(), Agent.GetPosition().Y + AgentViewDir.Y * Agent.GetMaxLinearSpeed(), 0.f),
		FColor::Green);
	
	//SlowCircle
	DrawDebugCircle(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		SlowRadius,
		16,
		FColor::Blue,
		false,-1,0,0,
		FVector(0,1,0), FVector(1,0,0));
	
	//TargetCircle
	DrawDebugCircle(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f),
		TargetRadius,
		16,
		FColor::Orange,
		false,-1,0,0,
		FVector(0,1,0), FVector(1,0,0));

	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	if (FirstCall)
	{
		MaxSpeed = Agent.GetMaxLinearSpeed();
		FirstCall = false;
	}
	
	SteeringOutput Steering{};

	//add angular velocity handling in SteeringAgent.cpp
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize(); //Already happens when processed
	
	Agent.SetMaxLinearSpeed(0.f);

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	const float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	const float ArrivalTime{ Distance / Agent.GetMaxLinearSpeed() }; //time until collision if target is static
	const FVector2D DisplacedPosition{ Target.Position + Target.LinearVelocity * ArrivalTime }; //position of target if it moves at constant velocity and direction

	Steering.LinearVelocity = DisplacedPosition - Agent.GetPosition();
	Steering.LinearVelocity.Normalize(); //Already happens when processed

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	const float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
	const float ArrivalTime{ Distance / Agent.GetMaxLinearSpeed() }; //time until collision if target is static
	const FVector2D DisplacedPosition{ Target.Position + Target.LinearVelocity * ArrivalTime }; //position of target if it moves at constant velocity and direction

	Steering.LinearVelocity = -(DisplacedPosition - Agent.GetPosition());
	Steering.LinearVelocity.Normalize(); //Already happens when processed

	//Add debug rendering for grades :3

	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	const float Angle{ static_cast<float>(rand() % 180) / 180.f * PI };
	const FVector2D TargetPosition{ Agent.GetPosition() + Agent.GetLinearVelocity().Normalize() * CircleDistance + FVector2D{cos(Angle) * CircleRadius, sin(Angle) * CircleRadius} };

	Steering.LinearVelocity = TargetPosition - Agent.GetPosition();
	Steering.LinearVelocity.Normalize(); //Already happens when processed

	//Add debug rendering for grades :3

	return Steering;
}