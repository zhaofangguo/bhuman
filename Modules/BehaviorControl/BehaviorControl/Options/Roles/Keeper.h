option(Keeper)
{
initial_state(start)
{
    transition
    {
        if(state_time>1000)
        goto TurnToRightAngle;    
    }
    action
    {
        HeadControlMode(HeadControl::lookLeftAndRight);
        Stand();
    }
}
state(TurnToRightAngle)
{
  transition
  {
    if(state_time>3000)
    goto WalkToDestination;
  }
  action
  {
    HeadControlMode(HeadControl::lookForward);
     WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(18_deg,0.f,0.f));
}
}
state(WalkToDestination)
{
   transition
    {
        if(theOdometer.distanceWalked>16090.f)//theOdometer.distanceWalked>5408.f
        goto TurnToTeammate;
       
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,3000.f,-5300.f));//4500,4000
        
    }
}
state(TurnToTeammate)
{
  transition
  {
    if(theLibCodeRelease.timeSinceBallWasSeen<300)
    
    goto launch;
  }
  action
{
  HeadControlMode(HeadControl::lookForward);
   WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
}
}
state(launch)
{
    transition
    {
        if(theBallModel.estimate.position.norm()>4500.f)
        goto Faraway;
        else if (theBallModel.estimate.position.norm()<=4500.f)
        goto  BallisComing; 
        else
        {
            goto SearchForBall;
        }
        
    }
    action
    {
        HeadControlMode(HeadControl::lookAtBall);
        Stand();
    }
}
state(Faraway)
{
    transition
    {
        if(theBallModel.estimate.position.norm()<4500.f)
        goto BallisComing;
        if(theLibCodeRelease.timeSinceBallWasSeen >7000&&theOdometer.distanceWalked>10100.f)
        goto SearchForBall;
    }
    action
    {
        HeadControlMode(HeadControl::lookAtBall);
         WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),0.f,0.f));
    }
}
state(BallisComing)
{
    transition
    {
         if(theBallModel.estimate.position.norm()<1000.f)
        goto GoToBall;
         if(theLibCodeRelease.timeSinceBallWasSeen >7000&&theOdometer.distanceWalked>10100.f)
        goto SearchForBall;
    }
    action
    {
         HeadControlMode(HeadControl::lookAtBall);
         WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),0.f,0.f));
    }
}
state(GoToBall)
{
    transition
    {
        if(theBallModel.estimate.position.x()<100.f&&theBallModel.estimate.position.y()<100.f)
        goto Kick;
    }
    action
    {
       HeadControlMode(HeadControl::lookAtBall);
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),theBallModel.estimate.position));
    }
}
state(Kick)
{
    transition
    {
        if(state_time > 3000 || (state_time > 10 && action_done))
        goto launch;
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0.f, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
}
state(SearchForBall)
{
    transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen<300)
        goto launch;
    }
    action
    {
    HeadControlMode(HeadControl::lookAtBall);
    WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
}
}
















