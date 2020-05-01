option(Receiver)
{
int i;
initial_state(start)
{
    transition
    {
        if(state_time > 1000)
        goto TurnToRightAngle;    
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
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
     WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(12_deg,0.f,0.f));
  }
}
state(WalkToDestination)
{
   transition
    {
        if(theOdometer.distanceWalked>16000.f)//theOdometer.distanceWalked>5408.f
        goto TurnToTeammate;
       
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,4500.f,4000.f));
        
    }
}
state(TurnToTeammate)
{
  transition
  {
    if(theObstacleModel.obstacles.size()!=0)
     { for(int j=0;j<int(theObstacleModel.obstacles.size());j++)
    {if(theObstacleModel.obstacles[j].type==Obstacle::teammate)
    goto ReadyToReceive;}}
  }
  action
{
  HeadControlMode(HeadControl::lookForward);
   WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
}
}
/*state(JudgeTeammate)
{
    transition
    {
        if(state_time>5000)
        goto ReadyToReceive;
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
        for(i=0;i<int(theObstacleModel.obstacles.size());i++)
        {
            if(theObstacleModel.obstacles[i].type==Obstacle:: teammate)
            break;
        }
        Stand();
    }
}*/

state(NoObstacle)
{
    action
    {
    HeadControlMode(HeadControl::lookForward);
    Stand();
    }
}
state(ReadyToReceive)
{for(i=0;i<int(theObstacleModel.obstacles.size());i++)
        {
            if(theObstacleModel.obstacles[i].type==Obstacle::teammate)
            break;
        }
    transition
    {
         
         if((theBallModel.estimate.position.norm()<500.f)&&(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut))
        goto searchForBall;
        if((theBallModel.estimate.position.norm()<500.f)&&(theLibCodeRelease.timeSinceBallWasSeen < 300))
        goto walkToBall;
    }
    
    action
    {
        HeadControlMode(HeadControl::lookForward);
        Stand();
    }
}
 state(walkToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 400.f)//原来是500.f如果球的什么参数小于500？这个参数还不太懂
        goto alignToGoal;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);//此处的target直接就是球的位置，走向球
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      goto searchForBall;
     if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;//目标角度小于10度且球的y坐标小于100时
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));//这里的x坐标-400应该是和机器人的脚的宽度有关系吧，我猜的
      //Stand();
    }//发现当pose2f作为速度变量是似乎都是用同一个构造函数
  }

  state(alignBehindBall)//在球的后方对齐，也不知道对齐哪里，还没看明白
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)//这块的lookForwardbetween还有待我仔细研究，为什么是这个判定条件？angleToGoal原来是相对球门的角度，这下可弄明白了
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));//此处的走向目标似乎是确定的目标?等我运行看看
    }//这里的速度变了？
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))//第一个判定条件应该是为了解决没踢到球的状况，第二个条件是踢到球的状况
        goto NoObstacle;//原来是start
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }//方向，腿，踢球姿势？，不过这个姿势很奇怪啊，球门角度，球后方160mm，左边55mm

  }

/*state(GoToFindBall)
{
    transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 400.f)//原来是500.f如果球的什么参数小于500？这个参数还不太懂
        goto alignToGoal;
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
}*/

state(searchForBall)
{
    transition
    {
         if(theLibCodeRelease.timeSinceBallWasSeen < 300)
         goto walkToBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
}
/*state()
{
    transition
    {

    }
    action
    {

    }
}*/
}