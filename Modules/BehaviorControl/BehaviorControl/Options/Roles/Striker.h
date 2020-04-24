/** A test striker option without common decision */
option(Striker)
{
  int i=0;
  int robotlist[7];
  int robotnumber=0;
  int num=0;
  int teammatenum=0;
  static float x,y;
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)//这里的state_time=1000应该是state执行的时间吧，我猜的
        goto JudgeBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);//在一个枚举里定义，有好多参数可以选择
      Stand();//目前来看是在进球后会跳转到stand中等待
    }
  }
  
state(JudgeBall)
{
  transition
  {
    if(state_time>500&&theObstacleModel.obstacles.size()!=0)
    goto IfOrNot;
    if(theObstacleModel.obstacles.size()==0)
    goto stand;
  }
  action
  {
    HeadControlMode(HeadControl::lookForward);
     if(theObstacleModel.obstacles.size()!=0)
   {
    for(int j=0,i=0;i<int(theObstacleModel.obstacles.size());i++)
     {
      if(theObstacleModel.obstacles[i].type==Obstacle:: opponent||theObstacleModel.obstacles[i].type==Obstacle:: teammate)
     { robotlist[j]=i;j++;num++; }
     }
     for(int j=0;j<num;j++)
     { if(theObstacleModel.obstacles[j].type==Obstacle:: teammate)
      {teammatenum=j;
      break;
      }}
     for(int j=0;j<num;j++)
     {
       if(j==0)robotnumber=robotlist[j];
       if(std::fabs(theObstacleModel.obstacles[robotlist[j]].center.y())>std::fabs(theObstacleModel.obstacles[robotnumber].center.y()))
        robotnumber=robotlist[j];
        
     }}
     Stand();
  }
}
state(IfOrNot)
{
  transition
  {
     if((theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut||(std::fabs(theBallModel.estimate.position.x())>std::fabs(theObstacleModel.obstacles[i].center.x())))&&theObstacleModel.obstacles.size()!=2)//初始值是7000ms
        goto FindRobot; //壁障
    if(theObstacleModel.obstacles.size()==2)
     //goto SnatchTheBall;
      goto turnToBall;//传球
      if((std::fabs(theBallModel.estimate.position.x())<=std::fabs(theObstacleModel.obstacles[i].center.x()))&&(std::fabs(theObstacleModel.obstacles[i].center.y())-(std::fabs(theBallModel.estimate.position.y()))<=500.f)&&theObstacleModel.obstacles.size()!=2)
        goto SnatchTheBall;//抢球
     
      
  }
  action
  {
    HeadControlMode(HeadControl::lookForward);
    Stand();
  }
}

  state(SnatchTheBall)
  {
    transition
    {
      
      if(theBallModel.estimate.position.norm() < 500.f)//原来是500.f如果球的什么参数小于500？这个参数还不太懂
        goto alignToGoalwhenSnatch;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);//此处的target直接就是球的位置，走向球
    }
  }

  state(alignToGoalwhenSnatch)
  {
    transition
    {
      
     if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBallwhenSnatch;//目标角度小于10度且球的y坐标小于100时
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));//这里的x坐标-400应该是和机器人的脚的宽度有关系吧，我猜的
      //Stand();
    }//发现当pose2f作为速度变量是似乎都是用同一个构造函数
  }

  state(alignBehindBallwhenSnatch)//在球的后方对齐，也不知道对齐哪里，还没看明白
  {
    transition
    {
      
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)//这块的lookForwardbetween还有待我仔细研究，为什么是这个判定条件？angleToGoal原来是相对球门的角度，这下可弄明白了
        goto kickToBeside;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));//此处的走向目标似乎是确定的目标?等我运行看看
    }//这里的速度变了？
  }

  state(kickToBeside)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))//第一个判定条件应该是为了解决没踢到球的状况，第二个条件是踢到球的状况
        goto stand;//原来是start
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }//方向，腿，踢球姿势？，不过这个姿势很奇怪啊，球门角度，球后方160mm，左边55mm

  }
  
  state(stand)
  {
    action
    {
      HeadControlMode(HeadControlMode::lookForward);
      Stand();
    }
  }
  state(turnToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)//初始值是7000ms
        goto searchForBall;//原来是searchForBall
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));//两个参数是速度和目标位置，位置是相对机器人的位置
    }//转向球时不需要考虑球的位置，只用调整到对准球的方向即可，所以theBallModel.estimate.position的
  }

  state(walkToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 500.f&&theObstacleModel.obstacles.size()==2)//原来是500.f如果球的什么参数小于500？这个参数还不太懂
        goto alignToGoal;
        if(theBallModel.estimate.position.norm() < 500.f&&theObstacleModel.obstacles.size()!=2)
        goto FindTeammate;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      if(theObstacleModel.obstacles.size()!=0)
   {
     for(int j=0;j<int(theObstacleModel.obstacles.size());j++)
     { if(theObstacleModel.obstacles[j].type==Obstacle:: teammate)
      {teammatenum=j;
      break;
      }}
     }
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);//此处的target直接就是球的位置，走向球
    }
  }
state(FindTeammate)
{
  transition
  {
    if(theObstacleModel.obstacles[1].type==Obstacle::teammate)
    goto alignToGoal;
  }
  action
  {
    HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
  }
}
  state(alignToGoal)
  { if(theObstacleModel.obstacles.size()!=0)
   {
     for(int j=0;j<int(theObstacleModel.obstacles.size());j++)
     { if(theObstacleModel.obstacles[j].type==Obstacle:: teammate)
      {teammatenum=j;
      break;
      }}
     }
    transition
    {
      
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      goto searchForBall;
     if(std::abs(theObstacleModel.obstacles[teammatenum].center.angle()) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;//目标角度小于10度且球的y坐标小于100时
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
       
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theObstacleModel.obstacles[teammatenum].center.angle(), theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));//这里的x坐标-400应该是和机器人的脚的宽度有关系吧，我猜的
      //Stand();
    }//发现当pose2f作为速度变量是似乎都是用同一个构造函数
  }

  state(alignBehindBall)//在球的后方对齐，也不知道对齐哪里，还没看明白
  { if(theObstacleModel.obstacles.size()!=0)
   {
     for(int j=0;j<int(theObstacleModel.obstacles.size());j++)
     { if(theObstacleModel.obstacles[j].type==Obstacle:: teammate)
      {teammatenum=j;
      break;
      }}
     }
    transition
    {
       
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(theObstacleModel.obstacles[teammatenum].center.angle()) < 2_deg)//这块的lookForwardbetween还有待我仔细研究，为什么是这个判定条件？angleToGoal原来是相对球门的角度，这下可弄明白了
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theObstacleModel.obstacles[teammatenum].center.angle(), theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));//此处的走向目标似乎是确定的目标?等我运行看看
    }//这里的速度变了？
  }

  state(kick)
  { if(theObstacleModel.obstacles.size()!=0)
   {
     for(int j=0;j<int(theObstacleModel.obstacles.size());j++)
     { if(theObstacleModel.obstacles[j].type==Obstacle:: teammate)
      {teammatenum=j;
      break;
      }}
     }
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))//第一个判定条件应该是为了解决没踢到球的状况，第二个条件是踢到球的状况
        goto stand;//原来是start
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
       
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theObstacleModel.obstacles[teammatenum].center.angle(), theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }//方向，腿，踢球姿势？，不过这个姿势很奇怪啊，球门角度，球后方160mm，左边55mm

  }
 
 state(FindRobot)
 { 
   transition
   {
    
     if((fabs(theObstacleModel.obstacles[robotnumber].left.y())>=fabs(theObstacleModel.obstacles[robotnumber].right.y())))
     goto GoToBesideLeftObstacle;
     if((fabs(theObstacleModel.obstacles[robotnumber].left.y())<fabs(theObstacleModel.obstacles[robotnumber].right.y())))
     goto GoToBesideRightObstacle;
   }
   action
   { if(theObstacleModel.obstacles.size()!=0)
   {
    for(i=0;i<int(theObstacleModel.obstacles.size());i++)
     {
     if(i==0)robotnumber=i;
     if(std::fabs(theObstacleModel.obstacles[i].center.y())>=std::fabs(theObstacleModel.obstacles[robotnumber].center.y()))
     {
       robotnumber=i;
     }
     }}   x=theObstacleModel.obstacles[robotnumber].center.x();
     y=theObstacleModel.obstacles[robotnumber].center.y();
      HeadControlMode(HeadControl::lookForward);
     Stand();
   }
 }
  state(GoToBesideLeftObstacle)
  { 
    transition
    {
      if(state_time>23000)
        goto searchForBall;
    } 
    action
    {
        HeadControlMode(HeadControl:: lookForward);
       
     
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,x+200.f,y+500.f));//修改
    }
  }
   state(GoToBesideRightObstacle)
  {
    transition
    {
      if(state_time>23000)
        goto searchForBall;
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,x+200.f,y-500.f));//修改
    }
  }
  state(searchForBall)
  {
    transition
    {
     if(theLibCodeRelease.timeSinceBallWasSeen < 300)//找出限定条件？
     //if(theObstacleModel.obstacles[0].center.x()==0)
        goto turnToBall;//原来是turnToBall
      //if(state_time >10000)
        //goto GoToFindBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));//此处的参数只有角度变量，所以可以进行原地转圈的检测方法
     // WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,theObstacleModel.obstacles[i].center.x(),theObstacleModel.obstacles[i].center.y()+500.f));//修改
    }
  }

}
//下一步的思路：尽量找到识别机器人的参数，这样就可以完成避开机器人的方案，如果不行就只能朝着别的方向走走再找球
/*option(Striker)
{ 
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)//这里的state_time应该是state执行的时间吧，我猜的
        goto FindBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);//在一个枚举里定义，有好多参数可以选择
      Stand();//目前来看是在进球后会跳转到stand中等待
    }
  }
  state(FindBall)
  {
    transition
    {
      
    } 
    action
    {
       HeadControlMode(HeadControl::lookForward);//在一个枚举里定义，有好多参数可以选择
       Stand();//目前来看是在进球后会跳转到stand中等待
    }
  }

  state(SearchForObstacle)
  {
    transition
    {
      if(theObstacleModel.obstacles[0].type )
       goto start;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
  }

  state(GoToBesideObstacle)
  {
    transition
    {
      if(action_done)
        goto Observe;
    }
    action
    {
        HeadControlMode(HeadControl::lookForward);
       WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0.f,theObstacleModel.obstacles[0].center.x(),theObstacleModel.obstacles[0].center.y()+700.f));
    }
  }

  state(WalkToBall)
  {
    transition
    {
      if(action_done)
        goto Stand;
    }
    action
    {
      HeadControlMode(HeadControl::lookActiveWithBall);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),theBallModel.estimate.position ));
    }
  }

  state(Observe)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto WalkToBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookLeftAndRight);
    }
  }
  state(Stand)
  {
    action
    {
      Stand();
    }
  }
}
if((!theObstacleModel.obstacles[0].type )&&(theLibCodeRelease.timeSinceBallWasSeen > 300))//没有障碍物并且没有球
        goto SearchForObstacle;
      if((theObstacleModel.obstacles[0].type )||(theBallModel.estimate.position.x()>theObstacleModel.obstacles[0].center.x()))//有障碍物或障碍物距离小于球
        goto GoToBesideObstacle;
      if((theObstacleModel.obstacles[0].type )&&(theBallModel.estimate.position.x()<theObstacleModel.obstacles[0].center.x()))//有障碍物且障碍物距离大于球
        goto WalkToBall;
      if((!theObstacleModel.obstacles[0].type )&&(theLibCodeRelease.timeSinceBallWasSeen < 300))//没有障碍物且有球
        goto WalkToBall;*/