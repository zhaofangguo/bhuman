option(PlayingState)
{
  initial_state(demo)
  {
    action
    {
      if(theGameInfo.kickingTeam==theOwnTeamInfo.teamNumber)
      {
        switch(theRobotInfo.number)
        {
          case 1:
          {
            Receiver();
            break;
          }
          case 2:
          {
            Striker();
            break;
          }
        }
      }
      
    }
  }
}
