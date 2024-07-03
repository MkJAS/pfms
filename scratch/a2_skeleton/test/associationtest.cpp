#include "gtest/gtest.h"
#include <iostream>
#include <vector>

// Student defined libraries, for instance
//#include "flightplanner.h"
#include "mockbogiepos.h"


//==================== UNIT TEST START ====================//


TEST(AssociationTest, RangeBearingToBogie)
{
  Mockbogiepos m;
  m.setglb_bogie_pos_({{{2000,2000},0},{{3000,1000},0},{{-3000,-2000},0},{{-3000,3000},0}});
  std::vector<RangeBearingStamped> rbs = {{2828.4,0.7854,1},{3162.27766,0.3218,1},{3605.5513,3.7296,1},{4242.6407,2.3562,1}};
  Point friendxy = {0,0,0};
  std::vector<std::pair<int,int>> v = m.rangebearassociate(rbs,friendxy);

  for(unsigned int i=0;i<rbs.size();i++)
  {
    ASSERT_EQ(v[i].first,v[i].second);
  }
  //Change order around
  rbs = {{4242.6407,2.3562,1},{3162.27766,0.3218,1},{2828.4,0.7854,1},{3605.5513,3.7296,1}};
  v = m.rangebearassociate(rbs,friendxy);

  //v[i].first = the index of the original globalxy vector that relates to the ith element of rbs
  ASSERT_EQ(v[0].first,3); 
  ASSERT_EQ(v[1].first,1);  
  ASSERT_EQ(v[2].first,0); 
  ASSERT_EQ(v[3].first,2);
  //v[1].first = 1, because the 2nd element in rbs is {3162.27766,0.3218,1}
  //which relates to the second element in the glb_bogie_pos_ vector


  //move friendly
  friendxy = {2000,0,0};
  rbs = {{5385.16481,1.9513,2},{2000,0,2},{5830.9519,1.0304,1},{1414.2136,5.49779,2}};
  //we swap the order around to simulate the friendly moving closer to a bogie such that
  //the return of rbs from the sim will change order
  //glb_bogie_pos_({{{2000,2000},0},{{3000,1000},0},{{-3000,-2000},0},{{-3000,3000},0}});
  v = m.rangebearassociate(rbs,friendxy);

  ASSERT_EQ(v[0].first,2); 
  ASSERT_EQ(v[1].first,0);  
  ASSERT_EQ(v[2].first,3); 
  ASSERT_EQ(v[3].first,1);



}

TEST(AssociationTest, RangeVelocityToBogie)
{
  Mockbogiepos m;
  m.setglb_bogie_pos_({{{2000,2000},0},{{3000,1000},0},{{-3000,-2000},0},{{-3000,3000},0}});
  std::vector<RangeVelocityStamped> rvs = {{2828.4,0.7854,1},{3162.27766,0.3218,1},{3605.5513,3.7296,1},{4242.6407,2.3562,1}};
  Point friendxy = {0,0,0};
  std::vector<std::pair<int,int>> v = m.rangevelassociate(rvs);

  for(unsigned int i=0;i<rvs.size();i++)
  {
    ASSERT_EQ(v[i].first,v[i].second);
  }

  //Change order around
  rvs = {{4242.6407,2.3562,1},{3162.27766,0.3218,1},{2828.4,0.7854,1},{3605.5513,3.7296,1}};
  v = m.rangevelassociate(rvs);

  //v[i].first = the index of the original globalxy vector that relates to the ith element of rbs
  //sice bogie do not move in basic mode, the relation between the base Range readings and the bogies
  //will always be the same
  ASSERT_EQ(v[0].first,3); 
  ASSERT_EQ(v[1].first,1);  
  ASSERT_EQ(v[2].first,0); 
  ASSERT_EQ(v[3].first,2);

  //remove an element to simulate a bogie being intercepted
  rvs = {{3162.27766,0.3218,1},{3605.5513,3.7296,1},{4242.6407,2.3562,1}};
  v = m.rangevelassociate(rvs);
  ASSERT_EQ(v[0].first,1); 
  ASSERT_EQ(v[1].first,2);  
  ASSERT_EQ(v[2].first,3); 




}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
