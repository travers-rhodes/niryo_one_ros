// see http://wiki.ros.org/gtest
// Bring in my package's API, which is what I'm testing
#include "niryo_one_tutorial/transform_helpers.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, distance_function)
{
  Eigen::Vector3d a(0,0,1), b(1,-1,2);
  EXPECT_FLOAT_EQ(distance(Eigen::Translation3d(a), Eigen::Translation3d(b)), std::sqrt(3));
  EXPECT_FLOAT_EQ(distance(Eigen::Translation3d(a), Eigen::Translation3d(a)), 0);
  EXPECT_FLOAT_EQ(distance(Eigen::Translation3d(b), Eigen::Translation3d(a)), std::sqrt(3));
}

TEST(TestSuite, quat_distance_function)
{
  Eigen::Vector3d axis_a(0,0,1), axis_b(1,0,0);
  Eigen::Quaternion<double> q_0(Eigen::AngleAxis<double>(0, axis_a)), 
                     q_1(Eigen::AngleAxis<double>(M_PI,axis_a)), 
                     q_2(Eigen::AngleAxis<double>(M_PI/2, axis_b)),
                     q_3(Eigen::AngleAxis<double>(M_PI/2, axis_a));
  EXPECT_FLOAT_EQ(quat_distance(q_0, q_0), 0);
  EXPECT_FLOAT_EQ(quat_distance(q_0, q_1), M_PI);
  EXPECT_FLOAT_EQ(quat_distance(q_0, q_2), M_PI/2);
  EXPECT_FLOAT_EQ(quat_distance(q_1, q_2), M_PI);
  EXPECT_FLOAT_EQ(quat_distance(q_1, q_3), M_PI/2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  //ros::init(argc, argv, "tester");
  //ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
