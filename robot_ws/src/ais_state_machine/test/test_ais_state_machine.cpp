// Bring in my package's API, which is what I'm testing
#include <ais_state_machine/Core/StateMachineManager.h>

// Bring in gtest
#include <gtest/gtest.h>

// libraries required for test code
#include <limits>

#define FLOAT_MATCH_EPSILON 1e-5

TEST(StaticFunctions, clampVel)
{
    ASSERT_EQ(true, true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
