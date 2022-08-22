// Bring in my package's API, which is what I'm testing
#include <amr_localization/1d_localization.h>

// Bring in gtest
#include <gtest/gtest.h>

// libraries required for test code
#include <limits>

#define FLOAT_MATCH_EPSILON 1e-5

TEST(StaticFunctions, testName)
{
    float output;
    float input = 0.5;

    output = 0.5;//oneDNode::test_function(input);
    EXPECT_NEAR(0.5, output,  FLOAT_MATCH_EPSILON);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
