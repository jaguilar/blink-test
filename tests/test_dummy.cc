#include "CppUTest/TestHarness.h"

TEST_GROUP(DummyTest)
{
};

TEST(DummyTest, ShouldPass) {
    CHECK_EQUAL(1, 1);
}
