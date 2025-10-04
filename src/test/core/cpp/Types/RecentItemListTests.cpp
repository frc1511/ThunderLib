#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>

#include <ThunderLibCore/RecentItemList.hpp>

using namespace thunder::core;

using ::testing::ElementsAre;

TEST(RecentItemListTests, DefaultConstructor) {
  RecentItemList<std::string, 5> recentItems;
  EXPECT_EQ(recentItems.size(), 0);
  EXPECT_TRUE(recentItems.empty());
}

TEST(RecentItemListTests, AddUniqueItems) {
  RecentItemList<std::string, 5> recentItems;
  recentItems.add("1");
  recentItems.add("2");
  recentItems.add("3");
  recentItems.add("4");
  recentItems.add("5");
  recentItems.add("6");

  EXPECT_THAT(recentItems, ElementsAre("6", "5", "4", "3", "2"));
}

TEST(RecentItemListTests, UpdateExistingItems) {
  RecentItemList<std::string, 5> recentItems;
  recentItems.add("1");
  recentItems.add("2");
  recentItems.add("3");
  recentItems.add("4");
  recentItems.add("5");
  recentItems.add("3");  // Duplicate
  recentItems.add("6");

  EXPECT_THAT(recentItems, ElementsAre("6", "3", "5", "4", "2"));

  recentItems.add("2");  // Duplicate

  EXPECT_THAT(recentItems, ElementsAre("2", "6", "3", "5", "4"));
}

TEST(RecentItemListTests, RemoveItems) {
  RecentItemList<std::string, 5> recentItems;
  recentItems.add("1");
  recentItems.add("2");
  recentItems.add("3");
  recentItems.add("4");
  recentItems.add("5");

  EXPECT_TRUE(recentItems.remove("3"));
  EXPECT_THAT(recentItems, ElementsAre("5", "4", "2", "1"));

  EXPECT_FALSE(recentItems.remove("6"));  // Non-existent item
  EXPECT_THAT(recentItems, ElementsAre("5", "4", "2", "1"));

  EXPECT_TRUE(recentItems.remove("1"));
  EXPECT_THAT(recentItems, ElementsAre("5", "4", "2"));

  recentItems.add("1");
  EXPECT_THAT(recentItems, ElementsAre("1", "5", "4", "2"));
}

