

#ifndef OWN_LOCAL_PLANNER_JOIN_COSTMAP_H_
#define OWN_LOCAL_PLANNER_JOIN_COSTMAP_H_

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <vector>
using std::vector;

namespace own_local_planner
{
    class JoinCostmap
    {
    public:
        /**
         * @brief JoinCostmap constructor.
         */
        JoinCostmap();

        /**
         * @brief initialize the two costmaps and check if resolution global costmap > resolution local costmap. With is neseccary.
         * @param local_costmap_ros
         * @param global_costmap_ros
         */
        void initialize(costmap_2d::Costmap2DROS* local_costmap_ros, costmap_2d::Costmap2DROS* global_costmap_ros);

        /**
         * @brief joinMaps join the local costmap in the global costmap.
         */
        void joinMaps();

    private:
        costmap_2d::Costmap2DROS* global_costmap_ros_;
        costmap_2d::Costmap2DROS* local_costmap_ros_;

        //Vector with all costvalues from the original global costmap.
        vector<vector<int> > global;

        //Is initialization true.
        bool init;
    };
}
#endif
