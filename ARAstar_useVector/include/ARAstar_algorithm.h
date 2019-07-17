#ifndef PLANNER_ARASTAR_ALGORITHM_H
#define PLANNER_ARASTAR_ALGORITHM_H

#include <float.h>
#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"

typedef std::numeric_limits<float> INF_f;  // float最大值
void SearchOneMap(int map_num_);           //对一张地图进行ARA*搜索
void PrintSumResult();                     //打印结果统计

//每一个cell的信息，使用结构体定义
struct CellInfo {
  Points xoy_;     //坐标点
  float f_value_;  // f(s)=g(s)+h(s)
  float h_value_;  //当前状态到终点的启发距离值
  float g_value_;  //起始点到当前点的距离值
  float v_value_;  //引入的v(s)值

  bool operator<(const CellInfo& pos) const {
    // if f-vaule 相同，比较 g-vaule
    if (f_value_ == pos.f_value_)
      return g_value_ < pos.g_value_;
    else
      return f_value_ < pos.f_value_;
  }
};

class ARAstar {
 public:
  ARAstar(int16_t row_arg, int16_t column_arg, Points statr_arg,
          Points goal_arg,
          std::vector<Points> obstacle_list_arg);  //构造函数
  ARAstar(const ARAstar& as) = delete;             //不使用复制构造函数

  //*****一次ARA*算法所需函数*****//
  bool AstarAlgorithm(CellInfo& start, CellInfo& goal);  // A*算法
  bool ARAstarGetPath();  // 一次ARA*算法函数
  std::vector<Points> GetNeighborsPoint(const Points& current_pos);
  std::vector<CellInfo> GetNeighborsInfo(const Points& current_pos);
  void PrintSearchResult();

  //计算h_value
  inline float DistenceToGoal(const Points& current) {
    return static_cast<float>(abs(current.first - goal_pos_.first) +
                              abs(current.second - goal_pos_.second));
  }

  //递减启发式的启发因子
  inline void DecreaseHeuristicFactor(const CellInfo& goal_arg) {
    heuristic_factor_ =
        std::min(heuristic_factor_,
                 (goal_arg.g_value_ / (OpenLIstPopMinElem().g_value_ +
                                       OpenLIstPopMinElem().h_value_)));
    std::cout << "heuristic_factor_ =  " << heuristic_factor_ << std::endl;
  }
  void InconsPushOpenlist(const std::vector<CellInfo>& incons_list_arg);

  void UpdateOpenlisByNewFactor();  //启发式因子改变，随即更新openlist

  //*****动态ARA*算法所需函数*****//

  void UpdataMapInfo();  //获得当前节点的neighbors，但仅仅包含障碍信息

  //判断是否走到了终点
  inline bool ArriveGoal() const { return current_start_ == goal_pos_; }

  //沿着当前path前进一步
  inline void StartMove() {
    current_start_ = current_path_.back();
    current_path_.pop_back();
  }
  //判断要移动的下一个点是否为障碍物
  inline bool NextStepIsInObstacleList() {
    return IsInList(current_path_.back(), current_obstacle_list_);
  }
  //打印计数结果
  void PrintCountResult();

  //**********获取类内私有成员函数**********//
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::vector<Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::vector<Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::vector<Points> get_current_path() const { return current_path_; }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }

 private:
  // map info
  int16_t row_, column_;                   // map的行、列数
  Points start_pos_, goal_pos_;            // map的起点与终点
  std::vector<Points> map_obstacle_list_;  //所有障碍物list

  //一次ARA*所需的容器
  Points current_start_;                     //当前起点
  std::vector<Points> current_path_;         //当前路径
  std::vector<CellInfo> current_open_list_;  //当前openlist,修改为vector
  std::vector<Points> current_obstacle_list_;  //当前已经获得的障碍物列表
  std::map<Points, Points> current_save_path_hash_;  //当前存储path的hash
  //当前存储已经expanded points的信息的hash
  std::map<Points, CellInfo> current_observed_cell_info_list_;

  float heuristic_factor_;               // h(s)的权重系数
  int32_t current_expand_points_count_,  //一次算法中的expand计数
      all_expand_points_count_,          //整体算法中的expand计数
      search_nums_count_;                //搜索次数计数
  //判断点是否在list中
  inline bool IsInList(const Points& point,
                       const std::vector<Points>& list) const {
    return std::find(list.begin(), list.end(), point) != list.end();
  }
  //清空当前ARA*所需的容器
  void ClearCurrentContainers();

  // openlist pop minimal elem
  CellInfo OpenLIstPopMinElem();
};

#endif