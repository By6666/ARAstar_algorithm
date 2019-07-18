#include "ARAstar_algorithm.h"
std::vector<std::string> sum_result;  //总结输出结果

/* 构造函数
 * 输入：地图的行数、列数、起点、终点、障碍物点
 * 输出：无
 * */
ARAstar::ARAstar(int16_t row_arg, int16_t column_arg, Points statr_arg,
                 Points goal_arg, std::vector<Points> obstacle_list_arg)
    : row_(row_arg),
      column_(column_arg),
      start_pos_(statr_arg),
      goal_pos_(goal_arg),
      map_obstacle_list_(obstacle_list_arg) {
  //初始化赋值
  current_start_ = start_pos_;  //赋值当前起点为最初的起点

  search_nums_count_ = 0;            //搜索次数初始化
  all_expand_points_count_ = 0;      //扩展点个数计数初始化
  current_expand_points_count_ = 0;  //当前扩展点计数
  move_step_nums_ = 0;               //移动步数计数
}

/* 清空一次ARA*算法计算所需的容器
 * 输入：无
 * 输出：无
 * */
void ARAstar::ClearCurrentContainers() {
  //*****清空当前openlist*****//
  current_open_list_.clear();

  //*****清空当前存储path的hash*****//
  current_save_path_hash_.clear();

  //*****清空当前记录expanded points的hash*****//
  current_observed_cell_info_list_.clear();
}

/* 将openlist中的最小元素放到末尾
 * 输入：无
 * 输出：openlist中的最小元素
 * */
CellInfo ARAstar::OpenLIstPopMinElem() {
  if (current_open_list_.empty())
    return CellInfo{Points(0, 0), INF_f::max(), INF_f::max(), INF_f::max(),
                    INF_f::max()};

  if (current_open_list_.size() < 2) return current_open_list_.back();

  std::vector<CellInfo>::iterator itr =
      std::min_element(current_open_list_.begin(), current_open_list_.end());

  CellInfo temp = *itr;
  *itr = current_open_list_.back();
  current_open_list_.back() = temp;

  return temp;
}

/* 在一次A*算法中获得当前点的四个临近点的坐标
 * 输入：当前点的坐标
 * 输出：四个临近点的坐标
 * */
std::vector<Points> ARAstar::GetNeighborsPoint(const Points& current_pos) {
  std::vector<Points> neighbors;
  // UP
  if ((current_pos.first - 1) >= 0) {
    neighbors.push_back(Points(current_pos.first - 1, current_pos.second));
  }
  // Down
  if ((current_pos.first + 1) < row_) {
    neighbors.push_back(Points(current_pos.first + 1, current_pos.second));
  }
  // Left
  if ((current_pos.second - 1) >= 0) {
    neighbors.push_back(Points(current_pos.first, current_pos.second - 1));
  }
  // Right
  if ((current_pos.second + 1) < column_) {
    neighbors.push_back(Points(current_pos.first, current_pos.second + 1));
  }
  return neighbors;
}

/* 在一次A*算法中获得当前点的四个临近点的信息
 * 输入：当前点的坐标
 * 输出：四个临近点的信息
 * */
std::vector<CellInfo> ARAstar::GetNeighborsInfo(const Points& current_pos) {
  std::vector<CellInfo> neighbors;
  std::vector<Points> neighbors_pos = GetNeighborsPoint(current_pos);

  for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
    //保证neighbors不是障碍物
    if (!IsInList(neighbors_pos[i], current_obstacle_list_)) {
      //如果临近点已经扩展过，则赋值其之前的数据
      if (current_observed_cell_info_list_.find(neighbors_pos[i]) !=
          current_observed_cell_info_list_.end()) {
        neighbors.push_back(
            {neighbors_pos[i],
             current_observed_cell_info_list_[neighbors_pos[i]].f_value_,
             current_observed_cell_info_list_[neighbors_pos[i]].h_value_,
             current_observed_cell_info_list_[neighbors_pos[i]].g_value_,
             current_observed_cell_info_list_[neighbors_pos[i]].v_value_});
      } else {  //若没被扩展过，赋为初始值
        neighbors.push_back(
            {neighbors_pos[i], INF_f::max(), 0.0f, INF_f::max(), INF_f::max()});
      }
    }
  }
  return neighbors;
}

/* 执行一次A*算法
 * 输入：当前起点与终点的信息
 * 输出：alse：搜索失败; true：搜索成功
 * */
bool ARAstar::AstarAlgorithm(CellInfo& start, CellInfo& goal) {
  std::vector<CellInfo> incons_list;     //非一致列表，初始化为空集
  std::vector<Points> close_list;        // closelist，初始化为空集
  std::vector<Points> path_result_list;  //存放本次搜索的路径

  int8_t search_successful_flg = 0, find_new_path_flg = 0;  //判断flg

  //*****搜索循环*****//
  while (goal.g_value_ > OpenLIstPopMinElem().f_value_) {
    //*****弹出当前节点*****//
    CellInfo current_cell_pos = current_open_list_.back();
    current_open_list_.pop_back();
    current_cell_pos.v_value_ = current_cell_pos.g_value_;

    //放入closelist以防重复遍历
    close_list.push_back(current_cell_pos.xoy_);

    //存储已经遍历过点的信息,用作之后的迭代
    current_observed_cell_info_list_[current_cell_pos.xoy_] = current_cell_pos;

    // get neighbors
    std::vector<CellInfo> neighbors = GetNeighborsInfo(current_cell_pos.xoy_);
    int8_t neighbor_expand_cnt = 0;

    //*****扩展点*****//
    for (int8_t i = 0; i < neighbors.size(); ++i) {
      if (neighbors[i].g_value_ > (current_cell_pos.g_value_ + 1.0f)) {
        ++neighbor_expand_cnt;
        // g(s)
        neighbors[i].g_value_ = current_cell_pos.g_value_ + 1.0f;

        // h(s)
        neighbors[i].h_value_ = DistenceToGoal(neighbors[i].xoy_);

        // f(s) = g(s) + factor * h(s)
        neighbors[i].f_value_ =
            neighbors[i].g_value_ + heuristic_factor_ * neighbors[i].h_value_;

        //保存信息
        current_observed_cell_info_list_[neighbors[i].xoy_] = neighbors[i];

        //节点链接
        current_save_path_hash_[neighbors[i].xoy_] = current_cell_pos.xoy_;

        if (!IsInList(neighbors[i].xoy_, close_list)) {
          //**找到了终点**//
          if (neighbors[i].xoy_ == goal.xoy_) {
            goal = neighbors[i];  //赋值goal
            // std::cout << "goal.g_value_ =  " << goal.g_value_ << std::endl;
            find_new_path_flg = 1;  //找到了新路经
          }
          current_open_list_.push_back(neighbors[i]);
        } else {
          incons_list.push_back(neighbors[i]);
        }
      }
    }
    if (neighbor_expand_cnt) ++current_expand_points_count_;  //扩展点自曾

    //搜索失败
    if (current_open_list_.empty()) {
      search_successful_flg = 1;
      break;
    }
  }

  //*****搜索结果判断*****//
  if (search_successful_flg) {
    std::cout << "search fail !!" << std::endl;
    return false;
  }
  // there is one shortest path to goal
  else {
    std::cout << "search successfully !!" << std::endl;
    if (find_new_path_flg) {
      std::cout << "Have found new path !!" << std::endl;
      Points node = goal.xoy_;

      //**路径回溯**//
      while (current_save_path_hash_.find(node) !=
             current_save_path_hash_.end()) {
        path_result_list.push_back(node);  // path中含有goal
        node = current_save_path_hash_[node];
      }
      current_path_.clear();
      current_path_ = path_result_list;

      //打印结果
      // PrintSearchResult();

    } else {
      std::cout << "Not found new path !!" << std::endl;
    }

    //总扩展点计数
    all_expand_points_count_ += current_expand_points_count_;

    //扩展点计数清零
    current_expand_points_count_ = 0;

    //融合openlist与incons_list
    InconsPushOpenlist(incons_list);

    return true;
  }
}

/* inconslist与openlist融合
 * 输入：inconslist
 * 输出：无
 * */
void ARAstar::InconsPushOpenlist(const std::vector<CellInfo>& incons_list_arg) {
  current_open_list_.insert(current_open_list_.end(), incons_list_arg.begin(),
                            incons_list_arg.end());
}

/* 更新openlist
 * 输入：无
 * 输出：无
 * 说明：由于heuristicfactor改变，需要对openlist中所有的元素进行更新，
 * 这里也是影响整体运行速度的重要环节
 * */
void ARAstar::UpdateOpenlisByNewFactor() {
  for (int16_t i = 0; i < current_open_list_.size(); ++i) {
    current_open_list_[i].f_value_ =
        current_open_list_[i].g_value_ +
        heuristic_factor_ * current_open_list_[i].h_value_;
  }
}

/* 一次ARA*算法
 * 输入：无
 * 输出：无
 * */
bool ARAstar::ARAstarGetPath() {
  ClearCurrentContainers();  //每次开始ARA*时都清空所需的容器

  //初始化信息
  CellInfo goal({goal_pos_, INF_f::max(), 0.0f, INF_f::max(), INF_f::max()});
  CellInfo start({current_start_, 0.0f, 0.0f, 0.0f, INF_f::max()});
  start.h_value_ = DistenceToGoal(start.xoy_);
  start.f_value_ = heuristic_factor_ * start.h_value_;
  heuristic_factor_ = 10.0f;  //初始化启发式乘子

  current_open_list_.push_back(start);

  //若第一次没找到path，说明没有从起点到终点的path
  if (!AstarAlgorithm(start, goal)) {
    return false;
  }

  //循环进行A*搜索
  while (heuristic_factor_ > 1.0f) {
    //递减启发式因子
    DecreaseHeuristicFactor(goal);

    //启发式因子改变，随即更新openlist
    UpdateOpenlisByNewFactor();

    //执行一次A*算法
    AstarAlgorithm(start, goal);
  }
  ++search_nums_count_;
  return true;
}

/* 打印一次搜索的结果
 * 输入：无
 * 输出：无
 * */
void ARAstar::PrintSearchResult() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (current_start_.first == i && current_start_.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInList(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

/* 在整体ARA*算法中获得当前起点的四个临近点的信息
 * 输入：无
 * 输出：无
 * */
void ARAstar::UpdataMapInfo() {
  // UP
  if ((current_start_.first - 1) >= 0) {
    if (IsInList(Points(current_start_.first - 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    if (IsInList(Points(current_start_.first + 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    if (IsInList(Points(current_start_.first, current_start_.second - 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    if (IsInList(Points(current_start_.first, current_start_.second + 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
}

/* 打印计数结果
 * 输入：无
 * 输出：无
 *  */
void ARAstar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}

/* 进行一次总体的ARA*算法
 * 输入：文件序号
 * 输出：无
 *  */
void SearchOneMap(int map_num_) {
  //**获得map信息**//
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  //数据传入，构造ARA*算法对象
  ARAstar ARAstar_algorithm(map_info.get_grid_rows(),
                            map_info.get_grid_columns(),
                            map_info.get_start_pos(), map_info.get_goal_pos(),
                            map_info.get_obstacle_pos());

  while (1) {
    //规划路径
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << ARAstar_algorithm.get_search_nums() + 1
              << std::endl;
    bool flg =
        ARAstar_algorithm.ARAstarGetPath();  //以当前起点为起点进行一次路径规划

    //**如果失败，结束循环
    if (!flg) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      ARAstar_algorithm.PrintCountResult();
      break;
    }

    //**当前起点go along current path
    while (!ARAstar_algorithm.get_current_path().empty()) {
      //更新当前点各临近点的信息
      ARAstar_algorithm.UpdataMapInfo();

      if (ARAstar_algorithm.NextStepIsInObstacleList()) {
        break;  //当前点要移动到的下一个点是obstacle
      } else {
        ARAstar_algorithm.StartMove();
      }
    }

    //**走到了终点
    if (ARAstar_algorithm.ArriveGoal()) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      ARAstar_algorithm.PrintCountResult();
      break;
    }
  }

  //**结果统计
  sum_result.push_back(
      std::to_string(map_num_ + 1) + "          " +
      std::to_string(ARAstar_algorithm.get_search_nums()) + "          " +
      std::to_string(ARAstar_algorithm.get_all_expand_nums()) + "          " +
      std::to_string(ARAstar_algorithm.get_move_step_nums()));
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;
  for (int16_t i = 0; i < sum_result.size(); ++i) {
    std::cout << sum_result[i] << std::endl;
  }
}