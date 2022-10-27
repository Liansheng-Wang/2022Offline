/***
 * 
 * 该文件负责达到 planner 期待的状态。
 * 担任轨迹跟踪控制器的角色
 * 输入:  轨迹
 * 输出： 控制指令
 * 
*/

#pragma once 
#include <plan/planner.h>
#include <control/actuator.h>


class Controller{
public:
  Controller(){

  }

  ~Controller(){

  }

  void init(Actuator& actuator){
    actuator_ = std::make_unique<Actuator>(actuator);
  }

private:
  Actuator::Ptr actuator_;
  




private:



};