#ifndef RVO_AGENT_H_ //防止头文件被重复包含，如果RVO_AGENT_H_未定义，则定义它并继续编译。
#define RVO_AGENT_H_ //如果已定义，则跳过文件，内容，避免重复定义错误。

/**
 * \file Agent.h
 * \brief Contains the Agent class
 */
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include "Definitions.h"
#include "RVOSimulator.h"
#define TIME_STEP 0.15f
#define COLLISION_RADIUS 1.0f
#define MAX_SPEED 0.8f
//将Agent类及相关功能封装到RVO命名空间中，避免与其他库冲突。
namespace RVO {
    class Agent {
    public:
        Agent(const std::string &id, float radius);//初始化对象，赋予标识符id和物理半径radius。

        void prepareStep(const gazebo_msgs::ModelStates &neighbors, const geometry_msgs::Twist &pref);//收集当前智能体的邻居信息并设置期望速度，为后续的避障计算做准备。

        std::string computeNewVelocity();//计算智能体的新速度.

        size_t addObstacle(const std::vector<Vector2> &vertices);//将一个多边形障碍物添加到智能体的障碍物列表中，用于后续避撞，输入是一组顶点（Vector2 二维向量）定义的多边形障碍物；输出：返回障碍物的唯一标识符（索引），用于后续引用或删除障碍物

        void insertObstacleNeighbor(Obstacle *obstacle);

        size_t maxNeighbors_;//最大邻居数量
        float maxSpeed_;//最大速度
        float neighborDist_;//邻居检测半径：多少米以内才算邻居
        float radius_;//智能体的碰撞半径
        float timeHorizon_;//普通避障时间范围
        float timeHorizonObst_;//障碍物避障时间范围
        std::string id_;

        Vector2 position_;//当前无人机的位置
        Vector2 velocity_;//当其无人机的速度
        Vector2 prefVelocity_;//当前无人机正要飞往的位置

        std::vector<Obstacle*> obstacles; //定义障碍物的顶点
        std::vector<std::pair<float,const Obstacle *>>obstacleNeighbors_;//存储当前智能体附近的障碍物边及其距离信息

        std::vector<std::pair<float, const Agent *>>agentNeighbors_;

        Vector2 newVelocity_;
        std::vector<Line> orcaLines_;

    };

	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
        float radius, const Vector2 &optVelocity,
        bool directionOpt, Vector2 &result);

    /**
    * \relates    Agent
    * \brief      Solves a two-dimensional linear program subject to linear
    *             constraints defined by lines and a circular constraint.
    * \param      lines         Lines defining the linear constraints.
    * \param      radius        The radius of the circular constraint.
    * \param      optVelocity   The optimization velocity.
    * \param      directionOpt  True if the direction should be optimized.
    * \param      result        A reference to the result of the linear program.
    * \return     The number of the line it fails on, and the number of lines if successful.
    */
    size_t linearProgram2(const std::vector<Line> &lines, float radius,
            const Vector2 &optVelocity, bool directionOpt,
            Vector2 &result);

    /**
    * \relates    Agent
    * \brief      Solves a two-dimensional linear program subject to linear
    *             constraints defined by lines and a circular constraint.
    * \param      lines         Lines defining the linear constraints.
    * \param      numObstLines  Count of obstacle lines.
    * \param      beginLine     The line on which the 2-d linear program failed.
    * \param      radius        The radius of the circular constraint.
    * \param      result        A reference to the result of the linear program.
    */
    void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
            float radius, Vector2 &result);
}









#endif