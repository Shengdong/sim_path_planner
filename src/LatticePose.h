#ifndef LATTICE_PLANNER_LATTICEPOSE_H
#define LATTICE_PLANNER_LATTICEPOSE_H

#include <Eigen/Dense>


class LatticePose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LatticePose();
    LatticePose(const Eigen::Vector3i& position,
                const Eigen::Vector2i& yaw = Eigen::Vector2i::Zero());
    LatticePose(const LatticePose& p);

    void set(const Eigen::Vector3i& position,
             const Eigen::Vector2i& yaw = Eigen::Vector2i::Zero());

    int& x(void);
    int x(void) const;

    int& y(void);
    int y(void) const;

    int& z(void);
    int z(void) const;

    Eigen::Vector3i& position(void);
    const Eigen::Vector3i& position(void) const;

    int& xYaw(void);
    int xYaw(void) const;

    int& yYaw(void);
    int yYaw(void) const;

    bool operator==(const LatticePose& other) const;

    std::string print(void) const;

private:
    Eigen::Vector3i m_position;
    Eigen::Matrix<int,6,1> m_attitude;
};



#endif
